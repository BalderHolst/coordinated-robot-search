mod pool;

use std::{
    f32::consts::PI,
    mem,
    sync::{mpsc, Arc},
    time::{Duration, Instant},
};

use botbrain::{
    self,
    behaviors::{Behavior, BehaviorFn},
    params::{CAM_FOV, CAM_RANGE, DIAMETER, LIDAR_RANGE, RADIUS},
    scaled_grid::ScaledGrid,
    CamData, CamPoint, Control, RobotId, RobotPose,
};
use eframe::egui::{Pos2, Vec2};
use polars::prelude::*;
use pool::ThreadPool;
use serde::Serialize;

use crate::{
    cli::ScenarioArgs,
    scenario::Scenario,
    world::{Cell, World},
};

const SPEED_MULTIPLIER: f32 = 1.0;
const STEER_MULTIPLIER: f32 = 1.0;

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

const SIMULATION_DT: f32 = 1.0 / 60.0;

const COVERAGE_GRID_SCALE: f32 = 0.1;

#[derive(Clone, Default)]
struct RobotData {
    x: Vec<f32>,
    y: Vec<f32>,
    angle: Vec<f32>,
    vel: Vec<f32>,
    avel: Vec<f32>,
    steer: Vec<f32>,
    speed: Vec<f32>,
}

impl RobotData {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            x: Vec::with_capacity(capacity),
            y: Vec::with_capacity(capacity),
            angle: Vec::with_capacity(capacity),
            vel: Vec::with_capacity(capacity),
            avel: Vec::with_capacity(capacity),
            steer: Vec::with_capacity(capacity),
            speed: Vec::with_capacity(capacity),
        }
    }

    fn push_state(&mut self, state: &RobotState, control: &Control) {
        self.x.push(state.pose.pos.x);
        self.y.push(state.pose.pos.y);
        self.angle.push(state.pose.angle);
        self.vel.push(state.vel);
        self.avel.push(state.avel);
        self.steer.push(control.steer);
        self.speed.push(control.speed);
    }

    fn into_cols(self, prefix: &str) -> Vec<Column> {
        vec![
            Column::new(format!("{}x", prefix).into(), self.x),
            Column::new(format!("{}y", prefix).into(), self.y),
            Column::new(format!("{}angle", prefix).into(), self.angle),
            Column::new(format!("{}vel", prefix).into(), self.vel),
            Column::new(format!("{}avel", prefix).into(), self.avel),
            Column::new(format!("{}steer", prefix).into(), self.steer),
            Column::new(format!("{}speed", prefix).into(), self.speed),
        ]
    }
}

pub fn run_scenario_headless(
    mut sim: Simulator,
    scenario: Scenario,
    scenario_args: ScenarioArgs,
) -> DataFrame {
    let steps = (scenario.duration / SIMULATION_DT).ceil() as usize;

    let mut time_data = Vec::with_capacity(steps);
    let mut robot_data = vec![RobotData::with_capacity(steps); scenario.robots.len()];
    let mut coverage_data = Vec::with_capacity(steps);

    let start_time = Instant::now();
    let mut last_print = start_time;

    loop {
        if (Instant::now() - last_print).as_secs_f64() > scenario_args.print_interval {
            last_print = Instant::now();
            println!("Time: {:.1}s", sim.state.time.as_secs_f64());
        }

        time_data.push(sim.state.time.as_secs_f64());

        // Calculate and store coverage
        let coverage = sim.state.diagnostics.coverage();
        coverage_data.push(coverage);

        // Store robot data
        for (n, robot_state) in sim.state.robot_states.iter().enumerate() {
            robot_data[n].push_state(robot_state, &robot_state.control);
        }

        if sim.state.time.as_secs_f32() >= scenario.duration {
            break;
        }

        sim.step();
    }

    println!(
        "Simulation finished after {:.2} seconds",
        sim.state.time.as_secs_f32()
    );

    println!();
    println!("Robot poses:");
    for agent in sim.state.robot_states {
        let RobotPose { pos, angle } = agent.pose;
        println!("  pos: {:?}, angle: {:.2}", pos, angle);
    }

    let robot_cols = robot_data
        .into_iter()
        .enumerate()
        .flat_map(|(n, data)| data.into_cols(&format!("robot_{}/", n)))
        .collect::<Vec<_>>();

    let cols = vec![
        Column::new("time".into(), time_data),
        Column::new("coverage".into(), coverage_data),
    ]
    .into_iter()
    .chain(robot_cols)
    .collect::<Vec<_>>();

    DataFrame::new(cols).unwrap()
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct RobotState {
    pub id: RobotId,
    pub pose: RobotPose,
    pub vel: f32,
    pub avel: f32,
    pub control: botbrain::Control,
}

#[derive(Clone)]
pub struct SimState {
    pub robot_states: Vec<RobotState>,
    pub time: Duration,
    pub diagnostics: SimDiagnostics,
}

#[derive(Clone)]
pub struct SimDiagnostics {
    pub coverage_grid: ScaledGrid<bool>,
}

impl SimDiagnostics {
    pub fn coverage(&self) -> f32 {
        let total_area = self.coverage_grid.width() * self.coverage_grid.height();
        let covered_cells = self
            .coverage_grid
            .iter()
            .filter(|(_, _, &covered)| covered)
            .count();
        let covered_area = covered_cells as f32 * self.coverage_grid.scale().powi(2);
        covered_area / total_area
    }
}

pub struct SimArgs {
    pub world: World,
    pub behavior: Behavior,
    pub threads: usize,
}

#[allow(clippy::type_complexity)]
pub struct Simulator {
    pub state: SimState,
    robots: Vec<Box<dyn botbrain::Robot + 'static>>,
    world: World,
    pool: ThreadPool<
        (RobotState, Box<dyn botbrain::Robot>, Arc<StepArgs>),
        (RobotState, Box<dyn botbrain::Robot>),
    >,
    pending_messages: Vec<botbrain::Message>,
    behavior: Behavior,
    dt: f32,
}

impl Simulator {
    pub fn new(args: SimArgs) -> Self {
        let SimArgs {
            world,
            behavior,
            threads,
        } = args;

        let state = SimState {
            robot_states: vec![],
            time: Duration::default(),
            diagnostics: SimDiagnostics {
                coverage_grid: ScaledGrid::new(world.width(), world.height(), COVERAGE_GRID_SCALE),
            },
        };
        Self {
            state,
            robots: vec![],
            world,
            pool: ThreadPool::new(
                threads,
                |(mut agent_state, mut robot, args): (
                    RobotState,
                    Box<dyn botbrain::Robot>,
                    Arc<StepArgs>,
                )| {
                    step_agent(&mut agent_state, &mut robot, args);
                    (agent_state, robot)
                },
            ),
            pending_messages: vec![],
            behavior,
            dt: SIMULATION_DT,
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    pub fn robots(&self) -> &[Box<dyn botbrain::Robot>] {
        &self.robots
    }

    pub fn add_robot(&mut self, robot_pose: RobotPose) {
        let id = RobotId::new(self.state.robot_states.len() as u32);

        // Add the robot to the state which is managed by the simulator
        let agent_state = RobotState {
            id,
            pose: robot_pose.clone(),
            ..Default::default()
        };
        self.state.robot_states.push(agent_state);

        // Create a new robot and add it to the simulator. This calculates
        // the robot's movement and behavior
        let mut robot = self.behavior.create_robot();
        robot.input_pose(robot_pose);
        robot.set_id(id);
        robot.set_world_size(self.world.size());
        robot.get_debug_soup_mut().activate();
        self.robots.push(robot);
    }

    pub fn step(&mut self) {
        let dt = self.dt;

        let (msg_send_tx, msg_send_rx) = mpsc::channel();

        let args = Arc::new(StepArgs {
            agents: self.state.robot_states.clone(),
            behavior_fn: self.behavior.behavior_fn(),
            world: self.world.clone(),
            time: self.state.time,
            dt,
            msg_send_tx,
            pending_messages: mem::take(&mut self.pending_messages),
        });

        // Move the agents and robots to the thread pool
        let agents = mem::replace(&mut self.state.robot_states, Vec::with_capacity(0));
        let robots: Vec<Box<dyn botbrain::Robot + 'static>> =
            mem::replace(&mut self.robots, Vec::with_capacity(0));

        // Construct the input for the thread pool function
        let input = agents
            .into_iter()
            .zip(robots)
            .map(|(a, r)| (a, r, args.clone()))
            .collect::<Vec<_>>();

        // Process the agents and robots in parallel
        let (states, robots) = self.pool.process(input).into_iter().unzip();

        // Move the agents and robots back to the simulator
        self.state.robot_states = states;
        self.robots = robots;

        // Collect all pending messages
        self.pending_messages = msg_send_rx.try_iter().collect::<Vec<botbrain::Message>>();

        // Update diagnostics
        let diagnostics = &mut self.state.diagnostics;
        for msg in &self.pending_messages {
            match &msg.kind {
                botbrain::MessageKind::ShapeDiff { shape, diff: _ } => {
                    diagnostics.coverage_grid.set_shape(shape, true)
                }
                botbrain::MessageKind::CamDiff {
                    cone,
                    lidar: _,
                    diff: _,
                } => diagnostics.coverage_grid.set_cone(cone, true),
                botbrain::MessageKind::Debug(_) => {}
            }
        }

        self.state.time += Duration::from_secs_f32(dt);
    }
}

/// Shared state needed to step an agent forward in time
struct StepArgs {
    agents: Vec<RobotState>,
    behavior_fn: BehaviorFn,
    world: World,
    time: Duration,
    dt: f32,
    msg_send_tx: mpsc::Sender<botbrain::Message>,
    pending_messages: Vec<botbrain::Message>,
}

fn step_agent(
    agent_state: &mut RobotState,
    robot: &mut Box<dyn botbrain::Robot>,
    args: Arc<StepArgs>,
) {
    let StepArgs {
        agents,
        behavior_fn,
        world,
        time,
        dt,
        msg_send_tx,
        pending_messages,
    } = &*args;

    let dt = *dt;

    // Call the behavior function
    let (control, msgs) = behavior_fn(robot, *time);

    {
        agent_state.control = control;
        agent_state.vel = agent_state.control.speed * SPEED_MULTIPLIER;
        agent_state.avel = agent_state.control.steer * STEER_MULTIPLIER;

        // Update position of the robot
        let vel = Vec2::angled(agent_state.pose.angle) * agent_state.vel;
        agent_state.pose.pos += vel * dt;
        agent_state.pose.angle += agent_state.avel * dt;

        // Set the new state of the robot
        robot.input_pose(RobotPose {
            pos: agent_state.pose.pos,
            angle: agent_state.pose.angle,
        });
    }

    // Update postboxes
    {
        // Send messages
        for msg in msgs {
            msg_send_tx.send(msg).unwrap();
        }

        // Receive messages
        robot.input_msgs(pending_messages.clone());
    }

    // Update lidar data
    {
        let points = (0..LIDAR_RAYS)
            .map(|n| {
                let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                let (distance, _) = cast_ray(
                    world,
                    agents,
                    agent_state.pose.pos,
                    agent_state.pose.angle + angle,
                    DIAMETER / 2.0,
                    LIDAR_RANGE,
                    &[Cell::SearchItem],
                );
                botbrain::LidarPoint { angle, distance }
            })
            .collect();

        robot.input_lidar(botbrain::LidarData::new(points));
    }

    // Update robot camera
    {
        let angle_step = CAM_FOV / (CAMERA_RAYS - 1) as f32;
        let points = (0..CAMERA_RAYS)
            .filter_map(|n| {
                let angle = n as f32 * angle_step - CAM_FOV / 2.0;
                let (distance, cell) = cast_ray(
                    world,
                    agents,
                    agent_state.pose.pos,
                    agent_state.pose.angle + angle,
                    DIAMETER / 2.0,
                    CAM_RANGE,
                    &[],
                );
                match cell {
                    Some(Cell::SearchItem) => {
                        let probability = (CAM_RANGE - distance) / CAM_RANGE;
                        Some((n, botbrain::CamPoint { angle, probability }))
                    }
                    _ => None,
                }
            })
            .collect::<Vec<_>>();

        // Consolidate points next to each other
        {
            let mut sparse_points = vec![];
            let mut adjacant = vec![];

            let consolidate_points = |adjacant: &mut Vec<_>, sparse_points: &mut Vec<_>| {
                if adjacant.is_empty() {
                    return;
                }

                let mut avg_point = adjacant.iter().fold(
                    CamPoint {
                        angle: 0.0,
                        probability: 0.0,
                    },
                    |a: CamPoint, (_, b): &(usize, CamPoint)| CamPoint {
                        angle: a.angle + b.angle,
                        probability: a.probability + b.probability,
                    },
                );
                avg_point.probability /= adjacant.len() as f32;
                avg_point.angle /= adjacant.len() as f32;
                sparse_points.push(avg_point);
                adjacant.clear();
            };

            for (n, point) in points {
                match adjacant.last() {
                    None => adjacant.push((n, point)),
                    Some((last_n, _)) if *last_n == n - 1 => adjacant.push((n, point)),
                    Some(_) => consolidate_points(&mut adjacant, &mut sparse_points),
                }
            }
            consolidate_points(&mut adjacant, &mut sparse_points);
            robot.input_cam(CamData::Points(sparse_points));
        }
    }

    // Resolve collisions
    resolve_robot_collisions(agent_state, agents);
    resolve_border_collisions(agent_state, world);
    resolve_world_collisions(agent_state, world);
}

fn cast_ray(
    world: &World,
    agents: &[RobotState],
    base_pos: Pos2,
    angle: f32,
    robot_radius: f32,
    max_range: f32,
    ignore: &[Cell],
) -> (f32, Option<Cell>) {
    let step_size = world.scale() * 0.5;
    let direction = Vec2::angled(angle);

    let mut distance = robot_radius;

    while distance < max_range {
        let pos = base_pos + direction * distance;

        let cell = world.get(pos);

        // Check for collisions with the world
        if let Some(cell) = cell {
            if !cell.is_empty() && !ignore.contains(cell) {
                return (distance, Some(*cell));
            }
        } else {
            return (distance, Some(Cell::Wall));
        }

        // Check for collisions with other robots
        if distance > robot_radius {
            for robot_state in agents {
                let d = (robot_state.pose.pos - pos).length();
                if d < robot_radius {
                    return (distance, None);
                }
            }
        }
        distance += step_size;
    }
    (distance - step_size / 2.0, None)
}

fn resolve_robot_collisions(me: &mut RobotState, all: &[RobotState]) {
    for other in all {
        if me.id == other.id {
            continue;
        }
        if (me.pose.pos - other.pose.pos).length() < DIAMETER {
            let diff = me.pose.pos - other.pose.pos;
            let overlap = DIAMETER - diff.length();
            let dir = diff.normalized() * overlap / 2.0;
            me.pose.pos += dir;
        }
    }
}

fn resolve_world_collisions(robot_state: &mut RobotState, world: &World) {
    // Look in a circle around the robot
    let radius = DIAMETER / 2.0 * 1.4 / world.scale();
    let center = world.world_to_grid(robot_state.pose.pos);
    let mut nudge = Vec2::ZERO;
    let mut nudgers = 0;
    let grid = world.grid();
    for (x, y) in grid.iter_circle(center, radius) {
        let cell = grid.get(x, y);
        if matches!(cell, Some(Cell::Wall) | None) {
            let cell_center = world.grid_to_world(Pos2 {
                x: x as f32,
                y: y as f32,
            });
            let diff = robot_state.pose.pos - cell_center;

            let overlap = DIAMETER / 2.0 - diff.length() + 0.5 * world.scale();
            if overlap < 0.0 {
                continue;
            }

            let diff = match diff.x.abs() > diff.y.abs() {
                true => Vec2::new(diff.x, 0.0),
                false => Vec2::new(0.0, diff.y),
            };
            let dir = diff.normalized();

            nudge += dir * overlap;
            nudgers += 1;
        }
    }

    if nudgers > 0 {
        // Only nudge in the direction with most difference
        robot_state.pose.pos += nudge / nudgers as f32;
    }
}

fn resolve_border_collisions(robot_state: &mut RobotState, world: &World) {
    let pos = &mut robot_state.pose.pos;
    if pos.x - RADIUS < -world.width() / 2.0 {
        pos.x = -world.width() / 2.0 + RADIUS;
    }
    if pos.x + RADIUS > world.width() / 2.0 {
        pos.x = world.width() / 2.0 - RADIUS;
    }
    if pos.y - RADIUS < -world.height() / 2.0 {
        pos.y = -world.height() / 2.0 + RADIUS;
    }
    if pos.y + RADIUS > world.height() / 2.0 {
        pos.y = world.height() / 2.0 - RADIUS;
    }
}
