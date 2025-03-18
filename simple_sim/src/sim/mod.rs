mod pool;

use std::{
    f32::consts::PI,
    mem,
    sync::{mpsc, Arc},
    time::{Duration, Instant},
};

use botbrain::{
    self, behaviors::{Behavior, BehaviorFn}, scaled_grid::ScaledGrid, shapes::Shape, CamData, CamPoint, Control, RobotId, RobotParameters, RobotPose
};
use eframe::egui::{Pos2, Vec2};
use pool::ThreadPool;
use serde::Serialize;

use crate::{
    scenario::{Scenario, TrialData},
    world::{Cell, World},
};

const SPEED_MULTIPLIER: f32 = 1.0;
const STEER_MULTIPLIER: f32 = 1.0;

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

const SIMULATION_DT: f32 = 1.0 / 60.0;

const COVERAGE_GRID_SCALE: f32 = 0.1;

pub fn run_scenario_headless(
    mut sim: Simulator,
    scenario: Scenario,
    print_interval: f64,
) -> TrialData {
    let mut trial_data = TrialData::new();

    let start_time = Instant::now();
    let mut last_print = start_time;

    let width = sim.world().width();
    let height = sim.world().height();
    let total_area = width * height;
    let mut coverage_grid = ScaledGrid::<bool>::new(width, height, COVERAGE_GRID_SCALE);

    while sim.state.time.as_secs_f32() < scenario.duration {
        if (Instant::now() - last_print).as_secs_f64() > print_interval {
            last_print = Instant::now();
            println!("Time: {:.1}s", sim.state.time.as_secs_f64());
        }

        let covered_cells = coverage_grid
            .iter()
            .filter(|(_, _, &covered)| covered)
            .count() as f32;
        let covered_area = covered_cells * COVERAGE_GRID_SCALE;
        let coverage = covered_area / total_area;

        trial_data.add_state(sim.state.clone(), coverage);

        sim.step();

        for msg in &sim.pending_messages {
            match &msg.kind {
                botbrain::MessageKind::ShapeDiff { shape, diff: _ } => {
                    coverage_grid.set_shape(shape, true)
                }
                botbrain::MessageKind::CamDiff {
                    cone,
                    lidar: _,
                    diff: _,
                    // TODO: use lidar data
                } => coverage_grid.set_cone(cone, true),
                botbrain::MessageKind::Debug(_) => {}
            }
        }
    }

    println!(
        "Simulation finished after {:.2} seconds",
        sim.state.time.as_secs_f32()
    );

    println!();
    println!("Robot poses:");
    for agent in sim.state.agents {
        let RobotPose { pos, angle } = agent.state.pose;
        println!("  pos: {:?}, angle: {:.2}", pos, angle);
    }

    trial_data
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct AgentState {
    pub pose: RobotPose,
    pub vel: f32,
    pub avel: f32,
}

#[derive(Serialize)]
pub struct Agent {
    pub state: AgentState,
    pub control: botbrain::Control,

    #[serde(skip)]
    pub robot: Box<dyn botbrain::Robot>,
}

impl Clone for Agent {
    fn clone(&self) -> Self {
        Self {
            state: self.state.clone(),
            robot: self.robot.clone_box(),
            control: self.control.clone(),
        }
    }
}

#[derive(Clone, Serialize)]
pub struct SimulatorState {
    pub agents: Vec<Agent>,
    pub time: Duration,
}

pub struct SimArgs {
    pub world: World,
    pub behavior: Behavior,
    pub threads: usize,
}

pub struct Simulator {
    pub state: SimulatorState,
    world: World,
    pool: ThreadPool<(Agent, Arc<StepArgs>), Agent>,
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

        let state = SimulatorState {
            agents: vec![],
            time: Duration::default(),
        };
        Self {
            state,
            world,
            pool: ThreadPool::new(threads, |(mut agent, args): (Agent, Arc<StepArgs>)| {
                step_agent(&mut agent, args);
                agent
            }),
            pending_messages: vec![],
            behavior,
            dt: SIMULATION_DT,
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    pub fn add_robot(&mut self, robot_pose: RobotPose) {
        let mut agent = Agent {
            state: AgentState {
                pose: robot_pose.clone(),
                ..Default::default()
            },
            control: Control::default(),
            robot: self.behavior.create_robot(),
        };

        agent.robot.input_pose(robot_pose);

        let id = self.state.agents.len() as u32;
        agent.robot.set_id(RobotId::new(id));
        agent.robot.set_world_size(self.world.size());
        agent.robot.get_debug_soup_mut().activate();
        self.state.agents.push(agent);
    }

    pub fn step(&mut self) {
        let dt = self.dt;

        let (msg_send_tx, msg_send_rx) = mpsc::channel();

        let args = Arc::new(StepArgs {
            agents: self.state.agents.clone(),
            behavior_fn: self.behavior.behavior_fn(),
            world: self.world.clone(),
            time: self.state.time,
            dt,
            msg_send_tx,
            pending_messages: mem::take(&mut self.pending_messages),
        });

        let agents = mem::replace(&mut self.state.agents, Vec::with_capacity(0));

        let input = agents.into_iter().map(|a| (a, args.clone())).collect();
        self.state.agents = self.pool.process(input);

        // Collect all pending messages
        self.pending_messages = msg_send_rx.try_iter().collect::<Vec<botbrain::Message>>();

        self.state.time += Duration::from_secs_f32(dt);
    }
}

/// Shared state needed to step an agent forward in time
struct StepArgs {
    agents: Vec<Agent>,
    behavior_fn: BehaviorFn,
    world: World,
    time: Duration,
    dt: f32,
    msg_send_tx: mpsc::Sender<botbrain::Message>,
    pending_messages: Vec<botbrain::Message>,
}

fn step_agent(agent: &mut Agent, args: Arc<StepArgs>) {
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

    let RobotParameters {
        cam_fov,
        cam_range,
        diameter,
        lidar_range,
        ..
    } = *agent.robot.params();

    {
        // Call the behavior function
        agent.control = behavior_fn(&mut agent.robot, *time);
        agent.state.vel = agent.control.speed * SPEED_MULTIPLIER;
        agent.state.avel = agent.control.steer * STEER_MULTIPLIER;

        // Update position of the robot
        let vel = Vec2::angled(agent.state.pose.angle) * agent.state.vel;
        agent.state.pose.pos += vel * dt;
        agent.state.pose.angle += agent.state.avel * dt;

        // Set the new state of the robot
        agent.robot.input_pose(RobotPose {
            pos: agent.state.pose.pos,
            angle: agent.state.pose.angle,
        });
    }

    // Update postboxes
    {
        let robot_id = *agent.robot.id();
        let postbox = agent.robot.get_postbox_mut();

        // Send messages
        for msg in postbox.empty() {
            msg_send_tx.send(msg).unwrap();
        }

        // Receive messages
        postbox.deposit(
            pending_messages
                .iter()
                .filter(|m| m.sender_id != robot_id)
                .cloned(),
        );
    }

    // Update lidar data
    {
        let robot = &mut agent.robot;
        let points = (0..LIDAR_RAYS)
            .map(|n| {
                let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                let (distance, _) = cast_ray(
                    world,
                    agents,
                    agent.state.pose.pos,
                    agent.state.pose.angle + angle,
                    diameter / 2.0,
                    lidar_range,
                    &[Cell::SearchItem],
                );
                botbrain::LidarPoint { angle, distance }
            })
            .collect();

        robot.input_lidar(botbrain::LidarData::new(points));
    }

    // Update robot camera
    {
        let robot = &mut agent.robot;
        let angle_step = cam_fov / (CAMERA_RAYS - 1) as f32;
        let points = (0..CAMERA_RAYS)
            .filter_map(|n| {
                let angle = n as f32 * angle_step - cam_fov / 2.0;
                let (distance, cell) = cast_ray(
                    world,
                    agents,
                    agent.state.pose.pos,
                    agent.state.pose.angle + angle,
                    diameter / 2.0,
                    cam_range,
                    &[],
                );
                match cell {
                    Some(Cell::SearchItem) => {
                        let propability = (cam_range - distance) / cam_range;
                        Some((n, botbrain::CamPoint { angle, propability }))
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
                        propability: 0.0,
                    },
                    |a: CamPoint, (_, b): &(usize, CamPoint)| CamPoint {
                        angle: a.angle + b.angle,
                        propability: a.propability + b.propability,
                    },
                );
                avg_point.propability /= adjacant.len() as f32;
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
            robot.input_cam(CamData(sparse_points));
        }
    }

    // Resolve collisions
    resolve_robot_collisions(agent, agents);
    resolve_border_collisions(agent, world);
    resolve_world_collisions(agent, world);
}

fn cast_ray(
    world: &World,
    agents: &[Agent],
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
            for agent in agents {
                let d = (agent.state.pose.pos - pos).length();
                if d < robot_radius {
                    return (distance, None);
                }
            }
        }
        distance += step_size;
    }
    (distance - step_size / 2.0, None)
}

fn resolve_robot_collisions(me: &mut Agent, agents: &[Agent]) {
    for other in agents {
        if other.robot.id() == me.robot.id() {
            continue;
        }
        let diameter = f32::max(me.robot.params().diameter, other.robot.params().diameter);
        if (me.state.pose.pos - other.state.pose.pos).length() < diameter {
            let diff = me.state.pose.pos - other.state.pose.pos;
            let overlap = diameter - diff.length();
            let dir = diff.normalized() * overlap / 2.0;
            me.state.pose.pos += dir;
        }
    }
}

fn resolve_world_collisions(me: &mut Agent, world: &World) {
    // Look in a circle around the robot
    let radius = me.robot.params().diameter / 2.0 * 1.4 / world.scale();
    let robot_diameter = me.robot.params().diameter;
    let center = world.world_to_grid(me.state.pose.pos);
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
            let diff = me.state.pose.pos - cell_center;

            let overlap = robot_diameter / 2.0 - diff.length() + 0.5 * world.scale();
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
        me.state.pose.pos += nudge / nudgers as f32;
    }
}

fn resolve_border_collisions(me: &mut Agent, world: &World) {
    let diameter = me.robot.params().diameter;
    let pos = &mut me.state.pose.pos;
    let radius = diameter / 2.0;
    if pos.x - radius < -world.width() / 2.0 {
        pos.x = -world.width() / 2.0 + radius;
    }
    if pos.x + radius > world.width() / 2.0 {
        pos.x = world.width() / 2.0 - radius;
    }
    if pos.y - radius < -world.height() / 2.0 {
        pos.y = -world.height() / 2.0 + radius;
    }
    if pos.y + radius > world.height() / 2.0 {
        pos.y = world.height() / 2.0 - radius;
    }
}
