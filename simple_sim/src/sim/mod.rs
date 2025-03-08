mod pool;

use std::{
    collections::HashMap,
    f32::consts::PI,
    mem,
    sync::{mpsc, Arc},
    time::{Duration, Instant},
};

use eframe::egui::{Pos2, Vec2};
use pool::ThreadPool;
use robcore::{
    self, behaviors::BehaviorFn, debug::DebugType, grid::iter_circle, scaled_grid::ScaledGrid,
    CamData, CamPoint,
};

use crate::world::{Cell, World};

const SPEED_MULTIPLIER: f32 = 1.0;
const STEER_MULTIPLIER: f32 = 1.0;

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

const SEARCH_GRID_SCALE: f32 = 0.20;

const SIMULATION_DT: f32 = 1.0 / 60.0;

#[derive(Clone)]
pub struct Agent {
    pub robot: robcore::Robot,
    pub control: robcore::Control,
}

impl Agent {
    pub fn new_at(pos: Pos2, angle: f32) -> Self {
        let robot = robcore::Robot {
            pos,
            angle,
            ..Default::default()
        };
        Self {
            robot,
            control: robcore::Control::default(),
        }
    }

    pub fn pos(&self) -> Pos2 {
        self.robot.pos
    }
}

#[derive(Clone)]
pub struct SimulatorState {
    pub agents: Vec<Agent>,
    pub world: World,
    pub sps: f32,
    pub time: f32,
}

pub struct Simulator {
    pub state: SimulatorState,
    pool: ThreadPool<(Agent, Arc<StepArgs>), Agent>,
    pending_messages: Vec<robcore::Message>,
    debug_channels: Vec<mpsc::Receiver<DebugType>>,
    behavior: BehaviorFn,
    start_time: Instant,
    dt: f32,
}

impl Simulator {
    pub fn new(world: World, sps: f32, behavior: BehaviorFn, threads: usize) -> Self {
        let state = SimulatorState {
            agents: vec![],
            world,
            sps,
            time: 0.0,
        };
        Self {
            state,
            pool: ThreadPool::new(threads, |(mut agent, args): (Agent, Arc<StepArgs>)| {
                step_agent(&mut agent, args);
                agent
            }),
            pending_messages: vec![],
            debug_channels: vec![],
            behavior,
            start_time: Instant::now(),
            dt: SIMULATION_DT,
        }
    }

    pub fn add_robot(&mut self, mut agent: Agent) {
        let id = self.state.agents.len() as u32;
        agent.robot.id = robcore::RobotId::new(id);
        agent.robot.search_grid = ScaledGrid::new(
            self.state.world.width(),
            self.state.world.height(),
            SEARCH_GRID_SCALE,
        );
        agent.robot.debug_soup = Some(HashMap::new());
        self.state.agents.push(agent);
    }

    pub fn step(&mut self) {
        let dt = self.dt;

        let (msg_send_tx, msg_send_rx) = mpsc::channel();

        let args = Arc::new(StepArgs {
            agents: self.state.agents.clone(),
            world: self.state.world.clone(),
            time: self.start_time + Duration::from_secs_f32(self.state.time),
            behavior: self.behavior,
            dt,
            msg_send_tx,
            pending_messages: mem::take(&mut self.pending_messages),
        });

        let agents = mem::replace(&mut self.state.agents, Vec::with_capacity(0));
        let input = agents.into_iter().map(|a| (a, args.clone())).collect();
        self.state.agents = self.pool.process(input);

        // Collect all pending messages
        self.pending_messages = msg_send_rx.try_iter().collect::<Vec<robcore::Message>>();

        self.state.time += dt;
    }
}

/// Shared state needed to step an agent forward in time
struct StepArgs {
    agents: Vec<Agent>,
    world: World,
    time: Instant,
    behavior: BehaviorFn,
    dt: f32,
    msg_send_tx: mpsc::Sender<robcore::Message>,
    pending_messages: Vec<robcore::Message>,
}

fn step_agent(agent: &mut Agent, args: Arc<StepArgs>) {
    let StepArgs {
        agents,
        world,
        time,
        behavior,
        dt,
        msg_send_tx,
        pending_messages,
    } = &*args;

    let dt = *dt;

    // Call the behavior function
    agent.control = (behavior)(&mut agent.robot, *time);
    agent.robot.vel = agent.control.speed * SPEED_MULTIPLIER;
    agent.robot.avel = agent.control.steer * STEER_MULTIPLIER;

    // Update position of the robot
    let vel = Vec2::angled(agent.robot.angle) * agent.robot.vel;
    agent.robot.pos += vel * dt;
    agent.robot.angle += agent.robot.avel * dt;

    // Send messages
    for msg in agent.robot.outgoing_msg.drain(..) {
        msg_send_tx.send(msg).unwrap();
    }

    // Receive messages
    agent.robot.incoming_msg.extend(
        pending_messages
            .iter()
            .filter(|m| m.sender_id != agent.robot.id)
            .cloned(),
    );

    // Update lidar data
    {
        let robot = &mut agent.robot;
        let points = (0..LIDAR_RAYS)
            .map(|n| {
                let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                let (distance, _) = cast_ray(
                    world,
                    agents,
                    robot.pos,
                    robot.angle + angle,
                    robot.params.diameter / 2.0,
                    robot.params.lidar_range,
                    &[Cell::SearchItem],
                );
                robcore::LidarPoint { angle, distance }
            })
            .collect();
        robot.lidar = robcore::LidarData::new(points);
    }

    // Update robot camera
    {
        let robot = &mut agent.robot;
        let angle_step = robot.params.cam_fov / (CAMERA_RAYS - 1) as f32;
        let max_camera_range = robot.params.cam_range;
        let points = (0..CAMERA_RAYS)
            .filter_map(|n| {
                let angle = n as f32 * angle_step - robot.params.cam_fov / 2.0;
                let (distance, cell) = cast_ray(
                    world,
                    agents,
                    robot.pos,
                    robot.angle + angle,
                    robot.params.diameter / 2.0,
                    max_camera_range,
                    &[],
                );
                match cell {
                    Some(Cell::SearchItem) => {
                        let propability = (max_camera_range - distance) / max_camera_range;
                        Some((n, robcore::CamPoint { angle, propability }))
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
            robot.cam = CamData(sparse_points);
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
            for robot in agents {
                let d = (robot.pos() - pos).length();
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
        if other.robot.id == me.robot.id {
            continue;
        }
        let diameter = f32::max(me.robot.params.diameter, other.robot.params.diameter);
        if (me.pos() - other.pos()).length() < diameter {
            let diff = me.pos() - other.pos();
            let overlap = diameter - diff.length();
            let dir = diff.normalized() * overlap / 2.0;
            me.robot.pos += dir;
        }
    }
}

fn resolve_world_collisions(me: &mut Agent, world: &World) {
    // Look in a circle around the robot
    let radius = me.robot.params.diameter / 2.0 * 1.4 / world.scale();

    let center = world.world_to_grid(me.pos());
    let mut nudge = Vec2::ZERO;
    let mut nudgers = 0;
    for (x, y) in iter_circle(center, radius) {
        let cell = world.grid().get(x, y);
        if matches!(cell, Some(Cell::Wall) | None) {
            let cell_center = world.grid_to_world(Pos2 {
                x: x as f32,
                y: y as f32,
            });
            let diff = me.pos() - cell_center;

            let overlap = me.robot.params.diameter / 2.0 - diff.length() + 0.5 * world.scale();
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
        me.robot.pos += nudge / nudgers as f32;
    }
}

fn resolve_border_collisions(me: &mut Agent, world: &World) {
    let pos = &mut me.robot.pos;
    let radius = me.robot.params.diameter / 2.0;
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
