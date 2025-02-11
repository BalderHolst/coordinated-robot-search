use std::f32::consts::PI;

use eframe::egui::{Pos2, Vec2};
use robcore::{self, CamPoint};

use crate::{
    cli::BehaviorFn,
    world::{Cell, World, CELLS_PR_METER},
};

const LIDAR_RAYS: usize = 40;
const LIDAR_RANGE: f32 = 5.0;

const CAMERA_RAYS: usize = 20;
const CAMERA_RANGE: f32 = 3.0;
pub const CAMERA_FOV: f32 = PI / 2.0;

const RAY_CAST_STEP: f32 = 0.5 / CELLS_PR_METER;

#[derive(Clone)]
pub struct Agent {
    pub robot: robcore::Robot,
    pub control: robcore::Control,
}

impl Agent {
    pub fn new_at(x: f32, y: f32, angle: f32) -> Self {
        let robot = robcore::Robot {
            pos: Pos2 { x, y },
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
pub struct Simulator {
    pub agents: Vec<Agent>,
    pub robot_radius: f32,
    pub world: World,
    pub sps: f32,
    behavior: BehaviorFn,
    messages: Vec<robcore::Message>,
}

impl Simulator {
    pub fn new(world: World, sps: f32, behavior: BehaviorFn) -> Self {
        Self {
            agents: vec![],
            messages: vec![],
            robot_radius: 0.3,
            behavior,
            world,
            sps,
        }
    }

    fn dt(&self) -> f32 {
        1.0 / self.sps
    }

    pub fn add_robot(&mut self, mut agent: Agent) {
        let id = self.agents.len() as u32;
        agent.robot.id = robcore::RobotId::new(id);
        self.agents.push(agent);
    }

    fn cast_ray(&self, pos: Pos2, angle: f32, max_range: f32) -> (f32, Option<Cell>) {
        let mut distance = 0.0;
        let mut pos = pos;
        let step = RAY_CAST_STEP * Vec2::angled(angle);

        while distance < max_range {
            let cell = self.world.get_cell(pos);
            if !cell.is_empty() {
                return (distance, Some(cell));
            }

            // Check for collisions with other robots
            if distance > self.robot_radius {
                for robot in &self.agents {
                    if (robot.pos() - pos).length() < self.robot_radius {
                        return (distance, None);
                    }
                }
            }

            pos += step;
            distance += RAY_CAST_STEP;
        }

        (distance, None)
    }

    fn resolve_robot_collisions(&mut self) {
        for i in 0..self.agents.len() {
            for j in i + 1..self.agents.len() {
                let robot1 = &self.agents[i];
                let robot2 = &self.agents[j];
                if (robot1.pos() - robot2.pos()).length() < self.robot_radius * 2.0 {
                    let diff = robot1.pos() - robot2.pos();
                    let overlap = self.robot_radius * 2.0 - diff.length();
                    let dir = diff.normalized() * overlap / 2.0;
                    self.agents[i].robot.pos += dir;
                    self.agents[j].robot.pos -= dir;
                }
            }
        }
    }

    fn resolve_world_collisions(&mut self) {
        for robot in &mut self.agents {
            // Look in a circle around the robot
            let radius = self.robot_radius * CELLS_PR_METER * 1.4;

            let center = self.world.world_to_grid(robot.pos());
            let mut nudge = Vec2::ZERO;
            let mut nudgers = 0;
            for (x, y) in self.world.grid().circle_iter(center, radius) {
                let cell = self.world.grid().get(x, y);
                if !cell.is_empty() {
                    let cell_center = self.world.grid_to_world(Pos2 {
                        x: x as f32,
                        y: y as f32,
                    });
                    let diff = robot.pos() - cell_center;

                    let overlap = self.robot_radius - diff.length() + 0.5 / CELLS_PR_METER;
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
                robot.robot.pos += nudge / nudgers as f32;
            }
        }
    }

    fn resolve_border_collisions(&mut self) {
        for robot in &mut self.agents {
            if robot.robot.pos.x - self.robot_radius < -self.world.width() / 2.0 {
                robot.robot.pos.x = -self.world.width() / 2.0 + self.robot_radius;
            }
            if robot.robot.pos.x + self.robot_radius > self.world.width() / 2.0 {
                robot.robot.pos.x = self.world.width() / 2.0 - self.robot_radius;
            }
            if robot.robot.pos.y - self.robot_radius < -self.world.height() / 2.0 {
                robot.robot.pos.y = -self.world.height() / 2.0 + self.robot_radius;
            }
            if robot.robot.pos.y + self.robot_radius > self.world.height() / 2.0 {
                robot.robot.pos.y = self.world.height() / 2.0 - self.robot_radius;
            }
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt();

        // Call the behavior function for each robot
        for agent in &mut self.agents {
            agent.control = (self.behavior)(&mut agent.robot);
            agent.robot.vel = agent.control.speed;
            agent.robot.avel = agent.control.steer;
        }

        // Update the position of each robot
        for agent in self.agents.iter_mut() {
            let robot = &mut agent.robot;
            let vel = Vec2::angled(robot.angle) * robot.vel;

            robot.pos += vel * dt;
            robot.angle += robot.avel * dt;
        }

        // Handle messages
        let messages: Vec<_> = self
            .agents
            .iter_mut()
            .flat_map(|agent| agent.robot.outgoing_msg.drain(..))
            .collect();
        for message in &messages {
            println!("[{}] {}", message.from.as_u32(), message.kind);
        }
        for robot in &mut self.agents {
            robot.robot.incomming_msg = messages
                .iter()
                .filter(|m| m.from != robot.robot.id)
                .cloned()
                .collect();
        }

        self.resolve_robot_collisions();
        self.resolve_border_collisions();
        self.resolve_world_collisions();

        // Update robot lidar data
        let mut lidar_data = Vec::with_capacity(self.agents.len());
        for agent in &self.agents {
            let robot = &agent.robot;
            let points = (0..LIDAR_RAYS)
                .map(|n| {
                    let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                    let (distance, _) = self.cast_ray(robot.pos, robot.angle + angle, LIDAR_RANGE);
                    robcore::LidarPoint { angle, distance }
                })
                .collect();
            lidar_data.push(robcore::LidarData(points));
        }

        // Update robot camera
        let mut cam_data = Vec::with_capacity(self.agents.len());

        const ANGLE_STEP: f32 = CAMERA_FOV / (CAMERA_RAYS - 1) as f32;

        for agent in &self.agents {
            let robot = &agent.robot;
            let points = (0..CAMERA_RAYS)
                .filter_map(|n| {
                    let angle = n as f32 * ANGLE_STEP - CAMERA_FOV / 2.0;
                    let (distance, cell) =
                        self.cast_ray(robot.pos, robot.angle + angle, CAMERA_RANGE);
                    match cell {
                        Some(Cell::SearchItem) => {
                            let propability = (CAMERA_RANGE - distance) / CAMERA_RANGE;
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
                cam_data.push(robcore::CamData(sparse_points));
            }
        }

        // Actually update the robots
        let data = lidar_data.into_iter().zip(cam_data);
        for (agent, (lidar, cam)) in self.agents.iter_mut().zip(data) {
            let robot = &mut agent.robot;
            robot.lidar = lidar;
            robot.cam = cam;
        }
    }
}
