use std::f32::consts::PI;

use eframe::egui::{Pos2, Vec2};
use robcore::{self, grid::iter_circle, scaled_grid::ScaledGrid, CamPoint};

use crate::{
    cli::BehaviorFn,
    world::{Cell, World},
};

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

/// The factor of the world size to use as the search grid size
const SEARCH_GRID_FACTOR: f32 = 0.25;

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
        agent.robot.search_grid = ScaledGrid::new(
            self.world.width(),
            self.world.height(),
            self.world.scale() / SEARCH_GRID_FACTOR,
        );
        self.agents.push(agent);
    }

    fn cast_ray(
        &self,
        base_pos: Pos2,
        angle: f32,
        robot_radius: f32,
        max_range: f32,
    ) -> (f32, Option<Cell>) {
        let step_size = self.world.scale() * 0.5;
        let direction = Vec2::angled(angle);

        let mut distance = robot_radius;

        while distance < max_range {
            let pos = base_pos + direction * distance;

            let cell = self.world.get(pos);

            if !cell.is_empty() {
                return (distance, Some(cell));
            }

            // Check for collisions with other robots
            if distance > robot_radius {
                for robot in &self.agents {
                    let d = (robot.pos() - pos).length();
                    if d < robot_radius {
                        return (distance, None);
                    }
                }
            }

            distance += step_size;
        }

        (distance, None)
    }

    fn resolve_robot_collisions(&mut self) {
        for i in 0..self.agents.len() {
            for j in i + 1..self.agents.len() {
                let agent1 = &self.agents[i];
                let agent2 = &self.agents[j];
                let diameter = f32::max(agent1.robot.diameter, agent2.robot.diameter);
                if (agent1.pos() - agent2.pos()).length() < diameter {
                    let diff = agent1.pos() - agent2.pos();
                    let overlap = diameter - diff.length();
                    let dir = diff.normalized() * overlap / 2.0;
                    self.agents[i].robot.pos += dir;
                    self.agents[j].robot.pos -= dir;
                }
            }
        }
    }

    fn resolve_world_collisions(&mut self) {
        for agent in &mut self.agents {
            // Look in a circle around the robot
            let radius = agent.robot.diameter / 2.0 * 1.4 / self.world.scale();

            let center = self.world.world_to_grid(agent.pos());
            let mut nudge = Vec2::ZERO;
            let mut nudgers = 0;
            for (x, y) in iter_circle(center, radius) {
                let cell = self.world.grid().get(x, y);
                if !cell.is_empty() {
                    let cell_center = self.world.grid_to_world(Pos2 {
                        x: x as f32,
                        y: y as f32,
                    });
                    let diff = agent.pos() - cell_center;

                    let overlap =
                        agent.robot.diameter / 2.0 - diff.length() + 0.5 * self.world.scale();
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
                agent.robot.pos += nudge / nudgers as f32;
            }
        }
    }

    fn resolve_border_collisions(&mut self) {
        for agent in &mut self.agents {
            let pos = &mut agent.robot.pos;
            let radius = agent.robot.diameter / 2.0;
            if pos.x - radius < -self.world.width() / 2.0 {
                pos.x = -self.world.width() / 2.0 + radius;
            }
            if pos.x + radius > self.world.width() / 2.0 {
                pos.x = self.world.width() / 2.0 - radius;
            }
            if pos.y - radius < -self.world.height() / 2.0 {
                pos.y = -self.world.height() / 2.0 + radius;
            }
            if pos.y + radius > self.world.height() / 2.0 {
                pos.y = self.world.height() / 2.0 - radius;
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
                    let (distance, _) = self.cast_ray(
                        robot.pos,
                        robot.angle + angle,
                        robot.diameter / 2.0,
                        robot.lidar_range,
                    );
                    robcore::LidarPoint { angle, distance }
                })
                .collect();
            lidar_data.push(robcore::LidarData(points));
        }

        // Update robot camera
        let mut cam_data = Vec::with_capacity(self.agents.len());

        for agent in &self.agents {
            let robot = &agent.robot;
            let angle_step = robot.cam_fov / (CAMERA_RAYS - 1) as f32;
            let (_, max_camera_range) = robot.cam_range;
            let points = (0..CAMERA_RAYS)
                .filter_map(|n| {
                    let angle = n as f32 * angle_step - robot.cam_fov / 2.0;
                    let (distance, cell) = self.cast_ray(
                        robot.pos,
                        robot.angle + angle,
                        robot.diameter / 2.0,
                        max_camera_range,
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
