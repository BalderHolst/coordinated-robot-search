use std::f32::consts::PI;

use eframe::egui::{Pos2, Vec2};
use robcore;

use crate::grid::{Cell, Grid};

const LIDAR_RANGE: f32 = 5.0;
const LIDAR_RAYS: usize = 40;

#[derive(Clone)]
pub struct World {
    grid: Grid,
    width: f32,
    height: f32,
}

impl World {
    const CELLS_PR_METER: f32 = 100.0;
    const RAY_CAST_STEP: f32 = 0.5;

    pub fn new(width: f32, height: f32) -> Self {
        let grid_width = (width * Self::CELLS_PR_METER).ceil() as usize;
        let grid_height = (height * Self::CELLS_PR_METER).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
        }
    }

    pub fn size(&self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    pub fn bounds(&self) -> (Pos2, Pos2) {
        let min = Pos2 {
            x: -self.width / 2.0,
            y: -self.height / 2.0,
        };
        let max = Pos2 {
            x: self.width / 2.0,
            y: self.height / 2.0,
        };
        (min, max)
    }

    fn world_to_grid(&self, pos: Pos2) -> Pos2 {
        ((pos + self.size() / 2.0) * Self::CELLS_PR_METER).floor()
    }

    fn grid_to_world(&self, pos: Pos2) -> Pos2 {
        (pos + Vec2::splat(0.5)) / Self::CELLS_PR_METER - self.size() / 2.0
    }

    pub fn get_cell(&self, pos: Pos2) -> Cell {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return Cell::OutOfBounds;
        }
        let pos = self.world_to_grid(pos);
        debug_assert!(pos.x >= 0.0, "x: {}", pos.x);
        debug_assert!(pos.y >= 0.0, "y: {}", pos.y);
        let x = pos.x as usize;
        let y = pos.y as usize;
        debug_assert!(
            x <= self.grid.width(),
            "x: {}, width: {}",
            x,
            self.grid.width()
        );
        debug_assert!(
            y <= self.grid.height(),
            "y: {}, height: {}",
            y,
            self.grid.height()
        );
        self.grid.get(x, y)
    }

    pub fn width(&self) -> f32 {
        self.width
    }

    pub fn height(&self) -> f32 {
        self.height
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: Cell) {
        let width = width * Self::CELLS_PR_METER;
        let start = self.world_to_grid(start);
        let end = self.world_to_grid(end);
        self.grid.line(start, end, width, cell);
    }

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: Cell) {
        let radius = radius * Self::CELLS_PR_METER;
        let center = self.world_to_grid(center);
        self.grid.circle(center, radius, cell);
    }
}

#[derive(Clone)]
pub struct Robot {
    pub pos: Pos2,
    pub vel: f32,
    pub angle: f32,
    pub avel: f32,
    pub lidar: robcore::LidarData,
}

impl Robot {
    pub fn new_at(x: f32, y: f32, angle: f32) -> Self {
        Self {
            pos: Pos2 { x, y },
            angle,
            vel: 0.0,
            avel: 0.0,
            lidar: robcore::LidarData(vec![]),
        }
    }
}

impl robcore::Robot for Robot {
    fn get_pos(&self) -> robcore::Pos2 {
        robcore::Pos2 {
            x: self.pos.x,
            y: self.pos.y,
        }
    }

    fn get_cam_data(&self) -> robcore::CamData {
        todo!()
    }

    fn get_lidar_data(&self) -> robcore::LidarData {
        self.lidar.clone()
    }

    fn post(&self, msg: robcore::Message) {
        todo!()
    }

    fn recv(&mut self) -> Vec<robcore::Message> {
        todo!()
    }

    fn set_control(&mut self, control: robcore::Control) {
        self.vel = control.speed;
        self.avel = control.steer;
    }
}

#[derive(Clone)]
pub struct Simulator {
    pub robots: Vec<Robot>,
    pub robot_radius: f32,
    pub world: World,
    pub sps: f32,
    behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
}

impl Simulator {
    pub fn new(world: World, sps: f32, behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a))) -> Self {
        Self {
            robots: vec![],
            robot_radius: 0.3,
            behavior,
            world,
            sps,
        }
    }

    fn dt(&self) -> f32 {
        1.0 / self.sps
    }

    pub fn add_robot(&mut self, robot: Robot) {
        self.robots.push(robot);
    }

    fn cast_ray(&self, pos: Pos2, angle: f32) -> f32 {
        let mut distance = 0.0;
        let mut pos = pos;
        let step_size = World::RAY_CAST_STEP / World::CELLS_PR_METER;
        let step = step_size * Vec2::angled(angle);

        while distance < LIDAR_RANGE {
            let cell = self.world.get_cell(pos);
            if !cell.is_empty() {
                break;
            }

            // Check for collisions with other robots
            if distance > self.robot_radius {
                for robot in &self.robots {
                    if (robot.pos - pos).length() < self.robot_radius {
                        return distance;
                    }
                }
            }

            pos += step;
            distance += step_size;
        }

        distance
    }

    fn resolve_robot_collisions(&mut self) {
        for i in 0..self.robots.len() {
            for j in i + 1..self.robots.len() {
                let robot1 = &self.robots[i];
                let robot2 = &self.robots[j];
                if (robot1.pos - robot2.pos).length() < self.robot_radius * 2.0 {
                    let diff = robot1.pos - robot2.pos;
                    let overlap = self.robot_radius * 2.0 - diff.length();
                    let dir = diff.normalized() * overlap / 2.0;
                    self.robots[i].pos += dir;
                    self.robots[j].pos -= dir;
                }
            }
        }
    }

    fn resolve_world_collisions(&mut self) {
        for robot in &mut self.robots {

            // Look in a circle around the robot
            let radius = self.robot_radius * World::CELLS_PR_METER * 1.4;

            let center = self.world.world_to_grid(robot.pos);
            let mut nudge = Vec2::ZERO;
            let mut nudgers = 0;
            for (x, y) in self.world.grid.circle_iter(center, radius) {
                let cell = self.world.grid.get(x, y);
                if !cell.is_empty() {
                    let cell_center = self.world.grid_to_world(Pos2 { x: x as f32, y: y as f32 });
                    let diff = robot.pos - cell_center;

                    let overlap = self.robot_radius - diff.length() + 0.5 / World::CELLS_PR_METER;
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

            if nudgers > 0{
                // Only nudge in the direction with most difference
                robot.pos += nudge / nudgers as f32;
            }
        }
    }

    fn resolve_border_collisions(&mut self) {
        for robot in &mut self.robots {
            if robot.pos.x - self.robot_radius < -self.world.width() / 2.0 {
                robot.pos.x = -self.world.width() / 2.0 + self.robot_radius;
            }
            if robot.pos.x + self.robot_radius > self.world.width() / 2.0 {
                robot.pos.x = self.world.width() / 2.0 - self.robot_radius;
            }
            if robot.pos.y - self.robot_radius < -self.world.height() / 2.0 {
                robot.pos.y = -self.world.height() / 2.0 + self.robot_radius;
            }
            if robot.pos.y + self.robot_radius > self.world.height() / 2.0 {
                robot.pos.y = self.world.height() / 2.0 - self.robot_radius;
            }
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt();

        // Call the behavior function for each robot
        for robot in &mut self.robots {
            (self.behavior)(robot);
        }

        // Update the position of each robot
        for robot in self.robots.iter_mut() {
            let vel = Vec2::angled(robot.angle) * robot.vel;

            robot.pos += vel * dt;
            robot.angle += robot.avel * dt;
        }

        self.resolve_robot_collisions();
        self.resolve_border_collisions();
        self.resolve_world_collisions();

        // Update robot lidar data
        let mut lidar_data = Vec::with_capacity(self.robots.len());
        for robot in &self.robots {
            let points = (0..LIDAR_RAYS)
                .map(|n| {
                    let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                    let distance = self.cast_ray(robot.pos, robot.angle + angle);
                    robcore::LidarPoint { angle, distance }
                })
                .collect();
            lidar_data.push(robcore::LidarData(points));
        }
        for (robot, data) in self.robots.iter_mut().zip(lidar_data) {
            robot.lidar = data;
        }
    }
}
