use eframe::egui::{Pos2, Rgba, Vec2};
use robcore;

use crate::grid::{Cell, Grid};

const LIDAR_RANGE: f32 = 5.0;

pub struct World {
    grid: Grid,
    width: f32,
    height: f32,
}

impl World {
    const CELLS_PR_METER: f32 = 10.0;
    const RAY_CAST_STEP: f32 = 0.1;

    pub fn new(width: f32, height: f32) -> Self {
        let grid_width = (width * Self::CELLS_PR_METER).ceil() as usize;
        let grid_height = (height * Self::CELLS_PR_METER).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
        }
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

    pub fn get_cell(&self, pos: Pos2) -> Cell {
        let x = (pos.x * Self::CELLS_PR_METER).floor();
        let y = (pos.y * Self::CELLS_PR_METER).floor();
        let (min, max) = self.bounds();
        if pos.x < min.x || pos.x > max.x || pos.y < min.y || pos.y > max.y {
            return Cell::OutOfBounds;
        }
        let x = x as usize;
        let y = y as usize;
        self.grid.get(x, y)
    }

    pub fn width(&self) -> f32 {
        self.width
    }

    pub fn height(&self) -> f32 {
        self.height
    }
}

pub struct Robot {
    pub pos: Pos2,
    pub vel: f32,
    pub angle: f32,
    pub avel: f32,
    pub color: Rgba,
    pub lidar: robcore::LidarData,
}

impl Robot {
    pub fn new_at(x: f32, y: f32) -> Self {
        Self {
            pos: Pos2 { x, y },
            vel: 0.0,
            angle: 0.0,
            avel: 0.0,
            color: Rgba::WHITE,
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

pub struct Simulator {
    pub robots: Vec<Robot>,
    pub robot_size: f32,
    pub world: World,
    behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
}

impl Simulator {
    pub fn new(world: World, behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a))) -> Self {
        Self {
            robots: vec![],
            robot_size: 0.3,
            behavior,
            world,
        }
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
            if distance > self.robot_size {
                for robot in &self.robots {
                    if (robot.pos - pos).length() < self.robot_size / 2.0 {
                        return distance;
                    }
                }
            }

            pos += step;
            distance += step_size;
        }

        distance
    }

    pub fn step(&mut self, dt: f32) {
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

        // Update robot lidar data
        let mut lidar_data = Vec::with_capacity(self.robots.len());
        for robot in &self.robots {
            let points = (0..360)
                .step_by(10)
                .map(|angle| {
                    let angle = angle as f32 * std::f32::consts::PI / 180.0;
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
