use eframe::egui::{Pos2, Rgba, Vec2};
use robcore;

use crate::grid::{Cell, Grid};

pub struct World {
    grid: Grid,
    width: f32,
    height: f32,
}

impl World {

    const CELLS_PR_METER: f32 = 10.0;

    pub fn new(width: f32, height: f32) -> Self {

        let grid_width = (width * Self::CELLS_PR_METER).ceil() as usize;
        let grid_height = (height * Self::CELLS_PR_METER).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
        }
    }

    pub fn get_cell(&self, pos: Pos2) -> Cell {
        let x = (pos.x * Self::CELLS_PR_METER).floor() as usize;
        let y = (pos.y * Self::CELLS_PR_METER).floor() as usize;
        self.grid.get(x, y)
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
    world: World,
    behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
}

impl Simulator {
    pub fn new(world: World, behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a))) -> Self {
        Self {
            robots: vec![],
            behavior,
            world,
        }
    }

    pub fn add_robot(&mut self, robot: Robot) {
        self.robots.push(robot);
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

            let points = (0..360)
                .step_by(10)
                .map(|angle| {
                    let angle = angle as f32 * std::f32::consts::PI / 180.0;
                    let distance = 1.0;
                    robcore::LidarPoint { angle, distance }
                })
                .collect();
            robot.lidar = robcore::LidarData(points)
        }
    }
}
