use eframe::egui::{Pos2, Rgba, Vec2};

pub struct Robot {
    pos: Pos2,
    vel: Vec2,
    color: Rgba,
}



pub struct Simulator {
    robots: Vec<Robot>,
}

impl Simulator {

    pub fn new() -> Self {
        Self {
            robots: vec![],
        }
    }

    pub fn add_robot(&mut self, robot: Robot) {
        self.robots.push(robot);
    }

    pub fn step(&mut self, dt: f32) {
        for robot in self.robots.iter_mut() {
            robot.pos += robot.vel * dt;
        }
    }

}
