use eframe::egui::{Pos2, Rgba, Vec2};
use robcore;

pub struct Robot {
    pub pos: Pos2,
    pub vel: f32,
    pub angle: f32,
    pub avel: f32,
    pub color: Rgba,
}

impl robcore::Robot for Robot {
    fn get_pos(&self) -> robcore::Pos {
        robcore::Pos {
            x: self.pos.x,
            y: self.pos.y,
        }
    }

    fn get_cam_data(&self) -> robcore::CamData {
        todo!()
    }

    fn get_lidar_data(&self) -> Vec<i32> {
        todo!()
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
    behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
}

impl Simulator {

    pub fn new(behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a))) -> Self {
        Self {
            robots: vec![],
            behavior,
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



        }
    }

}
