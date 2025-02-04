#![allow(dead_code)] // TODO: remove

pub struct Message;
pub struct CamData;
pub struct LidarData;

pub struct Control {
    speed: f32,
    steer: f32,
}

#[derive(Clone)]
pub struct Pos {
    pub x: f64,
    pub y: f64,
}
impl Copy for Pos {}

pub trait Robot {
    /// Get the position of the robot
    fn get_pos(&self) -> Pos;

    /// Get the data from the camera. Angles and probability of objects.
    fn get_cam_data(&self) -> CamData;

    /// Get the data from the lidar. Distance to objects.
    fn get_lidar_data(&self) -> Vec<i32>;

    /// Send a message to the other robots.
    fn post(&self, msg: Message);

    /// Get the messages from the other robots since the last call.
    fn recv(&mut self) -> Vec<Message>;

    /// Set the control signal for the robot. "Move the robot like this".
    fn set_control(&self, control: Control);

}

pub mod behaviors {
    use super::*;

    pub fn circle(robot: &mut dyn Robot) {
        robot.set_control(Control {
            speed: 1.0,
            steer: 0.5,
        });
    }
}
