pub use emath::{Pos2, Vec2};

#[derive(Debug, Clone)]
pub struct Message;

#[derive(Debug, Clone)]
pub struct CamData;

#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub angle: f32,
    pub distance: f32,
}

#[derive(Debug, Clone)]
pub struct LidarData(pub Vec<LidarPoint>);

pub struct Control {
    pub speed: f32,
    pub steer: f32,
}

pub trait Robot {
    /// Get the position of the robot
    fn get_pos(&self) -> Pos2;

    /// Get the data from the camera. Angles and probability of objects.
    fn get_cam_data(&self) -> CamData;

    /// Get the data from the lidar. Distance to objects.
    fn get_lidar_data(&self) -> LidarData;

    /// Send a message to the other robots.
    fn post(&self, msg: Message);

    /// Get the messages from the other robots since the last call.
    fn recv(&mut self) -> Vec<Message>;

    /// Set the control signal for the robot. "Move the robot like this".
    fn set_control(&mut self, control: Control);

}

pub mod behaviors {
    use super::*;

    pub fn circle(robot: &mut dyn Robot) {
        robot.set_control(Control {
            speed: 1.0,
            steer: 0.5,
        });
    }

    pub fn move_to_center(robot: &mut dyn Robot) {
        let pos = robot.get_pos();
        let steer = f32::atan2(-pos.y, -pos.x);
        robot.set_control(Control {
            speed: 1.0,
            steer,
        });
    }

}
