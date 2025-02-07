pub use emath::{Pos2, Vec2};

fn normalize_angle(angle: f32) -> f32 {
    let mut angle = angle;
    while angle < -std::f32::consts::PI {
        angle += 2.0 * std::f32::consts::PI;
    }
    while angle > std::f32::consts::PI {
        angle -= 2.0 * std::f32::consts::PI;
    }
    angle
}

#[derive(Debug, Clone)]
pub struct Message;

#[derive(Debug, Clone)]
pub struct CamPoint {
    pub angle: f32,
    pub propability: f32,
}

#[derive(Debug, Clone)]
pub struct CamData(pub Vec<CamPoint>);

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
    fn get_cam_data(&self) -> &CamData;

    /// Get the data from the lidar. Distance to objects.
    fn get_lidar_data(&self) -> &LidarData;

    /// Send a message to the other robots.
    fn post(&self, msg: Message);

    /// Get the messages from the other robots since the last call.
    fn recv(&mut self) -> Vec<Message>;

    /// Set the control signal for the robot. "Move the robot like this".
    fn set_control(&mut self, control: Control);
}

pub mod behaviors {
    use std::f32::consts::PI;

    use super::*;

    pub fn circle(robot: &mut dyn Robot) {
        robot.set_control(Control {
            speed: 1.0,
            steer: 0.5,
        });
    }

    pub fn only_straight(robot: &mut dyn Robot) {
        robot.set_control(Control {
            speed: 1.0,
            steer: 0.0,
        });
    }

    pub fn nothing(robot: &mut dyn Robot) {
        robot.set_control(Control {
            speed: 0.0,
            steer: 0.0,
        });
    }

    pub fn move_to_center(robot: &mut dyn Robot) {
        let pos = robot.get_pos();
        let steer = f32::atan2(-pos.y, -pos.x);
        robot.set_control(Control { speed: 1.0, steer });
    }

    pub fn avoid_obstacles(robot: &mut dyn Robot) {
        const MIN_DISTANCE: f32 = 3.0;
        const FOV: f32 = PI / 1.8;

        let lidar = robot.get_lidar_data();

        let mut steer = 0.0;
        let mut speed = 1.0;

        // Find the closest point in front of the robot
        let mut min_point = LidarPoint {
            angle: 0.0,
            distance: f32::INFINITY,
        };
        for point in &lidar.0 {
            let angle = normalize_angle(point.angle);
            if angle.abs() < FOV || angle.abs() > 2.0 * PI - FOV {
                if point.distance < min_point.distance {
                    min_point = point.clone();
                }
            }
        }

        // If the closest point is too close, steer away from it
        if min_point.distance < MIN_DISTANCE {
            let how_close = (MIN_DISTANCE - min_point.distance) / MIN_DISTANCE;
            steer = how_close * (min_point.angle - PI).signum();
            speed *= 1.0 - how_close;
        }

        robot.set_control(Control { speed, steer });
    }
}
