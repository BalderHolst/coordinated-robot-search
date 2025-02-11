#![allow(dead_code)]

pub use emath::{Pos2, Vec2};
pub use grid::Grid;

pub mod grid;

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

#[derive(Debug, Clone, Default)]
pub struct CamData(pub Vec<CamPoint>);

#[derive(Debug, Clone, Default)]
pub struct LidarPoint {
    pub angle: f32,
    pub distance: f32,
}

#[derive(Debug, Clone, Default)]
pub struct LidarData(pub Vec<LidarPoint>);

#[derive(Debug, Clone, Default)]
pub struct Control {
    pub speed: f32,
    pub steer: f32,
}

#[derive(Debug, Clone, Default)]
pub struct Robot {
    /// The position of the robot
    pub pos: Pos2,

    /// The velocity of the robot
    pub vel: f32,

    /// The angle of the robot
    pub angle: f32,

    /// The angular velocity of the robot
    pub avel: f32,

    /// The data from the camera. Angles and probability of objects.
    pub cam: CamData,
    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,
    /// The messages from the other robots since the last call.
    pub incomming_msg: Vec<Message>,
    /// The messages to be sent to the other robots.
    pub outgoing_msg: Vec<Message>,
}

impl Robot {
    /// Get the messages from the other robots since the last call.
    pub(crate) fn recv(&self) -> &Vec<Message> {
        &self.incomming_msg
    }

    /// Send a message to the other robots.
    pub(crate) fn post(&mut self, msg: Message) {
        self.outgoing_msg.push(msg);
    }
}

pub mod behaviors {
    use std::f32::consts::PI;

    use super::*;

    pub fn circle(_robot: &mut Robot) -> Control {
        Control {
            speed: 1.0,
            steer: 0.5,
        }
    }

    pub fn only_straight(_robot: &mut Robot) -> Control {
        Control {
            speed: 1.0,
            steer: 0.0,
        }
    }

    pub fn nothing(_robot: &mut Robot) -> Control {
        Control {
            speed: 0.0,
            steer: 0.0,
        }
    }

    pub fn move_to_center(robot: &mut Robot) -> Control {
        let pos = robot.pos;
        let steer = f32::atan2(-pos.y, -pos.x);
        Control { speed: 1.0, steer }
    }

    pub fn avoid_obstacles(robot: &mut Robot) -> Control {
        const MIN_DISTANCE: f32 = 3.0;
        const FOV: f32 = PI / 1.8;

        let mut steer = 0.0;
        let mut speed = 1.0;

        // Find the closest point in front of the robot
        let mut min_point = LidarPoint {
            angle: 0.0,
            distance: f32::INFINITY,
        };

        let LidarData(lidar) = &robot.lidar;
        for point in lidar {
            let angle = normalize_angle(point.angle);
            if (angle.abs() < FOV || angle.abs() > 2.0 * PI - FOV)
                && point.distance < min_point.distance
            {
                min_point = point.clone();
            }
        }

        // If the closest point is too close, steer away from it
        if min_point.distance < MIN_DISTANCE {
            let how_close = (MIN_DISTANCE - min_point.distance) / MIN_DISTANCE;
            steer = how_close * (min_point.angle - PI).signum();
            speed *= 1.0 - how_close;
        }

        Control { speed, steer }
    }

    /// Steers towards the furthest point in the lidar data that is closest to the heading of the robot.
    pub fn toward_space(robot: &mut Robot) -> Control {
        const MAX_SPEED: f32 = 1.0;
        const MAX_STEER: f32 = 10.0;

        let LidarData(mut points) = robot.lidar.clone();

        points.iter_mut().for_each(|point| {
            point.angle = normalize_angle(point.angle);
            if point.angle > PI {
                point.angle -= 2.0 * PI;
            }
        });

        points.sort_by(|a, b| a.angle.abs().partial_cmp(&b.angle.abs()).unwrap());

        let mut furthest_point = LidarPoint::default();
        for point in points {
            if point.distance > furthest_point.distance {
                furthest_point = point;
            }
        }

        let steer = furthest_point.angle / PI;
        let speed = 1.0 - steer;

        let speed = speed * MAX_SPEED;
        let steer = steer * MAX_STEER;

        Control { speed, steer }
    }
}
