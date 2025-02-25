//! This module contains the robot behaviors.

pub mod search;

use std::{f32::consts::PI, time::Instant};

use super::*;

pub use search::search;
use utils::normalize_angle;

/// Move in a circle.
pub fn circle(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 1.0,
        steer: 0.5,
    }
}

/// Do nothing.
pub fn nothing(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 0.0,
        steer: 0.0,
    }
}

/// Avoid obstacles by steering away from the closest point in front of the robot.
pub fn avoid_obstacles(robot: &mut Robot, time: Instant) -> Control {
    const MIN_DISTANCE: f32 = 3.0;
    const FOV: f32 = PI / 1.8;

    robot.update_search_grid(time);

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

    robot.clear_processed();
    Control { speed, steer }
}
