use std::{f32::consts::PI, time::Instant};

use crate::LidarPoint;

use super::{Control, Robot};

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

    for point in robot.lidar.points() {
        let angle = point.angle;
        if (angle.abs() < FOV || angle.abs() > 2.0 * PI - FOV)
            && point.distance < min_point.distance
        {
            min_point = point.clone();
        }
    }

    // If the closest point is too close, steer away from it
    if min_point.distance < MIN_DISTANCE {
        let how_close = (MIN_DISTANCE - min_point.distance) / MIN_DISTANCE;
        let how_close = how_close.powi(2);
        steer = how_close * (-min_point.angle.signum());

        robot.post(crate::MessageKind::Debug(format!(
            "Close obstacle at {:.2}",
            min_point.angle
        )));

        speed *= 1.0 - how_close;
    }

    robot.clear_processed();
    Control { speed, steer }
}
