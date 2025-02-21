pub mod search;

use std::{f32::consts::PI, time::Instant};

use super::*;

pub use search::search;

pub fn circle(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 1.0,
        steer: 0.5,
    }
}

pub fn nothing(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 0.0,
        steer: 0.0,
    }
}

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

    let CamData(cam) = robot.cam.clone();
    for point in cam.iter() {
        robot.post(MessageKind::String(format!(
            "Found point of interest at angle {:.2} with propability {:.2}!",
            point.angle, point.propability
        )));
    }

    for msg in robot.recv().clone() {
        if matches!(msg.kind, MessageKind::Debug(_)) {
            continue;
        }
        robot.post(MessageKind::Debug(format!(
            "Received message from robot {}.",
            msg.from.as_u32(),
        )));
    }

    Control { speed, steer }
}
