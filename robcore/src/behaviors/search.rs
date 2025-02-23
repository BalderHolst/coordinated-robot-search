use std::{f32::consts::PI, time::Instant};

use emath::Vec2;

use crate::{normalize_angle, LidarData};

use super::{Control, DebugType, Robot};

const GRADIENT_RADIUS: f32 = 6.0;

const GRADIENT_WEIGHT: f32 = 1.0;

const FORWARD_BIAS: f32 = 0.2;

const ANGLE_THRESHOLD: f32 = PI / 4.0;

pub fn search(robot: &mut Robot, time: Instant) -> Control {
    robot.update_search_grid(time);

    let mut gradient = Vec2::ZERO;
    {
        // Get the heat under the robot
        let mut robot_heat: f32 = 0.0;
        let mut robot_points = vec![];
        for (pos, cell) in robot.search_grid.iter_circle(robot.pos, robot.diameter) {
            robot_heat += cell;
            robot_points.push((pos, cell));
        }
        robot_heat /= robot_points.len() as f32;

        robot.show("Robot Heat", DebugType::Number(robot_heat));

        let mut points = vec![];
        let mut total_weight: f32 = 0.0;
        let mut n = 0;
        for (pos, cell) in robot.search_grid.iter_circle(robot.pos, GRADIENT_RADIUS) {
            let v = pos - robot.pos;
            let angle = v.angle();
            let weight = cell - robot_heat;
            points.push((pos, weight));
            gradient += Vec2::angled(angle) * weight;
            n += 1;
            total_weight += weight;
        }
        gradient /= n as f32;

        gradient *= GRADIENT_WEIGHT;

        robot.show("Gradient Values", DebugType::NumberPoints(points));
        robot.show("Gradient Radius", DebugType::Radius(GRADIENT_RADIUS));
        robot.show("Average Weight", DebugType::Number(total_weight / n as f32));
        robot.show("Gradient", DebugType::Vector(gradient));
        robot.show("Gradient Length", DebugType::Number(gradient.length()));
    }

    // Lidar contribution
    let mut lidar_contribution = Vec2::ZERO;
    {
        let mut total_weight: f32 = 0.0;
        let LidarData(points) = robot.lidar.clone();
        for point in points {
            let weight = point.distance / robot.lidar_range;
            lidar_contribution += Vec2::angled(point.angle + robot.angle) * weight;
        }
        if total_weight == 0.0 {
            total_weight = 1.0;
        }
        lidar_contribution /= total_weight.abs();

        robot.show("Lidar Contribution", DebugType::Vector(lidar_contribution));
    }

    let forward_bias = Vec2::angled(robot.angle) * FORWARD_BIAS;
    robot.show("Forward Bias", DebugType::Vector(forward_bias));

    let target = gradient + lidar_contribution + forward_bias;

    robot.show("Target", DebugType::Vector(target));

    robot.clear_processed();

    let angle_error = normalize_angle(robot.angle - target.angle());

    let mut t = (angle_error * angle_error / ANGLE_THRESHOLD).clamp(0.0, 1.0);

    if t < 0.25 {
        t = 0.0;
    }

    // println!("t: {t}");

    let speed = 1.0 - t;
    let steer = t * normalize_angle(target.angle() - robot.angle);

    Control { speed, steer }
}
