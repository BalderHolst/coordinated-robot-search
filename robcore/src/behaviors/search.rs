use std::{f32::consts::PI, time::Instant};

use emath::Vec2;

use crate::{normalize_angle, LidarData};

use super::{Control, DebugType, Robot};

const GRADIENT_RADIUS: f32 = 5.0;

const GRADIENT_WEIGHT: f32 = 1.0;
const LIDAR_WEIGHT: f32 = 0.3;
const FORWARD_BIAS: f32 = 0.1;

const ANGLE_THRESHOLD: f32 = PI / 4.0;

fn gradient(robot: &mut Robot) -> Vec2 {
    let mut gradient = Vec2::ZERO;
    {
        // Get the heat under the robot
        let mut robot_heat: f32 = 0.0;
        let mut robot_points = vec![];
        for (pos, cell) in robot.search_grid.iter_circle(robot.pos, robot.diameter) {
            robot_heat += cell.unwrap_or(0.0);
            robot_points.push((pos, cell));
        }
        robot_heat /= robot_points.len() as f32;

        robot.show("Robot Heat", DebugType::Number(robot_heat));

        let mut points = vec![];
        let mut total_weight: f32 = 0.0;
        let mut n = 0;
        for (pos, cell) in robot.search_grid.iter_circle(robot.pos, GRADIENT_RADIUS) {
            let Some(cell) = cell else {
                continue;
            };
            let angle = (pos - robot.pos).angle();
            let weight = cell - robot_heat;
            if weight <= 0.0 {
                continue;
            }
            points.push((pos, weight));
            gradient += Vec2::angled(angle) * weight;
            n += 1;
            total_weight += weight;
        }

        if n == 0 {
            n = 1;
        }
        gradient /= n as f32;

        gradient *= GRADIENT_WEIGHT;

        robot.show("Gradient Values", DebugType::NumberPoints(points));
        robot.show("Gradient Radius", DebugType::Radius(GRADIENT_RADIUS));
        robot.show("Average Weight", DebugType::Number(total_weight / n as f32));
        robot.show("Gradient", DebugType::Vector(gradient));
        robot.show("Gradient Length", DebugType::Number(gradient.length()));
    }

    gradient
}

fn lidar(robot: &mut Robot) -> Vec2 {
    let mut lidar_contribution = Vec2::ZERO;
    {
        let mut total_weight: f32 = 0.0;
        let LidarData(points) = robot.lidar.clone();
        for point in points {
            let weight = -(1.0 - point.distance / robot.lidar_range).powi(2);
            lidar_contribution += Vec2::angled(point.angle + robot.angle) * weight;
        }
        if total_weight == 0.0 {
            total_weight = 1.0;
        }
        lidar_contribution /= total_weight.abs();

        lidar_contribution *= LIDAR_WEIGHT;

        robot.show("Lidar Contribution", DebugType::Vector(lidar_contribution));
    }
    lidar_contribution
}

pub fn control_towards(robot: &mut Robot, target: Vec2) -> Control {
    let angle_error = normalize_angle(robot.angle - target.angle());
    let t = (angle_error * angle_error / ANGLE_THRESHOLD).clamp(0.0, 1.0);
    let speed = 1.0 - t;
    let steer = t * normalize_angle(target.angle() - robot.angle);

    Control { speed, steer }
}

pub fn search(robot: &mut Robot, time: Instant) -> Control {
    robot.update_search_grid(time);

    let forward_bias = Vec2::angled(robot.angle) * FORWARD_BIAS;
    robot.show("Forward Bias", DebugType::Vector(forward_bias));

    let mut target = Vec2::ZERO;
    target += forward_bias;
    target += gradient(robot);
    target += lidar(robot);

    robot.show("Target", DebugType::Vector(target));

    robot.clear_processed();

    control_towards(robot, target)
}
