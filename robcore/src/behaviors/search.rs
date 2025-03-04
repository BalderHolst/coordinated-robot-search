//! This module contains robot `search` behavior.

use std::{f32::consts::PI, time::Instant};

use emath::Vec2;

use crate::LidarData;

use super::{shapes::Circle, utils::normalize_angle, Control, DebugType, Robot};

/// The range of the lidar sensor at which the robot moves away from an object
const LIDAR_OBSTACLE_RANGE: f32 = 1.0;

const GRADIENT_WEIGHT: f32 = 2.0;
const LIDAR_WEIGHT: f32 = 0.3;
const FORWARD_BIAS: f32 = 0.05;

const ANGLE_THRESHOLD: f32 = PI / 4.0;

/// Search for the object using a gradient on the heat map
pub fn search(robot: &mut Robot, time: Instant) -> Control {
    robot.update_search_grid(time);

    let forward_bias = Vec2::angled(robot.angle) * FORWARD_BIAS;
    robot.debug("Forward Bias", DebugType::Vector(forward_bias));

    let mut target = Vec2::ZERO;
    target += forward_bias;
    target += gradient(robot);
    target += lidar(robot);

    robot.debug("Target", DebugType::Vector(target));

    robot.clear_processed();

    control_towards(robot, target)
}

/// Calculate the gradient of the heat map around the robot
fn gradient(robot: &mut Robot) -> Vec2 {
    let mut gradient = Vec2::ZERO;
    {
        // Get the heat under the robot
        let mut robot_heat: f32 = 0.0;
        {
            let mut robot_points = vec![];
            for (pos, cell) in robot.search_grid.iter_circle(
                &(Circle {
                    center: robot.pos,
                    radius: robot.diameter * 2.0,
                }),
            ) {
                robot_heat += cell.unwrap_or(&0.0);
                robot_points.push((pos, cell));
            }
            robot_heat /= robot_points.len() as f32;
            robot.debug("Robot Heat", DebugType::Number(robot_heat));
        }

        let mut points = vec![];
        let mut total_weight: f32 = 0.0;
        let mut total_cells = 0;

        for (pos, cell) in robot.search_grid.iter_circle(
            &(Circle {
                center: robot.pos,
                radius: robot.lidar_range,
            }),
        ) {
            let angle = (pos - robot.pos).angle() - robot.angle;
            let dist = (pos - robot.pos).length();
            if dist > robot.lidar.interpolate(angle) {
                continue;
            }

            // Skip cells which are out of bounds
            let Some(cell) = cell else {
                continue;
            };

            // The relative position of the cell to the robot
            let vec = pos - robot.pos;

            // Weight is the difference between the cell and the robot heat
            let weight = cell - robot_heat;

            // Ignore cells that are colder than the robot as this ends up
            // repelling the robot leading to situations where the robot
            // gets stuck at the edge of the map
            if weight <= 0.0 {
                continue;
            }

            // We want the weight to be stronger the closer we are to the robot
            let nearness = 1.0 - vec.length() / robot.lidar_range;
            let weight = weight * nearness;

            points.push((pos, weight));

            gradient += vec.normalized() * weight;

            total_cells += 1;
            total_weight += weight;
        }

        if total_cells == 0 {
            total_cells = 1;
        }
        gradient /= total_cells as f32;

        gradient *= GRADIENT_WEIGHT;

        robot.debug("Gradient Values", DebugType::NumberPoints(points));
        robot.debug(
            "Average Weight",
            DebugType::Number(total_weight / total_cells as f32),
        );
        robot.debug("Gradient", DebugType::Vector(gradient));
        robot.debug("Gradient Length", DebugType::Number(gradient.length()));
    }

    gradient
}

/// Calculate the lidar contribution to the control
fn lidar(robot: &mut Robot) -> Vec2 {
    assert!(robot.lidar_range >= LIDAR_OBSTACLE_RANGE);
    let mut lidar_contribution = Vec2::ZERO;
    {
        let mut total_weight: f32 = 0.0;
        let LidarData(points) = robot.lidar.clone();
        for point in points {
            let distance = point.distance.clamp(0.0, LIDAR_OBSTACLE_RANGE);
            let weight = -(1.0 - distance / LIDAR_OBSTACLE_RANGE).powi(2);
            lidar_contribution += Vec2::angled(point.angle + robot.angle) * weight;
        }
        if total_weight == 0.0 {
            total_weight = 1.0;
        }
        lidar_contribution /= total_weight.abs();

        lidar_contribution *= LIDAR_WEIGHT;

        robot.debug("Lidar Contribution", DebugType::Vector(lidar_contribution));
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
