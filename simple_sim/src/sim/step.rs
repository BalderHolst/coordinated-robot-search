use std::f32::consts::PI;

use botbrain::{
    behaviors::BehaviorFn,
    camera::{CamData, CamPoint},
    lidar::{LidarData, LidarPoint},
    params::{CAM_FOV, CAM_RANGE, DIAMETER, LIDAR_RANGE},
    Pos2, Robot, RobotPose, Vec2,
};
use eframe::emath::normalized_angle;

use crate::world::{Cell, World};

use super::{
    collisions, RobotState, StepArgs, CAMERA_RAYS, LIDAR_RAYS, SPEED_MULTIPLIER, STEER_MULTIPLIER,
};

pub fn step_agent(
    state: &mut RobotState,
    robot: &mut Box<dyn Robot>,
    args: &StepArgs,
    behavior_fn: BehaviorFn,
) {
    let StepArgs {
        agents,
        world,
        time,
        dt,
        msg_send_tx,
        pending_msgs,
    } = args;

    let dt = *dt;

    // Call the behavior function
    let (control, msgs) = behavior_fn(robot, *time);

    // Update the robot state
    {
        state.control = control;
        state.vel = state.control.speed * SPEED_MULTIPLIER;
        state.avel = state.control.steer * STEER_MULTIPLIER;

        // Update position of the robot
        let vel = Vec2::angled(state.pose.angle) * state.vel;
        state.pose.pos += vel * dt;
        state.pose.angle = normalized_angle(state.pose.angle + state.avel * dt);

        // Set the new state of the robot
        robot.input_pose(RobotPose {
            pos: state.pose.pos,
            angle: state.pose.angle,
        });

        let robot_soup = robot.get_debug_soup_mut();
        if robot_soup.is_active() {
            state.soup.activate();
        }

        // Swap soups
        std::mem::swap(robot_soup, &mut state.soup);
    }

    // Update postboxes
    {
        // Send messages
        for msg in msgs {
            let _ = msg_send_tx.send(msg).map_err(|e| {
                eprintln!("[ERROR] Error sending message: {:?}", e.to_string());
            });
        }

        // Receive messages
        robot.input_msgs(pending_msgs.clone());
    }

    // Update lidar data
    {
        let points = (0..LIDAR_RAYS)
            .map(|n| {
                let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                let (distance, _) = cast_ray(
                    world,
                    agents,
                    state.pose.pos,
                    state.pose.angle + angle,
                    DIAMETER / 2.0,
                    LIDAR_RANGE,
                    &[Cell::SearchItem],
                );
                LidarPoint { angle, distance }
            })
            .collect();

        robot.input_lidar(LidarData::new(points));
    }

    // Update robot camera
    {
        let angle_step = CAM_FOV / (CAMERA_RAYS - 1) as f32;
        let points = (0..CAMERA_RAYS)
            .filter_map(|n| {
                let angle = n as f32 * angle_step - CAM_FOV / 2.0;
                let (distance, cell) = cast_ray(
                    world,
                    agents,
                    state.pose.pos,
                    state.pose.angle + angle,
                    DIAMETER / 2.0,
                    CAM_RANGE,
                    &[],
                );
                match cell {
                    Some(Cell::SearchItem) => {
                        let probability = (CAM_RANGE - distance) / CAM_RANGE;
                        Some((n, CamPoint { angle, probability }))
                    }
                    _ => None,
                }
            })
            .collect::<Vec<_>>();

        // Consolidate points next to each other
        {
            let mut sparse_points = vec![];
            let mut adjacant = vec![];

            let consolidate_points = |adjacant: &mut Vec<_>, sparse_points: &mut Vec<_>| {
                if adjacant.is_empty() {
                    return;
                }

                let mut avg_point = adjacant.iter().fold(
                    CamPoint {
                        angle: 0.0,
                        probability: 0.0,
                    },
                    |a: CamPoint, (_, b): &(usize, CamPoint)| CamPoint {
                        angle: a.angle + b.angle,
                        probability: a.probability + b.probability,
                    },
                );
                avg_point.probability /= adjacant.len() as f32;
                avg_point.angle /= adjacant.len() as f32;
                sparse_points.push(avg_point);
                adjacant.clear();
            };

            for (n, point) in points {
                match adjacant.last() {
                    None => adjacant.push((n, point)),
                    Some((last_n, _)) if *last_n == n - 1 => adjacant.push((n, point)),
                    Some(_) => consolidate_points(&mut adjacant, &mut sparse_points),
                }
            }
            consolidate_points(&mut adjacant, &mut sparse_points);
            robot.input_cam(CamData::Points(sparse_points));
        }
    }

    // Resolve collisions
    collisions::resolve_robot_collisions(state, agents);
    collisions::resolve_border_collisions(state, world);
    collisions::resolve_world_collisions(state, world);
}

pub fn cast_ray(
    world: &World,
    agents: &[RobotState],
    base_pos: Pos2,
    angle: f32,
    robot_radius: f32,
    max_range: f32,
    ignore: &[Cell],
) -> (f32, Option<Cell>) {
    let step_size = world.scale() * 0.5;
    let direction = Vec2::angled(angle);

    let mut distance = robot_radius;

    while distance < max_range {
        let pos = base_pos + direction * distance;

        let cell = world.get(pos);

        // Check for collisions with the world
        if let Some(cell) = cell {
            if !cell.is_empty() && !ignore.contains(cell) {
                return (distance, Some(*cell));
            }
        } else {
            return (distance, Some(Cell::Wall));
        }

        // Check for collisions with other robots
        if distance > robot_radius {
            for robot_state in agents {
                let d = (robot_state.pose.pos - pos).length();
                if d < robot_radius {
                    return (distance, None);
                }
            }
        }
        distance += step_size;
    }

    if distance >= max_range {
        (max_range, None)
    } else {
        (distance - step_size / 2.0, None)
    }
}
