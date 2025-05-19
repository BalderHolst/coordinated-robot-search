use std::collections::HashMap;

use emath::{Pos2, Vec2};

use crate::{
    camera::CamData,
    lidar::{LidarData, LidarPoint},
    messaging::{Message, MessageKind, Postbox},
    params,
    scaled_grid::ScaledGrid,
    shapes::{Circle, Cone, Line, Shape},
    Map, MapCell, RobotId,
};

type SearchGrid = ScaledGrid<f32>;

pub(crate) fn update_search_cone(
    search_grid: &mut SearchGrid,
    cone: &Cone,
    lidar: &LidarData,
    diff: f32,
) {
    for (point, mut cell) in search_grid
        .iter_cone(cone)
        .filter_map(|p| {
            let angle = (p - cone.center).angle();
            let distance = (p - cone.center).length();
            let cell = search_grid.get(p)?;
            match distance <= lidar.interpolate(angle - cone.angle) {
                true => Some((p, *cell)),
                false => None,
            }
        })
        .collect::<Vec<_>>()
    {
        cell += diff;
        search_grid.set(point, cell);
    }
}

pub(crate) fn update_search_line(search_grid: &mut SearchGrid, line: &Line, diff: f32) {
    let dir = (line.end - line.start).normalized();
    let step_size = search_grid.scale();
    let radius = step_size * 2.0;
    let mut distance = 0.0;
    while distance < params::CAM_RANGE - radius / 2.0 {
        let pos = line.start + dir * distance;

        // Nearness is in range [0, 1]
        let nearness = distance / params::CAM_RANGE;

        for (point, mut cell) in search_grid
            .iter_circle(&Circle {
                center: pos,
                radius,
            })
            .filter_map(|p| search_grid.get(p).map(|c| (p, *c)))
            .collect::<Vec<_>>()
        {
            // We weight points closer to the robot more
            cell += 2.0 * (diff * nearness);

            search_grid.set(point, cell);
        }

        distance += step_size;
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn update_search_grid(
    map: &Map,
    search_grid: &mut SearchGrid,
    id: RobotId,
    pos: Pos2,
    angle: f32,
    postbox: &mut Postbox,
    lidar: &LidarData,
    cam: &CamData,
    others: &mut HashMap<RobotId, (Pos2, f32)>,
    multiplier: f32,
) {
    const CAM_MULTPLIER: f32 = 20.0;

    process_search_messages(map, search_grid, postbox, others);

    // Cool down the search grid within the view of the camera
    {
        let cone = Cone {
            center: pos,
            radius: params::CAM_RANGE,
            angle,
            fov: params::CAM_FOV,
        };
        let lidar = lidar.within_fov(params::CAM_FOV);
        let diff = -multiplier;
        update_search_cone(search_grid, &cone, &lidar, diff);
        postbox.post(Message {
            sender_id: id,
            kind: MessageKind::CamDiff { cone, diff },
        });
    }

    // Heat up the search grid in the direction of the search items
    // detected by the camera
    for point in cam.points() {
        let diff = CAM_MULTPLIER * point.probability * multiplier;
        let lidar = lidar.within_fov(params::CAM_FOV);
        update_search_cone(search_grid, &point.cone, &lidar, diff);
        postbox.post(Message {
            sender_id: id,
            kind: MessageKind::CamDiff {
                cone: point.cone,
                diff,
            },
        });
    }
}

const INFER_RAYS: usize = 7;
fn infer_lidar(map: &Map, robot_pos: Pos2, robot_angle: f32) -> LidarData {
    let step = params::CAM_FOV / INFER_RAYS as f32;

    let angles = (0..INFER_RAYS).map(|i| -params::CAM_FOV / 2.0 + step * i as f32 + step / 2.0);

    let points: Vec<LidarPoint> = angles
        .map(|angle| {
            let res = map.cast_ray(
                robot_pos,
                angle + robot_angle,
                params::LIDAR_RANGE,
                |cell| *cell != MapCell::Free,
            );
            let distance = res.distance();

            LidarPoint { angle, distance }
        })
        .collect();

    LidarData::new(points)
}

fn process_search_messages(
    map: &Map,
    search_grid: &mut SearchGrid,
    postbox: &mut Postbox,
    others: &mut HashMap<RobotId, (Pos2, f32)>,
) {
    for (msg_id, msg) in postbox
        .recv()
        .map(|(id, msg)| (id, msg.clone()))
        .collect::<Vec<_>>()
    {
        match &msg.kind {
            MessageKind::ShapeDiff { shape, diff } => {
                match shape {
                    Shape::Cone(_cone) => todo!(),
                    Shape::Line(line) => update_search_line(search_grid, line, *diff),
                    _ => todo!(),
                };
                postbox.set_processed(msg_id);
            }
            MessageKind::CamDiff { cone, diff } => {
                // Update the position of the other robot
                let pos = cone.center;
                let angle = cone.angle;

                others.insert(msg.sender_id, (pos, angle));

                let lidar = infer_lidar(map, pos, angle);

                // Update the search grid based on the camera data
                update_search_cone(search_grid, cone, &lidar, *diff);
                postbox.set_processed(msg_id);
            }
            MessageKind::Debug(_) => {}
        }
    }
}

/// Calculate the gradient of the heat map around the robot
pub fn gradient(
    pos: Pos2,
    angle: f32,
    range: f32,
    center_range: f32,
    lidar: LidarData,
    grid: &ScaledGrid<f32>,
) -> (Vec2, Vec<(Pos2, f32)>) {
    let mut total_heat = 0.0;
    let mut robot_points: usize = 0;
    for pos in grid.iter_circle(
        &(Circle {
            center: pos,
            radius: center_range,
        }),
    ) {
        let cell = grid.get(pos);
        total_heat += cell.unwrap_or(&0.0);
        robot_points += 1;
    }

    let robot_heat = total_heat / robot_points as f32;

    let mut cells = vec![];
    let mut gradient = Vec2::ZERO;
    {
        for cell_pos in grid.iter_circle(
            &(Circle {
                center: pos,
                radius: range,
            }),
        ) {
            let angle = (cell_pos - pos).angle() - angle;
            let dist = (cell_pos - pos).length();
            if dist > lidar.interpolate(angle) {
                continue;
            }

            // Skip cells which are out of bounds
            let Some(cell) = grid.get(cell_pos) else {
                continue;
            };

            // The relative position of the cell to the robot
            let vec = cell_pos - pos;

            // Weight is the difference between the cell and the robot heat
            let weight = cell - robot_heat;

            // Ignore cells that are colder than the robot as this ends up
            // repelling the robot leading to situations where the robot
            // gets stuck at the edge of the map
            if weight <= 0.0 {
                continue;
            }

            // We want the weight to be stronger the closer we are to the robot
            let nearness = 1.0 - vec.length() / range;
            let weight = weight * nearness;

            gradient += vec.normalized() * weight;

            cells.push((cell_pos, weight));
        }

        if !cells.is_empty() {
            gradient /= cells.len() as f32;
        }
    }

    (gradient, cells)
}
