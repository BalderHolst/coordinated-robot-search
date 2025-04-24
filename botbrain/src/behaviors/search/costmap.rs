use std::f32::consts::PI;

use emath::{Pos2, Vec2};

use crate::{
    behaviors::ScaledGrid,
    params::LIDAR_RANGE,
    shapes::{Circle, Line},
    LidarData, LidarPoint,
};

pub(super) const COSTMAP_GRID_SCALE: f32 = 0.5;
pub(super) const COSTMAP_DYNAMIC_OBSTACLE_WIDTH: f32 = 1.0;

pub(super) const COSTMAP_OBSTACLE: f32 = -3.0;
pub(super) const COSTMAP_DYNAMIC_OBSTACLE: f32 = -2.0;
pub(super) const COSTMAP_SEARCHED: f32 = -1.0;
pub(super) const COSTMAP_UNKNOWN: f32 = 1.0;

/// Checks if all cells in the line is free
pub fn validate_line(line: Line, costmap_grid: &ScaledGrid<f32>) -> bool {
    costmap_grid.iter_line(&line).all(|pos| {
        if let Some(&cell) = costmap_grid.get(pos) {
            cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE
        } else {
            true
        }
    })
}
/// Checks if a pos is free
/// If width is 0.0 only the pos is checked
/// If width is > 0.0 the pos is checked and the surrounding cells are checked
pub fn validate_pos(pos: Pos2, width: f32, costmap_grid: &ScaledGrid<f32>) -> bool {
    if width == 0.0 {
        if let Some(&cell) = costmap_grid.get(pos) {
            cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE
        } else {
            true
        }
    } else {
        let circle = Circle {
            center: pos,
            radius: width,
        };
        costmap_grid.iter_circle(&circle).all(|pos| {
            if let Some(&cell) = costmap_grid.get(pos) {
                cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE
            } else {
                true
            }
        })
    }
}

/// Checks if all cells in the line with a width is free
pub fn validate_thick_line(line: Line, width: f32, costmap_grid: &ScaledGrid<f32>) -> bool {
    let dir = (line.end - line.start).normalized();
    let points = [-width / 2.0, 0.0, width / 2.0];
    for point in points {
        let line = Line {
            start: line.start + (dir - Vec2::angled(-PI / 2.0)) * point,
            end: line.end + (dir - Vec2::angled(-PI / 2.0)) * point,
        };
        let valid = costmap_grid.iter_line(&line).all(|pos| {
            if let Some(&cell) = costmap_grid.get(pos) {
                cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE
            } else {
                true
            }
        });
        if !valid {
            println!("Invalid line: {:?}", line);
            return false;
        }
    }
    true
}

/// Constructs a costmap grid from a search grid, a obstacle map and a lidar
pub fn make_costmap_grid(
    robot_pos: Pos2,
    robot_angle: f32,
    map: &ScaledGrid<f32>,
    search_grid: &ScaledGrid<f32>,
    lidar: &LidarData,
) -> ScaledGrid<f32> {
    let mut costmap_grid = ScaledGrid::<f32>::new(map.width(), map.height(), COSTMAP_GRID_SCALE);

    // Update costmap from search grid
    search_grid.iter().for_each(|(x, y, &cell)| {
        // Negative if don't want to go there, positive if want to go there
        // Non-zero cells are explored therefore we won't go there
        let cell = if cell != 0.0 {
            COSTMAP_SEARCHED
        } else {
            COSTMAP_UNKNOWN
        };
        costmap_grid.set(Pos2 { x, y }, cell);
    });

    // // Using lidar to insert dynamic obstacles
    // lidar
    //     .points()
    //     .filter_map(|&LidarPoint { angle, distance }| {
    //         // Make small circle if distance is under max distance
    //         if distance < LIDAR_RANGE {
    //             let point = robot_pos + Vec2::angled(angle + robot_angle) * distance;
    //             Some(Circle {
    //                 center: point,
    //                 radius: COSTMAP_DYNAMIC_OBSTACLE_WIDTH / 2.0,
    //             })
    //         } else {
    //             None
    //         }
    //     })
    //     .for_each(|circle| {
    //         costmap_grid
    //             // Negative if don't want to go there, positive if want to go there
    //             .set_circle(circle.center, circle.radius, COSTMAP_DYNAMIC_OBSTACLE);
    //     });

    map.iter().for_each(|(x, y, &cell)| {
        // Negative if don't want to go there, positive if want to go there
        // Non-zero cells are explored therefore we won't go there
        if cell == COSTMAP_OBSTACLE {
            costmap_grid.set(Pos2 { x, y }, COSTMAP_OBSTACLE);
        };
    });

    costmap_grid
}
