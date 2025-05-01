use emath::{Pos2, Vec2};

use crate::{
    behaviors::ScaledGrid,
    params::LIDAR_RANGE,
    shapes::{Circle, Line},
    LidarData, LidarPoint, Map, MapCell,
};

pub(super) const COSTMAP_GRID_SCALE: f32 = 0.2;
pub(super) const COSTMAP_DYNAMIC_OBSTACLE_WIDTH: f32 = 0.25;

pub(super) const COSTMAP_OBSTACLE: f32 = -3.0;
pub(super) const COSTMAP_DYNAMIC_OBSTACLE: f32 = -2.0;
pub(super) const COSTMAP_SEARCHED: f32 = -1.0;
pub(super) const COSTMAP_UNKNOWN: f32 = 1.0;

/// Checks if a pos is free
/// If width is 0.0 only the pos is checked
/// If width is > 0.0 the pos is checked and the surrounding cells are checked
pub fn validate_pos(pos: Pos2, clearance: f32, costmap_grid: &ScaledGrid<f32>) -> bool {
    match clearance {
        0.0 => costmap_grid
            .get(pos)
            .is_some_and(|&cell| cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE),
        _ => {
            let circle = Circle {
                center: pos,
                radius: clearance,
            };
            costmap_grid.iter_circle(&circle).all(|pos| {
                costmap_grid.get(pos).is_some_and(|&cell| {
                    cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE
                })
            })
        }
    }
}

/// Checks if all cells in the line is free
pub fn validate_line(line: Line, costmap_grid: &ScaledGrid<f32>) -> bool {
    costmap_grid.iter_line(&line).all(|pos| {
        costmap_grid
            .get(pos)
            .is_some_and(|&cell| cell != COSTMAP_OBSTACLE && cell != COSTMAP_DYNAMIC_OBSTACLE)
    })
}

/// Checks if all cells in the line with a width is free
pub fn validate_thick_line(line: Line, clearance: f32, costmap_grid: &ScaledGrid<f32>) -> bool {
    let dir = (line.end - line.start).normalized();
    let points = [
        -clearance,
        -clearance / 2.0,
        0.0,
        clearance / 2.0,
        clearance,
    ];
    points.iter().all(|&diff| {
        let start = line.start - dir.rot90() * diff;
        let end = line.end - dir.rot90() * diff;
        let line = Line { start, end };
        validate_line(line, costmap_grid)
    })
}

/// Constructs a costmap grid from a search grid, a obstacle map and a lidar
pub fn make_costmap_grid(
    robot_pos: Pos2,
    robot_angle: f32,
    map: &Map,
    search_grid: &ScaledGrid<f32>,
    lidar: &LidarData,
) -> ScaledGrid<f32> {
    let mut costmap = ScaledGrid::<f32>::new(map.width(), map.height(), COSTMAP_GRID_SCALE);

    // Update costmap from search grid
    costmap.iter_mut().for_each(|(x, y, costmap_cell)| {
        if let Some(&cell) = search_grid.get(Pos2 { x, y }) {
            *costmap_cell = if cell != 0.0 {
                COSTMAP_SEARCHED
            } else {
                COSTMAP_UNKNOWN
            };
        };
    });

    map.iter().for_each(|(x, y, &cell)| {
        // Negative if don't want to go there, positive if want to go there
        // Non-zero cells are explored therefore we won't go there
        if cell == MapCell::Obstacle {
            costmap.set(Pos2 { x, y }, COSTMAP_OBSTACLE);
        };
    });

    costmap.iter_mut().for_each(|(x, y, costmap_cell)| {
        if let Some(&cell) = map.get(Pos2 { x, y }) {
            if cell == MapCell::Obstacle {
                *costmap_cell = COSTMAP_OBSTACLE
            }
        }
    });

    // Using lidar to insert dynamic obstacles
    lidar
        .points()
        .filter(|lidar| (lidar.distance < LIDAR_RANGE))
        .map(|&LidarPoint { angle, distance }| {
            let point = robot_pos + Vec2::angled(angle + robot_angle) * distance;
            Circle {
                center: point,
                radius: COSTMAP_DYNAMIC_OBSTACLE_WIDTH / 2.0,
            }
        })
        .for_each(|circle| {
            costmap
                // Negative if don't want to go there, positive if want to go there
                .set_circle(circle.center, circle.radius, COSTMAP_DYNAMIC_OBSTACLE);
        });

    costmap
}
