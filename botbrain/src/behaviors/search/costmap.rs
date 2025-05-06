use emath::{Pos2, Vec2};

use crate::{
    behaviors::ScaledGrid,
    lidar::{LidarData, LidarPoint},
    params::LIDAR_RANGE,
    shapes::{Circle, Line},
    Map, MapCell,
};

use super::{Costmap, CostmapCell};

pub(super) const COSTMAP_GRID_SCALE: f32 = 0.2;
pub(super) const COSTMAP_DYNAMIC_OBSTACLE_WIDTH: f32 = 0.25;

/// Checks if a pos is free
/// If width is 0.0 only the pos is checked
/// If width is > 0.0 the pos is checked and the surrounding cells are checked
pub fn validate_pos(pos: Pos2, clearance: f32, costmap_grid: &Costmap) -> bool {
    match clearance {
        0.0 => costmap_grid.get(pos).is_some_and(CostmapCell::maybe_free),
        _ => {
            let circle = Circle {
                center: pos,
                radius: clearance,
            };
            costmap_grid
                .iter_circle(&circle)
                .all(|pos| costmap_grid.get(pos).is_some_and(CostmapCell::maybe_free))
        }
    }
}

/// Checks if all cells in the line is free
pub fn validate_line(line: Line, costmap_grid: &Costmap) -> bool {
    costmap_grid
        .iter_line(&line)
        .all(|pos| costmap_grid.get(pos).is_some_and(CostmapCell::maybe_free))
}

/// Checks if all cells in the line with a width is free
pub fn validate_thick_line(line: Line, clearance: f32, costmap_grid: &Costmap) -> bool {
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
) -> Costmap {
    let mut costmap = Costmap::new(map.width(), map.height(), COSTMAP_GRID_SCALE);

    // Update costmap from search grid
    costmap.iter_mut().for_each(|(x, y, costmap_cell)| {
        if let Some(&cell) = search_grid.get(Pos2 { x, y }) {
            *costmap_cell = match cell {
                0.0 => CostmapCell::Unknown,
                _ => CostmapCell::Searched,
            };
        };
    });

    map.iter().for_each(|(x, y, &cell)| {
        // Negative if don't want to go there, positive if want to go there
        // Non-zero cells are explored therefore we won't go there
        if cell == MapCell::Obstacle {
            costmap.set(Pos2 { x, y }, CostmapCell::Obstacle);
        };
    });

    costmap.iter_mut().for_each(|(x, y, costmap_cell)| {
        if let Some(&cell) = map.get(Pos2 { x, y }) {
            if cell == MapCell::Obstacle {
                *costmap_cell = CostmapCell::Obstacle
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
                .set_circle(circle.center, circle.radius, CostmapCell::DynamicObstacle);
        });

    costmap
}
