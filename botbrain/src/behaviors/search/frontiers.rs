#![allow(unused)]

use std::collections::HashSet;

use emath::Pos2;

use crate::behaviors::ScaledGrid;

use super::{
    costmap::{COSTMAP_SEARCHED, COSTMAP_UNKNOWN},
    pathing::NEIGHBORS,
};

/// Frontier is a known cell with unknown neighbors
/// pos is the position of the cell in the underlying grid of the ScaledGrid
pub fn is_frontier(pos: (usize, usize), costmap_grid: &ScaledGrid<f32>) -> bool {
    if let Some(&cell) = costmap_grid.grid().get(pos.0, pos.1) {
        if cell != COSTMAP_SEARCHED {
            return false;
        }
    } else {
        return false;
    }

    NEIGHBORS.iter().any(|(x, y)| {
        let new_pos = (pos.0 as isize + x, pos.1 as isize + y);
        if new_pos.0 < 0 || new_pos.1 < 0 {
            false
        } else if let Some(&cell) = costmap_grid
            .grid()
            .get(new_pos.0 as usize, new_pos.1 as usize)
        {
            cell == COSTMAP_UNKNOWN
        } else {
            false
        }
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
enum FrontierRegion {
    #[default]
    Explored,
    Id(usize),
}

fn find_frontiers(robot_pos: Pos2, costmap_grid: &ScaledGrid<f32>) -> HashSet<(usize, usize)> {
    // TODO: Start from the robot position and find all frontiers
    // Use BFS to find all frontiers?
    todo!()
}

fn make_frontier_regions(
    robot_pos: Pos2,
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> ScaledGrid<FrontierRegion> {
    // TODO: Group the frontiers into regions
    // Using region growing?
    todo!()
}

fn evaluate_frontier_regions(robot_pos: Pos2, frontiers: ScaledGrid<FrontierRegion>) -> Pos2 {
    // TODO: Choose the frontier region to explore
    // Could be a weighted based on the size of the region and distance to the robot?
    todo!()
}

pub fn evaluate_frontiers(robot_pos: Pos2, costmap_grid: &ScaledGrid<f32>) -> Pos2 {
    // let frontiers = find_frontiers(robot_pos, costmap_grid);
    // let frontier_regions = make_frontier_regions(robot_pos, frontiers, costmap_grid);
    // let goal = evaluate_frontier_regions(robot_pos, frontier_regions);
    Pos2::new(25.0, -20.0)
}
