use emath::Pos2;

use crate::{behaviors::ScaledGrid, grid::Grid};

use super::{
    costmap::{COSTMAP_SEARCHED, COSTMAP_UNKNOWN},
    pathing::NEIGHBORS,
};

// TODO: Find frontier regions
pub fn evaluate_frontiers(robot_pos: Pos2, costmap_grid: &ScaledGrid<f32>) -> Pos2 {
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };
    Pos2::new(25.0, -20.0)
}

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

// TODO: Choose the frontier region to explore
fn evaluate_frontier_regions(
    robot_pos: Pos2,
    frontiers: Grid<FrontierRegion>,
    costmap_grid: &ScaledGrid<f32>,
) -> Pos2 {
    todo!()
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
enum FrontierRegion {
    #[default]
    Explored,
    Id(usize),
}

// TODO: Make a grid of the frontier regions
fn make_frontier_region(
    robot_pos: Pos2,
    frontiers: Pos2,
    costmap_grid: &ScaledGrid<f32>,
) -> Grid<FrontierRegion> {
    todo!()
}
