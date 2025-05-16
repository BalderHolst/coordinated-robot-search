use core::f32;
use std::{
    cmp::Ordering,
    collections::{HashSet, VecDeque},
    f32::consts::PI,
};

use emath::{Pos2, Vec2};

use crate::{behaviors::search::pathing::NEIGHBORS_8, params, utils};

use super::{costmap, pathing, Costmap, CostmapCell};

/// Weights for frontier evaluation
/// Higher weights means more importance
#[derive(Clone, Copy)]
pub(super) struct FrontierEvaluationWeights {
    /// Weight for the size of the frontier region
    pub frontier_region_size: f32,
    /// Weight for the distance to the frontier
    pub frontier_distance: f32,
    /// Weight for the turn to the frontier
    pub frontier_turn: f32,
}

impl Default for FrontierEvaluationWeights {
    fn default() -> Self {
        Self {
            frontier_region_size: 0.1,
            frontier_distance: 0.3,
            frontier_turn: 0.6,
        }
    }
}

/// Frontier is a known cell with unknown neighbors
/// pos is the position of the cell in the underlying grid of the ScaledGrid
pub fn is_frontier(pos: (usize, usize), costmap_grid: &Costmap) -> bool {
    if !matches!(
        costmap_grid.grid().get(pos.0, pos.1),
        Some(CostmapCell::Searched)
    ) {
        return false;
    }

    NEIGHBORS_8.iter().any(|(x, y)| {
        let new_pos = (pos.0 as isize + x, pos.1 as isize + y);
        if new_pos.0 < 0 || new_pos.1 < 0 {
            false
        } else if let Some(cell) = costmap_grid
            .grid()
            .get(new_pos.0 as usize, new_pos.1 as usize)
        {
            matches!(cell, CostmapCell::Unknown)
        } else {
            false
        }
    })
}

/// Finds the frontiers from the robot position using the costmap
pub fn find_frontiers(robot_pos: Pos2, costmap_grid: &Costmap) -> HashSet<(usize, usize)> {
    let robot_pos = {
        let tmp = costmap_grid.pos_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };

    let mut frontiers: HashSet<(usize, usize)> = HashSet::new();
    let mut visited: HashSet<(usize, usize)> = HashSet::new();
    let mut queue = VecDeque::new();
    for (dx, dy) in &NEIGHBORS_8 {
        let new_pos = (robot_pos.0 as isize + dx, robot_pos.1 as isize + dy);
        queue.push_back((new_pos.0 as usize, new_pos.1 as usize));
    }

    while let Some(pos) = queue.pop_front() {
        if visited.contains(&pos) {
            continue;
        } else {
            visited.insert(pos);
        }

        if is_frontier(pos, costmap_grid) {
            frontiers.insert(pos);
        }
        if let Some(cell) = costmap_grid.grid().get(pos.0, pos.1) {
            if !matches!(cell, CostmapCell::Searched) {
                continue;
            }
            for (dx, dy) in &NEIGHBORS_8 {
                let new_pos = (pos.0 as isize + dx, pos.1 as isize + dy);
                if new_pos.0 < 0 || new_pos.1 < 0 {
                    continue;
                }
                let new_pos = (new_pos.0 as usize, new_pos.1 as usize);
                if let Some(cell) = costmap_grid.grid().get(new_pos.0, new_pos.1) {
                    if matches!(cell, CostmapCell::Searched)
                        && !visited.contains(&(new_pos.0, new_pos.1))
                    {
                        queue.push_back((new_pos.0, new_pos.1));
                    }
                }
            }
        }
    }

    frontiers
}

/// Constructs the frontier regions from the frontiers
pub fn make_frontier_regions(
    frontiers: &HashSet<(usize, usize)>,
    costmap_grid: &Costmap,
) -> Vec<Vec<(usize, usize)>> {
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

    let mut visited: HashSet<(usize, usize)> = HashSet::default();

    let mut regions: Vec<Vec<(usize, usize)>> = Vec::new();
    for frontier in frontiers {
        if visited.contains(frontier) {
            continue;
        }
        let mut region = Vec::new();
        queue.push_back(*frontier);
        while let Some(pos) = queue.pop_front() {
            if visited.contains(&pos) {
                continue;
            }
            visited.insert(pos);
            region.push(pos);
            for (dx, dy) in &NEIGHBORS_8 {
                let new_pos = (pos.0 as isize + dx, pos.1 as isize + dy);
                if new_pos.0 < 0 || new_pos.1 < 0 {
                    continue;
                }
                let new_pos = (new_pos.0 as usize, new_pos.1 as usize);
                if let Some(cell) = costmap_grid.grid().get(new_pos.0, new_pos.1) {
                    if matches!(cell, CostmapCell::Searched)
                        && !visited.contains(&(new_pos.0, new_pos.1))
                        && frontiers.contains(&new_pos)
                    {
                        queue.push_back((new_pos.0, new_pos.1));
                    }
                }
            }
        }
        regions.push(region);
    }

    regions
}

/// Evaluates the fontier regions and returns the best frontier
pub fn evaluate_frontier_regions(
    robot_pos: (usize, usize),
    robot_angle: f32,
    evaluation_weights: FrontierEvaluationWeights,
    clearance: f32,
    frontier_regions: Vec<Vec<(usize, usize)>>,
    costmap_grid: &Costmap,
) -> Option<(usize, usize)> {
    // The furthest frontier region's closest frontier
    let mut furthest_frontier = f32::NEG_INFINITY;
    // The biggest frontier region
    let mut biggest_frontier = 0;

    let frontiers: Vec<(f32, usize, f32, (usize, usize))> = frontier_regions
        .iter()
        // Find the weight of the frontier regions
        .flat_map(|region| {
            // let closest_region_frontier = find_frontiers_region_closest_pos(robot_pos, region);
            region
                .iter()
                .filter_map(|&pos| {
                    let dist_grid = pathing::euclidean_dist(pos, robot_pos) as f32;
                    // FIX: Remove 2.0
                    if dist_grid < params::DIAMETER / costmap_grid.scale() * 2.0 {
                        // Very bad to be on the robot pos
                        return None;
                    }
                    let pos_map = {
                        let temp = Pos2::new(pos.0 as f32, pos.1 as f32);
                        costmap_grid.grid_to_pos(temp)
                    };

                    let is_valid = costmap::validate_pos(pos_map, clearance, costmap_grid);
                    if !is_valid {
                        // Very bad to be too close to obstacles
                        return None;
                    }

                    let size = region.len();

                    let frontier_angle = Vec2::new(
                        pos.0 as f32 - robot_pos.0 as f32,
                        pos.1 as f32 - robot_pos.1 as f32,
                    )
                    .angle();

                    let angle_to_target =
                        utils::normalize_angle(frontier_angle - robot_angle).abs();

                    furthest_frontier = furthest_frontier.max(dist_grid);
                    biggest_frontier = biggest_frontier.max(region.len());

                    Some((dist_grid, size, angle_to_target, pos))
                })
                .collect::<Vec<(f32, usize, f32, (usize, usize))>>()
        })
        .collect();

    let best_frontier = frontiers.into_iter().max_by(
        |&(dist_1, size_1, turn_1, _pos_1), &(dist_2, size_2, turn_2, _pos_2)| {
            let weight1 = evaluation_weights.frontier_region_size
                * (size_1 as f32 / biggest_frontier as f32)
                + evaluation_weights.frontier_distance * (1.0 - dist_1 / furthest_frontier)
                + evaluation_weights.frontier_turn * (1.0 - turn_1 / PI);

            let weight2 = evaluation_weights.frontier_region_size
                * (size_2 as f32 / biggest_frontier as f32)
                + evaluation_weights.frontier_distance * (1.0 - dist_2 / furthest_frontier)
                + evaluation_weights.frontier_turn * (1.0 - turn_2 / PI);

            weight1.partial_cmp(&weight2).unwrap_or(Ordering::Equal)
        },
    )?;

    Some(best_frontier.3)
}

/// Find the best frontier to go to
/// Returns `None` if no valid frontiers are found
pub fn evaluate_frontiers(
    robot_pos: Pos2,
    robot_angle: f32,
    evaluation_weights: FrontierEvaluationWeights,
    clearance: f32,
    frontiers: &HashSet<(usize, usize)>,
    costmap_grid: &Costmap,
) -> Option<Pos2> {
    // For working with grid coordinates
    let robot_pos_grid = {
        let tmp = costmap_grid.pos_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };

    let frontier_regions = make_frontier_regions(frontiers, costmap_grid);
    let goal = {
        let goal_grid = evaluate_frontier_regions(
            robot_pos_grid,
            robot_angle,
            evaluation_weights,
            clearance,
            frontier_regions,
            costmap_grid,
        )?;
        costmap_grid.grid_to_pos(Pos2::new(goal_grid.0 as f32, goal_grid.1 as f32))
    };
    Some(goal)
}
