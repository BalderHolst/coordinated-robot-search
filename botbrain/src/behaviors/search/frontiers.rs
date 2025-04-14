#![allow(unused)]

use core::f32;
use std::{
    cmp::Ordering,
    collections::{HashSet, VecDeque},
};

use emath::Pos2;

use crate::behaviors::{search::pathing::NEIGHBORS_8, ScaledGrid};

use super::{
    costmap::{COSTMAP_SEARCHED, COSTMAP_UNKNOWN},
    pathing::{self, NEIGHBORS_4},
};

const FRONTIER_REGION_SIZE_WEIGHT: f32 = 0.4;
const FRONTIER_REGION_DISTANCE_WEIGHT: f32 = 0.6;

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

    NEIGHBORS_4.iter().any(|(x, y)| {
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

pub fn find_frontiers(robot_pos: Pos2, costmap_grid: &ScaledGrid<f32>) -> HashSet<(usize, usize)> {
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };

    let mut frontiers: HashSet<(usize, usize)> = HashSet::new();
    let mut visited: HashSet<(usize, usize)> = HashSet::new();
    let mut queue = VecDeque::new();
    queue.push_back(robot_pos);

    while let Some(pos) = queue.pop_front() {
        if visited.contains(&pos) {
            continue;
        } else {
            visited.insert(pos);
        }

        if is_frontier(pos, costmap_grid) {
            frontiers.insert(pos);
        } else if let Some(&cell) = costmap_grid.grid().get(pos.0, pos.1) {
            if cell != COSTMAP_SEARCHED {
                continue;
            }
            for (dx, dy) in &NEIGHBORS_4 {
                let new_pos = (pos.0 as isize + dx, pos.1 as isize + dy);
                if new_pos.0 < 0 || new_pos.1 < 0 {
                    continue;
                }
                let new_pos = (new_pos.0 as usize, new_pos.1 as usize);
                if let Some(&cell) = costmap_grid.grid().get(new_pos.0, new_pos.1) {
                    if cell == COSTMAP_SEARCHED && !visited.contains(&(new_pos.0, new_pos.1)) {
                        queue.push_back((new_pos.0, new_pos.1));
                    }
                }
            }
        }
    }

    frontiers
}

pub fn make_frontier_regions(
    robot_pos: (usize, usize),
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> Vec<Vec<(usize, usize)>> {
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

    let mut visited: HashSet<(usize, usize)> = HashSet::default();

    let mut regions: Vec<Vec<(usize, usize)>> = Vec::new();
    for frontier in &frontiers {
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
                if let Some(&cell) = costmap_grid.grid().get(new_pos.0, new_pos.1) {
                    if cell == COSTMAP_SEARCHED
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

fn find_frontiers_region_closest_pos(
    robot_pos: (usize, usize),
    frontier_region: &[(usize, usize)],
) -> (usize, usize) {
    frontier_region
        .iter()
        .min_by(|&&f1, &&f2| {
            pathing::heuristic(robot_pos, f1).cmp(&pathing::heuristic(robot_pos, f2))
        })
        .unwrap()
        .to_owned()
}

pub fn evaluate_frontier_regions(
    robot_pos: (usize, usize),
    frontier_regions: Vec<Vec<(usize, usize)>>,
    costmap_grid: &ScaledGrid<f32>,
) -> Option<(usize, usize)> {
    // The furthest frontier region's closest frontier
    let mut furthest_frontier = f32::NEG_INFINITY;
    // The biggest frontier region
    let mut biggest_frontier = 0;

    let closest_region_frontier: Vec<_> = frontier_regions
        .iter()
        // Find the weight of the frontier regions
        .map(|region| {
            let closest_region_frontier = find_frontiers_region_closest_pos(robot_pos, region);
            let huristic = pathing::heuristic(closest_region_frontier, robot_pos) as f32;

            let size = region.len();

            furthest_frontier = furthest_frontier.max(huristic);
            biggest_frontier = biggest_frontier.max(region.len());

            (huristic, size, closest_region_frontier)
        })
        .collect();

    let best_frontier =
        closest_region_frontier
            .into_iter()
            .max_by(|&(h1, s1, p1), &(h2, s2, p2)| {
                let weight1 = FRONTIER_REGION_SIZE_WEIGHT * (s1 as f32 / biggest_frontier as f32)
                    + FRONTIER_REGION_DISTANCE_WEIGHT * (1.0 - h1 / furthest_frontier);
                let weight2 = FRONTIER_REGION_SIZE_WEIGHT * (s2 as f32 / biggest_frontier as f32)
                    + FRONTIER_REGION_DISTANCE_WEIGHT * (1.0 - h2 / furthest_frontier);

                weight1.partial_cmp(&weight2).unwrap_or(Ordering::Equal)
            })?;

    Some(best_frontier.2)
}

pub fn evaluate_frontiers(
    robot_pos: Pos2,
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> Option<Pos2> {
    // For working with grid coordinates
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };

    let frontier_regions = make_frontier_regions(robot_pos, frontiers, costmap_grid);
    let goal = {
        let goal = evaluate_frontier_regions(robot_pos, frontier_regions, costmap_grid)?;
        Pos2::new(goal.0 as f32, goal.1 as f32)
    };
    Some(costmap_grid.grid_to_world(goal))
}
