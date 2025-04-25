#![allow(unused)]

use core::f32;
use std::{
    cmp::Ordering,
    collections::{HashSet, VecDeque},
    f32::consts::PI,
};

use emath::{Pos2, Vec2};

use crate::{
    behaviors::{search::pathing::NEIGHBORS_8, ScaledGrid},
    params, utils,
};

use super::{
    costmap::{self, COSTMAP_SEARCHED, COSTMAP_UNKNOWN},
    pathing::{self, NEIGHBORS_4},
};

const FRONTIER_REGION_SIZE_WEIGHT: f32 = 0.2;
const FRONTIER_REGION_DISTANCE_WEIGHT: f32 = 0.4;
const FRONTIER_REGION_TURN_WEIGHT: f32 = 0.4;

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

    NEIGHBORS_8.iter().any(|(x, y)| {
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

/// Finds the frontiers from the robot position using the costmap
pub fn find_frontiers(robot_pos: Pos2, costmap_grid: &ScaledGrid<f32>) -> HashSet<(usize, usize)> {
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
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
        } else if let Some(&cell) = costmap_grid.grid().get(pos.0, pos.1) {
            if cell != COSTMAP_SEARCHED {
                continue;
            }
            for (dx, dy) in &NEIGHBORS_8 {
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

/// Constructs the frontier regions from the frontiers
pub fn make_frontier_regions(
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

/// Find the closest pos in the best frontier region to go to
fn find_frontiers_region_closest_pos(
    robot_pos: (usize, usize),
    frontier_region: &[(usize, usize)],
) -> (usize, usize) {
    frontier_region
        .iter()
        .min_by(|&&f1, &&f2| {
            let d1 = pathing::euclidean_dist(robot_pos, f1);
            let d2 = pathing::euclidean_dist(robot_pos, f2);
            let d1_valid = (d1 as f32) > params::DIAMETER;
            let d2_valid = (d2 as f32) > params::DIAMETER;
            if !d1_valid && !d2_valid {
                Ordering::Equal
            } else if !d1_valid {
                Ordering::Greater
            } else if !d2_valid {
                Ordering::Less
            } else {
                d1.cmp(&d2)
            }
        })
        .unwrap()
        .to_owned()
}

/// Evaluates the fontier regions and returns the best frontier
pub fn evaluate_frontier_regions(
    robot_pos: (usize, usize),
    robot_angle: f32,
    frontier_regions: Vec<Vec<(usize, usize)>>,
    costmap_grid: &ScaledGrid<f32>,
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
                .map(|&pos| {
                    let dist = pathing::euclidean_dist(pos, robot_pos) as f32;
                    if dist < params::DIAMETER * 2.0 {
                        // Very bad to be on the robot pos
                        return (f32::MAX, 0, PI, pos);
                    }
                    let pos_world = {
                        let temp = Pos2::new(pos.0 as f32, pos.1 as f32);
                        costmap_grid.grid_to_world(temp)
                    };

                    let is_valid =
                        costmap::validate_pos(pos_world, params::DIAMETER * 2.0, costmap_grid);
                    if !is_valid {
                        // Very bad to be too close to obstacles
                        return (f32::MAX, 0, PI, pos);
                    }

                    let size = region.len();

                    let frontier_angle = Vec2::new(
                        pos_world.x - robot_pos.0 as f32,
                        pos_world.y - robot_pos.1 as f32,
                    )
                    .angle();
                    let angle_to_target =
                        utils::normalize_angle(frontier_angle - robot_angle).abs();

                    furthest_frontier = furthest_frontier.max(dist);
                    biggest_frontier = biggest_frontier.max(region.len());

                    (dist, size, angle_to_target, pos)
                })
                .collect::<Vec<(f32, usize, f32, (usize, usize))>>()
        })
        .collect();

    let best_frontier = frontiers.into_iter().max_by(
        |&(dist_1, size_1, turn_1, pos_1), &(dist_2, size_2, turn_2, pos_2)| {
            let weight1 = FRONTIER_REGION_SIZE_WEIGHT * (size_1 as f32 / biggest_frontier as f32)
                + FRONTIER_REGION_DISTANCE_WEIGHT * (1.0 - dist_1 / furthest_frontier)
                + FRONTIER_REGION_TURN_WEIGHT * (1.0 - turn_1 / PI);

            let weight2 = FRONTIER_REGION_SIZE_WEIGHT * (size_2 as f32 / biggest_frontier as f32)
                + FRONTIER_REGION_DISTANCE_WEIGHT * (1.0 - dist_2 / furthest_frontier)
                + FRONTIER_REGION_TURN_WEIGHT * (1.0 - turn_2 / PI);

            weight1.partial_cmp(&weight2).unwrap_or(Ordering::Equal)
        },
    )?;

    Some(best_frontier.3)
}

/// Find the best frontier to go to
pub fn evaluate_frontiers(
    robot_pos: Pos2,
    robot_angle: f32,
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> Option<Pos2> {
    // For working with grid coordinates
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };

    let frontier_regions = make_frontier_regions(frontiers, costmap_grid);
    let goal = {
        let goal_grid =
            evaluate_frontier_regions(robot_pos, robot_angle, frontier_regions, costmap_grid)?;
        costmap_grid.grid_to_world(Pos2::new(goal_grid.0 as f32, goal_grid.1 as f32))
    };
    Some(goal)
}
