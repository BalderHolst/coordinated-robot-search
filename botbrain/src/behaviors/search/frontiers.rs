#![allow(unused)]

use std::collections::{HashSet, VecDeque};

use emath::Pos2;

use crate::behaviors::{search::pathing::NEIGHBORS_8, ScaledGrid};

use super::{
    costmap::{COSTMAP_SEARCHED, COSTMAP_UNKNOWN},
    pathing::NEIGHBORS_4,
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

fn make_frontier_regions(
    robot_pos: (usize, usize),
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> Vec<Vec<(usize, usize)>> {
    let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
    queue.push_back(robot_pos);

    let mut visited: HashSet<(usize, usize)> = HashSet::default();

    let mut clusters: Vec<Vec<(usize, usize)>> = Vec::new();
    for frontier in frontiers {
        if visited.contains(&frontier) {
            continue;
        }
        let mut cluster = Vec::new();
        queue.push_back(frontier);
        while let Some(pos) = queue.pop_front() {
            if visited.contains(&pos) {
                continue;
            }
            visited.insert(pos);
            cluster.push(pos);
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
        clusters.push(cluster);
    }

    todo!()
}

fn evaluate_frontier_regions(
    robot_pos: (usize, usize),
    frontiers: ScaledGrid<FrontierRegion>,
) -> Pos2 {
    // TODO: Choose the frontier region to explore
    // Could be a weighted based on the size of the region and distance to the robot?
    todo!()
}

pub fn evaluate_frontiers(
    robot_pos: Pos2,
    frontiers: HashSet<(usize, usize)>,
    costmap_grid: &ScaledGrid<f32>,
) -> Pos2 {
    // let robot_pos = {
    //     let tmp = costmap_grid.world_to_grid(robot_pos);
    //     (tmp.x as usize, tmp.y as usize)
    // };
    // let frontier_regions = make_frontier_regions(robot_pos, frontiers, costmap_grid);
    // let goal = evaluate_frontier_regions(robot_pos, frontier_regions);
    Pos2::new(25.0, -20.0)
}
