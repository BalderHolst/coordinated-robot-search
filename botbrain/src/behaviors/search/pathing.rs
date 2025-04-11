use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};

use emath::Pos2;

use crate::{behaviors::ScaledGrid, shapes::Line};

use super::costmap::{self, COSTMAP_DYNAMIC_OBSTACLE, COSTMAP_OBSTACLE};

pub(super) const PATH_PLANNER_GOAL_TOLERANCE: f32 = 0.5;

pub fn find_path(robot_pos: Pos2, goal: Pos2, costmap_grid: &ScaledGrid<f32>) -> Option<Vec<Pos2>> {
    find_straight_path(robot_pos, goal, costmap_grid)
        .or_else(|| find_a_star_path(robot_pos, goal, costmap_grid))
}

// TODO: Find frontier regions
pub fn find_frontiers(_costmap_grid: &ScaledGrid<f32>) {
    todo!()
}

// TODO: Choose the frontier region to explore
pub fn evaluate_frontiers() -> Pos2 {
    Pos2::new(25.0, -20.0)
}

pub fn find_straight_path(
    robot_pos: Pos2,
    goal: Pos2,
    costmap_grid: &ScaledGrid<f32>,
) -> Option<Vec<Pos2>> {
    let line = Line {
        start: robot_pos,
        end: goal,
    };
    // Check if all cells in the line to the goal are free
    costmap::validate_line(line, costmap_grid).then(|| vec![robot_pos, goal])
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct Node {
    position: (usize, usize),
    cost: usize,
    priority: usize,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Max-heap ordering
        other
            .priority
            .cmp(&self.priority)
            .then_with(|| self.cost.cmp(&other.cost))
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn heuristic(a: (usize, usize), b: (usize, usize)) -> usize {
    // Manhattan distance
    ((a.0 as i32 - b.0 as i32).abs() + (a.1 as i32 - b.1 as i32).abs()) as usize
}

pub fn find_a_star_path(
    robot_pos: Pos2,
    goal: Pos2,
    costmap_grid: &ScaledGrid<f32>,
) -> Option<Vec<Pos2>> {
    let world_size = costmap_grid.grid().size();

    // Do all A* stuff in the grid space to avoid working with floats
    let robot_pos = {
        let tmp = costmap_grid.world_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };
    let goal = {
        let tmp = costmap_grid.world_to_grid(goal);
        (tmp.x as usize, tmp.y as usize)
    };

    let mut open_set = BinaryHeap::new();
    open_set.push(Node {
        position: robot_pos,
        cost: 0,
        priority: heuristic(robot_pos, goal),
    });

    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    let mut cost_so_far: HashMap<(usize, usize), usize> = HashMap::new();
    cost_so_far.insert(robot_pos, 0);

    let neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)];
    let mut final_path = None;
    while let Some(current) = open_set.pop() {
        if current.position == goal {
            let mut path = vec![current.position];
            let mut current_pos = current.position;
            while let Some(&prev) = came_from.get(&current_pos) {
                path.push(prev);
                current_pos = prev;
            }
            path.reverse();
            final_path = Some(path);
        }

        for (dx, dy) in &neighbors {
            let new_x = current.position.0 as isize + dx;
            let new_y = current.position.1 as isize + dy;

            // Filter out negative coordinates
            if new_x < 0 || new_y < 0 {
                continue;
            }

            let new_pos = (new_x as usize, new_y as usize);

            // Filter out coordinates that are out of bounds
            if new_pos.0 >= world_size.0 || new_pos.1 >= world_size.1 {
                continue;
            }

            let new_pos_value = *costmap_grid.grid().get(new_pos.0, new_pos.1).unwrap();
            if new_pos_value == COSTMAP_OBSTACLE || new_pos_value == COSTMAP_DYNAMIC_OBSTACLE {
                continue; // obstacle
            }

            let new_cost = cost_so_far[&current.position] + 1;

            if !cost_so_far.contains_key(&new_pos) || new_cost < cost_so_far[&new_pos] {
                cost_so_far.insert(new_pos, new_cost);
                let priority = new_cost + heuristic(new_pos, goal);
                open_set.push(Node {
                    position: new_pos,
                    cost: new_cost,
                    priority,
                });
                came_from.insert(new_pos, current.position);
            }
        }
    }

    final_path.and_then(|path| {
        path.into_iter()
            .map(|(x, y)| Some(costmap_grid.grid_to_world(Pos2::new(x as f32, y as f32))))
            .collect()
    })
}
