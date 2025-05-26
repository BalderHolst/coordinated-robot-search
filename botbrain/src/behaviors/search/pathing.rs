use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};

use emath::Pos2;

use crate::{params, shapes::Line};

use super::{
    costmap::{self},
    Costmap, ROBOT_OBSTACLE_CLEARANCE,
};

/// When a goal/point is within this distance from the robot
pub(super) const PATH_PLANNER_DISTANCE_TOLERANCE: f32 = params::RADIUS;
pub(super) const NEIGHBORS_4: [(isize, isize); 4] = [(0, 1), (1, 0), (0, -1), (-1, 0)];
pub(super) const NEIGHBORS_8: [(isize, isize); 8] = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (-1, 1),
    (1, -1),
    (-1, -1),
];

/// Constructs a path using straight line or A* algorithm as backup
pub fn find_path(robot_pos: Pos2, goal: Pos2, costmap_grid: &Costmap) -> Option<Vec<Pos2>> {
    find_straight_path(robot_pos, goal, costmap_grid).or_else(|| {
        find_a_star_path(
            robot_pos,
            goal,
            ROBOT_OBSTACLE_CLEARANCE * 1.5,
            costmap_grid,
        )
        .or_else(|| {
            // println!("Trying with smaller a-star clearance");
            find_a_star_path(robot_pos, goal, ROBOT_OBSTACLE_CLEARANCE, costmap_grid)
        })
    })
}

/// Constructs a path using a straight line
pub fn find_straight_path(
    robot_pos: Pos2,
    goal: Pos2,
    costmap_grid: &Costmap,
) -> Option<Vec<Pos2>> {
    let dir = (goal - robot_pos).normalized();
    let line = Line {
        start: robot_pos + dir * params::DIAMETER,
        end: goal + dir * params::DIAMETER,
    };
    // Check if all cells in the line to the goal are free
    costmap::validate_thick_line(line, ROBOT_OBSTACLE_CLEARANCE, costmap_grid)
        .then(|| vec![robot_pos, goal])
}

#[derive(Copy, Clone, Eq, PartialEq)]
/// Node for the A* algorithm
struct Node {
    position: (usize, usize),
    cost: usize,
    priority: usize,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap order
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

/// Calculate the euclidean distance between two points
pub fn euclidean_dist(a: (usize, usize), b: (usize, usize)) -> usize {
    // Euclidean distance
    (((a.0 as f64 - b.0 as f64).powi(2) + (a.1 as f64 - b.1 as f64).powi(2)).sqrt()) as usize
}

/// Constructs a path using the A* algorithm
pub fn find_a_star_path(
    robot_pos: Pos2,
    goal: Pos2,
    clearance: f32,
    costmap_grid: &Costmap,
) -> Option<Vec<Pos2>> {
    let world_size = costmap_grid.grid().size();

    // Do all A* stuff in the grid space to avoid working with floats
    let robot_pos = {
        let tmp = costmap_grid.pos_to_grid(robot_pos);
        (tmp.x as usize, tmp.y as usize)
    };
    let goal = {
        let tmp = costmap_grid.pos_to_grid(goal);
        (tmp.x as usize, tmp.y as usize)
    };

    let mut open_set = BinaryHeap::new();
    open_set.push(Node {
        position: robot_pos,
        cost: 0,
        priority: euclidean_dist(robot_pos, goal),
    });

    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    let mut cost_so_far: HashMap<(usize, usize), usize> = HashMap::new();
    cost_so_far.insert(robot_pos, 0);

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

        for (dx, dy) in &NEIGHBORS_4 {
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

            let new_pos_world = {
                let temp = Pos2::new(new_pos.0 as f32, new_pos.1 as f32);
                costmap_grid.grid_to_pos(temp)
            };
            // A little clearance for the robot to move
            if !costmap::validate_pos(new_pos_world, clearance, costmap_grid) {
                continue; // obstacle
            }

            let new_cost = cost_so_far[&current.position] + 1;

            if !cost_so_far.contains_key(&new_pos) || new_cost < cost_so_far[&new_pos] {
                cost_so_far.insert(new_pos, new_cost);
                let priority = new_cost + euclidean_dist(new_pos, goal);
                open_set.push(Node {
                    position: new_pos,
                    cost: new_cost,
                    priority,
                });
                came_from.insert(new_pos, current.position);
            }
        }
    }

    final_path
        .and_then(|path| {
            path.into_iter()
                .map(|(x, y)| Some(costmap_grid.grid_to_pos(Pos2::new(x as f32, y as f32))))
                .collect()
        })
        .map(|path| smooth_path(path, costmap_grid))
}

// Shorten the path by removing while still keeping a min distance to obstacles
pub fn smooth_path(path: Vec<Pos2>, costmap_grid: &Costmap) -> Vec<Pos2> {
    let mut smoothed_path = vec![];
    let mut prev_pos = path[0];
    let mut idx = 1;
    let mut cur_len = 0;
    loop {
        let line = Line {
            start: prev_pos,
            end: path[idx],
        };

        if !costmap::validate_thick_line(line, ROBOT_OBSTACLE_CLEARANCE, costmap_grid) {
            if cur_len > 0 {
                smoothed_path.push(prev_pos);
                prev_pos = path[idx - 1];
                cur_len = 0;
            } else {
                smoothed_path.push(prev_pos);
                prev_pos = path[idx];
                idx += 1;
            }
        } else {
            cur_len += 1;
            idx += 1;
        }

        if idx >= path.len() {
            break;
        }
    }
    smoothed_path.push(prev_pos);
    smoothed_path.push(path[path.len() - 1]);
    smoothed_path
}
