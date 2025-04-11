use emath::Pos2;

use crate::{behaviors::ScaledGrid, shapes::Line};

use super::costmap;

pub(super) const PATH_PLANNER_GOAL_TOLERANCE: f32 = 0.5;

pub fn find_path(
    robot_pos: Pos2,
    goal: Pos2,
    costmap_grid: &ScaledGrid<f32>,
) -> Result<Vec<Pos2>, ()> {
    find_straight_path(robot_pos, goal, costmap_grid)
        .or_else(|_| find_a_star_path(robot_pos, goal, costmap_grid))
}

// TODO: Find frontier regions
pub fn find_frontiers(_costmap_grid: &ScaledGrid<f32>) {
    todo!()
}

// TODO: Choose the frontier region to explore
pub fn evaluate_frontiers() -> Pos2 {
    Pos2::new(25.0, -20.0)
}

// TODO: Use A* to find the shortest path to the goal
pub fn find_a_star_path(
    robot_pos: Pos2,
    goal: Pos2,
    _costmap_grid: &ScaledGrid<f32>,
) -> Result<Vec<Pos2>, ()> {
    Ok(vec![
        robot_pos,
        Pos2::new(25.0, 25.0),
        Pos2::new(-25.0, 25.0),
        Pos2::new(-25.0, -25.0),
        Pos2::new(25.0, -25.0),
        goal,
    ])
}
pub fn find_straight_path(
    robot_pos: Pos2,
    goal: Pos2,
    costmap_grid: &ScaledGrid<f32>,
) -> Result<Vec<Pos2>, ()> {
    let line = Line {
        start: robot_pos,
        end: goal,
    };
    // Check if all cells in the line to the goal are free
    costmap::validate_line(line, costmap_grid)
        .then(|| vec![robot_pos, goal])
        .ok_or(())
}
