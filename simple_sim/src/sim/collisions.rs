use botbrain::{
    params::{DIAMETER, RADIUS},
    Pos2, Vec2,
};

use crate::world::{Cell, World};

use super::RobotState;

pub fn resolve_robot_collisions(me: &mut RobotState, all: &[RobotState]) {
    for other in all {
        if me.id == other.id {
            continue;
        }
        if (me.pose.pos - other.pose.pos).length() < DIAMETER {
            let diff = me.pose.pos - other.pose.pos;
            let overlap = DIAMETER - diff.length();
            let dir = diff.normalized() * overlap / 2.0;
            me.pose.pos += dir;
        }
    }
}

pub fn resolve_world_collisions(robot_state: &mut RobotState, world: &World) {
    // Look in a circle around the robot
    let radius = DIAMETER / 2.0 * 1.4 / world.scale();
    let center = world.pos_to_grid(robot_state.pose.pos);
    let mut nudge = Vec2::ZERO;
    let mut nudgers = 0;
    let grid = world.grid();
    for (x, y) in grid.iter_circle(center, radius) {
        let cell = grid.get(x, y);
        if matches!(cell, Some(Cell::Wall) | None) {
            let cell_center = world.grid_to_pos(Pos2 {
                x: x as f32,
                y: y as f32,
            });
            let diff = robot_state.pose.pos - cell_center;

            let overlap = DIAMETER / 2.0 - diff.length() + 0.5 * world.scale();
            if overlap < 0.0 {
                continue;
            }

            let diff = match diff.x.abs() > diff.y.abs() {
                true => Vec2::new(diff.x, 0.0),
                false => Vec2::new(0.0, diff.y),
            };
            let dir = diff.normalized();

            nudge += dir * overlap;
            nudgers += 1;
        }
    }

    if nudgers > 0 {
        // Only nudge in the direction with most difference
        robot_state.pose.pos += nudge / nudgers as f32;
    }
}

pub fn resolve_border_collisions(robot_state: &mut RobotState, world: &World) {
    let pos = &mut robot_state.pose.pos;
    if pos.x - RADIUS < -world.width() / 2.0 {
        pos.x = -world.width() / 2.0 + RADIUS;
    }
    if pos.x + RADIUS > world.width() / 2.0 {
        pos.x = world.width() / 2.0 - RADIUS;
    }
    if pos.y - RADIUS < -world.height() / 2.0 {
        pos.y = -world.height() / 2.0 + RADIUS;
    }
    if pos.y + RADIUS > world.height() / 2.0 {
        pos.y = world.height() / 2.0 - RADIUS;
    }
}
