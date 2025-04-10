use std::time::Duration;

use crate::{
    behaviors::{
        rl::{action, state, RlRobot},
        BehaviorOutput,
    },
    RobotRef,
};

use super::run_robot;

pub type MinimalRlRobot = RlRobot<state::RayState, action::MinimalAction>;

pub fn run(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |_: &mut MinimalRlRobot| {})
}
