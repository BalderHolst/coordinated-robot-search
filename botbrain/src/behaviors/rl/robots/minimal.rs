use std::time::Duration;

use burn::prelude::Backend;

use crate::{
    behaviors::{
        rl::{action, model, state, RlRobot},
        BehaviorOutput,
    },
    RobotRef,
};

use super::run_robot;

type St = state::RayState;
type Ac = action::MinimalAction;
type Net<B> = model::tiny::TinyNet<B>;

pub type MinimalRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |_: &mut MinimalRlRobot<B>| {})
}
