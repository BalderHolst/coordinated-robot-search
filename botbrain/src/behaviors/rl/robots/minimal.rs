//! Implementation of [`MinimalRlRobot`].

use std::time::Duration;

use burn::prelude::Backend;

use crate::{
    behaviors::{
        rl::{action, network, state, RlRobot},
        BehaviorOutput,
    },
    RobotRef,
};

use super::run_robot;

type St = state::RayState;
type Ac = action::MinimalAction;
type Net<B> = network::tiny::TinyNet<B>;

/// A minimal RL robot using the minimal state, action and network
///
/// Used as a proof of concept to test that framework facilitates
/// learning.
pub type MinimalRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

/// Behavior function for the minimal RL robot
pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |_: &mut MinimalRlRobot<B>| {})
}

impl<B: Backend> network::TrainedNetwork<B, St, Ac> for Net<B> {
    fn bytes() -> &'static [u8] {
        include_bytes!("weights/minimal.bin")
    }
}
