//! Implementation of [`PolarRlRobot`].

use std::time::Duration;

use crate::behaviors::rl::network;
use crate::{behaviors::BehaviorOutput, Robot, RobotRef};

use crate::behaviors::rl::{action, state, RlRobot};

use super::run_robot;

use burn::prelude::*;

type St = state::PolarState;
type Ac = action::SliderAction<7>;
type Net<B> = network::small::SmallNetwork<B>;

/// A reinforcement learning search robot using polar coordinates for its state
pub type SmallPolarRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

/// Behavior function for the polar RL robot
pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |robot: &mut SmallPolarRlRobot<B>| {
        robot.visualize_state();
    })
}

impl<B: Backend> network::TrainedNetwork<B, St, Ac> for Net<B> {
    fn bytes() -> &'static [u8] {
        include_bytes!("weights/small_polar.bin")
    }
}

impl<B: Backend> RlRobot<B, St, Ac, Net<B>> {
    fn visualize_state(&mut self) {
        if !self.debug_enabled() {
            return;
        }

        let state = self.state();
        let soup = &mut self.debug_soup;

        state.visualize(soup);
    }
}
