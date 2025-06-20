//! Implementation of [`SmallRlRobot`].

use std::time::Duration;

use crate::behaviors::rl::network;
use crate::{behaviors::BehaviorOutput, debug_soup::DebugItem, Robot, RobotRef};

use crate::behaviors::rl::{
    action::{self, Action},
    state, RlRobot,
};

use super::run_robot;

use burn::prelude::*;

type St = state::SmallState;
type Ac = action::SquareAction<3, 5>;
type Net<B> = network::small::SmallNetwork<B>;

/// A reinforcement learning search robot
pub type SmallRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

/// Behavior function for the small RL robot
pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |robot: &mut SmallRlRobot<B>| {
        robot.visualize_state();
    })
}

impl<B: Backend> network::TrainedNetwork<B, St, Ac> for Net<B> {
    fn bytes() -> &'static [u8] {
        include_bytes!("weights/small.bin")
    }
}

impl<B: Backend, A: Action, N: network::Network<B, St, A>> RlRobot<B, St, A, N> {
    fn visualize_state(&mut self) {
        if !self.debug_enabled() {
            return;
        }

        let interpolated_lidar = self.state().lidar_rays().to_vec();

        self.debug(
            "",
            "Interpolated Lidar",
            DebugItem::RobotRays(interpolated_lidar),
        );
    }
}
