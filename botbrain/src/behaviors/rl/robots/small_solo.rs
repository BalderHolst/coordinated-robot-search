use std::time::Duration;

use crate::behaviors::rl::model::{self, Network};
use crate::{behaviors::BehaviorOutput, debug::DebugType, Robot, RobotRef};

use crate::behaviors::rl::{
    action::{self, Action},
    state, RlRobot,
};

use super::run_robot;

use burn::prelude::*;

type St = state::SmallSoloState;
type Ac = action::SquareAction<3, 5>;
type Net<B> = model::small::SmallNetwork<B>;

pub type SmallSoloRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |robot: &mut SmallSoloRlRobot<B>| {
        robot.visualize_state();
    })
}

impl<B: Backend, A: Action, N: Network<B, St, A>> RlRobot<B, St, A, N> {
    pub fn visualize_state(&mut self) {
        if !self.debug_enabled() {
            return;
        }

        let interpolated_lidar = self.state().lidar_rays().to_vec();

        self.debug(
            "",
            "Interpolated Lidar",
            DebugType::RobotRays(interpolated_lidar),
        );
    }
}
