use std::time::Duration;

use crate::behaviors::rl::model::{self, Network};
use crate::{behaviors::BehaviorOutput, debug::DebugType, Robot, RobotRef};

use crate::behaviors::rl::{
    action::{self, Action},
    state, RlRobot,
};

use super::run_robot;

use burn::prelude::*;

type St = state::SmallState;
type Ac = action::SquareAction<3, 5>;
type Net<B> = model::small::SmallNetwork<B>;

pub type SmallRlRobot<B> = RlRobot<B, St, Ac, Net<B>>;

pub fn run<B: Backend>(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |robot: &mut SmallRlRobot<B>| {
        robot.visualize_state();
    })
}

impl<B: Backend, A: Action, N: Network<B, St, A>> RlRobot<B, St, A, N> {
    pub fn visualize_state(&mut self) {
        if !self.debug_enabled() {
            return;
        }

        let interpolated_lidar = self.state().lidar_rays().to_vec();

        if let Some(shortest_ray) = interpolated_lidar
            .iter()
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .cloned()
        {
            self.debug(
                "",
                "Shortest Lidar Angle",
                DebugType::Number(shortest_ray.0),
            );
            self.debug(
                "",
                "Shortest Lidar Distance",
                DebugType::Number(shortest_ray.1),
            );
            self.debug(
                "",
                "Shortest Ray",
                DebugType::RobotRays(vec![(shortest_ray.0, shortest_ray.1)]),
            );
        }

        self.debug(
            "",
            "Interpolated Lidar",
            DebugType::RobotRays(interpolated_lidar),
        );
    }
}
