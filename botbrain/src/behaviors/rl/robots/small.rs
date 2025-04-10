use std::time::Duration;

use crate::{behaviors::BehaviorOutput, debug::DebugType, Robot, RobotRef};

use crate::behaviors::rl::{
    action::{self, Action},
    state, RlRobot,
};

use super::run_robot;

pub type SmallRlRobot = RlRobot<state::SmallState, action::SquareAction<3, 5>>;

pub fn run(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    run_robot(robot, time, |robot: &mut SmallRlRobot| {
        robot.visualize_state();
    })
}

impl<A: Action> RlRobot<state::SmallState, A> {
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
