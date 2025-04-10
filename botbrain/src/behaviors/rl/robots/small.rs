use std::time::Duration;

use crate::{
    behaviors::{common, BehaviorOutput},
    cast_robot,
    debug::DebugType,
    params, Robot, RobotRef,
};

use crate::behaviors::rl::{
    action::{self, Action},
    state, RlRobot, REACT_HZ, SEARCH_GRADIENT_RANGE,
};

pub type SmallRlRobot = RlRobot<state::SmallState, action::SquareAction<3, 5>>;

pub fn run(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    let robot = cast_robot::<SmallRlRobot>(robot);

    robot.visualize();

    robot.update_search_grid(time);
    robot.update_search_gradient();

    // Only update the control signal at a fixed rate
    if (time - robot.last_control_update).as_secs_f32() >= 1.0 / REACT_HZ
        || robot.model.is_controlled()
    {
        robot.last_control_update = time;
        robot.react();
    }

    robot.postbox.clean();

    let msgs = robot.postbox.empty();
    (robot.control.clone(), msgs)
}

impl<A: Action> RlRobot<state::SmallState, A> {
    fn visualize(&mut self) {
        if !self.debug_enabled() {
            return;
        }
        self.debug("", "Search Grid", DebugType::Grid(self.search_grid.clone()));

        let raw_lidar = self.lidar.points().map(|p| (p.angle, p.distance)).collect();
        self.debug("", "Actual Lidar", DebugType::RobotRays(raw_lidar));

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

    fn update_search_gradient(&mut self) {
        let (g, _) = common::gradient(
            self.pos,
            self.angle,
            SEARCH_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.search_grid,
        );
        self.debug("Search Gradient", "Vector", DebugType::Vector(g));
        self.debug("Search Gradient", "x", DebugType::Number(g.x));
        self.debug("Search Gradient", "y", DebugType::Number(g.y));
        self.debug("Search Gradient", "length", DebugType::Number(g.length()));
        self.debug("Search Gradient", "angle", DebugType::Number(g.angle()));
        self.search_gradient = g;
    }
}
