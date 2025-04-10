use std::time::Duration;

use crate::{behaviors::BehaviorOutput, cast_robot, RobotRef};

pub mod minimal;
pub mod small;

use super::{action::Action, state::State, RlRobot, REACT_HZ};

pub fn run_robot<S: State, A: Action>(
    robot: &mut RobotRef,
    time: Duration,
    update: fn(&mut RlRobot<S, A>),
) -> BehaviorOutput {
    let robot = cast_robot::<RlRobot<S, A>>(robot);

    robot.visualize();
    robot.update_search_grid(time);
    robot.update_search_gradient();

    update(robot);

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
