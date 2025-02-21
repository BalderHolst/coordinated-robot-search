use std::time::Instant;

use super::{Control, Robot};

pub fn search(robot: &mut Robot, time: Instant) -> Control {
    robot.update_search_grid(time);

    robot.clear_processed();

    Control {
        speed: 0.0,
        steer: 0.1,
    }
}
