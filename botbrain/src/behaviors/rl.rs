use std::time::Duration;

use crate::{debug::DebugSoup, CamData, LidarData, Postbox, Robot, RobotId, RobotPose, Vec2};

use super::{cast_robot, BehaviorFn, BehaviorOutput, RobotRef};

pub const MENU: &[(&str, BehaviorFn)] = &[("nn", run_nn)];

#[derive(Clone, Default)]
pub struct RlRobot {
    id: RobotId,
    postbox: Postbox,
    debug_soup: DebugSoup,
}

impl Robot for RlRobot {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_world_size(&mut self, _size: Vec2) {}

    fn get_postbox(&self) -> &Postbox {
        &self.postbox
    }

    fn get_postbox_mut(&mut self) -> &mut Postbox {
        &mut self.postbox
    }

    fn get_debug_soup(&self) -> &DebugSoup {
        &self.debug_soup
    }

    fn get_debug_soup_mut(&mut self) -> &mut DebugSoup {
        &mut self.debug_soup
    }

    fn input_pose(&mut self, _pos: RobotPose) {}
    fn input_cam(&mut self, _cam: CamData) {}
    fn input_lidar(&mut self, _lidar: LidarData) {}

    fn clone_box(&self) -> Box<dyn Robot> {
        Box::new(self.clone())
    }

    fn any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

fn run_nn(robot: &mut RobotRef, _time: Duration) -> BehaviorOutput {
    let _robot = cast_robot::<RlRobot>(robot);
    todo!();
}
