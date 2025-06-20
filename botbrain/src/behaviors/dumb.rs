//! Dumb robot with dumb behaviors

use super::{
    BehaviorFn, CamData, Control, DebugSoup, LidarData, Map, Postbox, Robot, RobotId, RobotPose,
};

pub const MENU: &[(&str, BehaviorFn)] = &[
    ("nothing", behaviors::nothing),
    ("straight", behaviors::straight),
    ("circle", behaviors::circle),
];

#[derive(Clone, Default)]
pub struct DumbRobot {
    id: RobotId,
    postbox: Postbox,
    debug_soup: DebugSoup,
}

impl Robot for DumbRobot {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_map(&mut self, _map: Map) {}

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

    fn any(&self) -> &dyn std::any::Any {
        self
    }

    fn any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

pub mod behaviors {
    use std::time::Duration;

    use crate::behaviors::BehaviorOutput;

    use super::*;

    pub fn nothing(_robot: &mut Box<dyn Robot>, _time: Duration) -> BehaviorOutput {
        (Control::default(), vec![])
    }

    pub fn straight(_robot: &mut Box<dyn Robot>, _time: Duration) -> BehaviorOutput {
        (
            Control {
                speed: 1.0,
                steer: 0.0,
            },
            vec![],
        )
    }

    pub fn circle(_robot: &mut Box<dyn Robot>, _time: Duration) -> BehaviorOutput {
        (
            Control {
                speed: 1.0,
                steer: 0.5,
            },
            vec![],
        )
    }
}
