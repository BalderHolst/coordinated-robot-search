//! Dumb robot with dumb behaviors

use {
    super::{
        BehaviorFn, CamData, Control, DebugSoup, LidarData, Postbox, Robot, RobotId,
        RobotParameters, RobotPose,
    },
    emath::Vec2,
};

pub const MENU: &[(&str, BehaviorFn)] = &[
    ("nothing", behaviors::nothing),
    ("circle", behaviors::circle),
];

#[derive(Clone, Default)]
pub struct DumbRobot {
    id: RobotId,
    params: RobotParameters,
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

    fn get_params(&self) -> &RobotParameters {
        &self.params
    }

    fn set_params(&mut self, params: RobotParameters) {
        self.params = params;
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

pub mod behaviors {
    use std::time::Duration;

    use crate::behaviors::BehaviorOutput;

    use super::*;

    pub fn nothing(_robot: &mut Box<dyn Robot>, _time: Duration) -> BehaviorOutput {
        (Control::default(), vec![])
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
