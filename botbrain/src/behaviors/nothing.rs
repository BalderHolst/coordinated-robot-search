use {
    super::{CamData, Control, DebugSoup, LidarData, Postbox, Robot, RobotId, RobotParameters},
    emath::{Pos2, Vec2},
};

#[derive(Clone, Default)]
pub struct NothingRobot {
    id: RobotId,
    params: RobotParameters,
    postbox: Postbox,
    debug_soup: DebugSoup,
}

impl Robot for NothingRobot {
    fn id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn params(&self) -> &RobotParameters {
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

    fn input_pos(&mut self, _pos: Pos2) {}
    fn input_angle(&mut self, _angle: f32) {}
    fn input_cam(&mut self, _cam: CamData) {}
    fn input_lidar(&mut self, _lidar: LidarData) {}

    fn clone_box(&self) -> Box<dyn Robot> {
        Box::new(self.clone())
    }

    fn behavior(&mut self, _time: std::time::Instant) -> Control {
        Control {
            speed: 0.0,
            steer: 0.0,
        }
    }
}
