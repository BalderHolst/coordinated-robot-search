use std::{f32::consts::PI, time::Instant};

use emath::{Pos2, Vec2};

use crate::{LidarPoint, MessageKind};

use super::{CamData, Control, DebugSoup, LidarData, Postbox, Robot, RobotId, RobotParameters};

#[derive(Clone, Default)]
pub struct AvoidObstaclesRobot {
    id: RobotId,
    params: RobotParameters,
    postbox: Postbox,
    debug_soup: DebugSoup,
    lidar: LidarData,
}

impl Robot for AvoidObstaclesRobot {
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

    fn input_lidar(&mut self, lidar: LidarData) {
        self.lidar = lidar;
    }

    fn clone_box(&self) -> Box<dyn Robot> {
        Box::new(self.clone())
    }

    fn behavior(&mut self, time: std::time::Instant) -> Control {
        self.avoid_obstacles(time)
    }
}

/// Avoid obstacles by steering away from the closest point in front of the robot.
impl AvoidObstaclesRobot {
    pub fn avoid_obstacles(&mut self, _time: Instant) -> Control {
        const MIN_DISTANCE: f32 = 3.0;
        const FOV: f32 = PI / 1.8;

        let mut steer = 0.0;
        let mut speed = 1.0;

        // Find the closest point in front of the robot
        let mut min_point = LidarPoint {
            angle: 0.0,
            distance: f32::INFINITY,
        };

        for point in self.lidar.points() {
            let angle = point.angle;
            if (angle.abs() < FOV || angle.abs() > 2.0 * PI - FOV)
                && point.distance < min_point.distance
            {
                min_point = point.clone();
            }
        }

        // If the closest point is too close, steer away from it
        if min_point.distance < MIN_DISTANCE {
            let how_close = (MIN_DISTANCE - min_point.distance) / MIN_DISTANCE;
            let how_close = how_close.powi(2);
            steer = how_close * (-min_point.angle.signum());

            self.post(MessageKind::Debug(format!(
                "Close obstacle at {:.2}",
                min_point.angle
            )));

            speed *= 1.0 - how_close;
        }

        self.postbox.clean();
        Control { speed, steer }
    }
}
