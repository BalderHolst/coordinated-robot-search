use std::{f32::consts::PI, time::Duration};

use crate::{
    behaviors::{BehaviorOutput, DebugItem},
    camera::CamData,
    cast_robot,
    debug_soup::DebugSoup,
    lidar::{LidarData, LidarPoint},
    messaging::{MessageKind, Postbox},
    Control, Map, Robot, RobotId, RobotPose,
};

use super::BehaviorFn;

pub const MENU: &[(&str, BehaviorFn)] = &[("avoid-closest", avoid_closest)];

#[derive(Clone, Default)]
pub struct AvoidObstaclesRobot {
    id: RobotId,
    postbox: Postbox,
    debug_soup: DebugSoup,
    lidar: LidarData,
}

impl Robot for AvoidObstaclesRobot {
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

    fn input_pose(&mut self, _poss: RobotPose) {}
    fn input_cam(&mut self, _cam: CamData) {}

    fn input_lidar(&mut self, lidar: LidarData) {
        self.lidar = lidar;
    }

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

/// Avoid obstacles by steering away from the closest point in front of the robot.
pub fn avoid_closest(robot: &mut Box<dyn Robot>, _time: Duration) -> BehaviorOutput {
    let robot = cast_robot::<AvoidObstaclesRobot>(robot);

    const MIN_DISTANCE: f32 = 3.0;
    const FOV: f32 = PI / 1.8;

    let mut steer = 0.0;
    let mut speed = 1.0;

    // Find the closest point in front of the robot
    let mut min_point = LidarPoint {
        angle: 0.0,
        distance: f32::INFINITY,
    };

    for point in robot.lidar.points() {
        let angle = point.angle;
        if (angle.abs() < FOV || angle.abs() > 2.0 * PI - FOV)
            && point.distance < min_point.distance
        {
            min_point = point.clone();
        }
    }

    if robot.debug_enabled() {
        robot.debug(
            "",
            "Lidar",
            DebugItem::RobotRays(
                robot
                    .lidar
                    .points()
                    .map(|p| (p.angle, p.distance))
                    .collect::<Vec<_>>(),
            ),
        );
    }

    // If the closest point is too close, steer away from it
    if min_point.distance < MIN_DISTANCE {
        let how_close = (MIN_DISTANCE - min_point.distance) / MIN_DISTANCE;
        let how_close = how_close.powi(2);
        steer = how_close * (-min_point.angle.signum());

        robot.post(MessageKind::Debug(format!(
            "Close obstacle at {:.2}",
            min_point.angle
        )));

        speed *= 1.0 - how_close;
    }

    robot.postbox.clean();
    let msgs = robot.postbox.empty();

    (Control { speed, steer }, msgs)
}
