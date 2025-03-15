//! This module contains robot `search` behavior.

use std::{f32::consts::PI, time::Duration};

use emath::{Pos2, Vec2};

use crate::{
    shapes::{Line, Shape},
    LidarData,
};

use super::{
    cast_robot, debug, scaled_grid::ScaledGrid, shapes::Circle, utils::normalize_angle, BehaviorFn,
    CamData, Cone, Control, DebugSoup, DebugType, Message, MessageKind, Postbox, Robot, RobotId,
    RobotParameters,
};

pub const MENU: &[(&str, BehaviorFn)] = &[("search", search)];

/// The range of the lidar sensor at which the robot moves away from an object
const LIDAR_OBSTACLE_RANGE: f32 = 1.0;

const GRADIENT_WEIGHT: f32 = 2.0;
const LIDAR_WEIGHT: f32 = 0.3;
const FORWARD_BIAS: f32 = 0.05;

const ANGLE_THRESHOLD: f32 = PI / 4.0;

const SEARCH_GRID_SCALE: f32 = 0.20;

#[derive(Clone)]
pub struct SearchRobot {
    /// The id of the robot
    pub id: RobotId,

    /// Parameters of the robot
    pub params: RobotParameters,

    /// The position of the robot
    pub pos: Pos2,

    /// The velocity of the robot
    pub vel: f32,

    /// The angle of the robot
    pub angle: f32,

    /// The angular velocity of the robot
    pub avel: f32,

    /// The data from the camera. Angles and probability of objects.
    pub cam: CamData,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    pub postbox: Postbox,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,

    /// The time of the last search grid update
    pub last_search_grid_update: Duration,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    pub debug_soup: DebugSoup,
}

impl Default for SearchRobot {
    fn default() -> Self {
        Self {
            id: Default::default(),
            params: RobotParameters::turtlebot4(),
            pos: Default::default(),
            vel: Default::default(),
            angle: Default::default(),
            avel: Default::default(),
            cam: Default::default(),
            lidar: Default::default(),
            postbox: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Duration::default(),
            debug_soup: DebugSoup::new_inactive(),
        }
    }
}

impl Robot for SearchRobot {
    fn id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_world_size(&mut self, size: Vec2) {
        self.search_grid = ScaledGrid::new(size.x, size.y, SEARCH_GRID_SCALE);
    }

    fn params(&self) -> &RobotParameters {
        &self.params
    }

    fn set_params(&mut self, params: RobotParameters) {
        self.params = params;
    }

    fn get_postbox(&self) -> &super::Postbox {
        &self.postbox
    }

    fn get_postbox_mut(&mut self) -> &mut super::Postbox {
        &mut self.postbox
    }

    fn get_debug_soup(&self) -> &super::DebugSoup {
        &self.debug_soup
    }

    fn get_debug_soup_mut(&mut self) -> &mut DebugSoup {
        &mut self.debug_soup
    }

    fn input_pos(&mut self, pos: Pos2) {
        self.pos = pos;
    }

    fn input_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    fn input_cam(&mut self, cam: CamData) {
        self.cam = cam;
    }

    fn input_lidar(&mut self, lidar: LidarData) {
        self.lidar = lidar;
    }

    fn clone_box(&self) -> Box<dyn Robot> {
        Box::new(self.clone())
    }

    fn any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl SearchRobot {
    pub(crate) fn update_search_cone(&mut self, cone: &Cone, lidar: &LidarData, diff: f32) {
        for (point, cell) in self
            .search_grid
            .iter_cone(cone)
            .filter_map(|(p, c)| {
                let angle = (p - cone.center).angle();
                let distance = (p - cone.center).length();
                match distance <= lidar.interpolate(angle - cone.angle) {
                    true => Some((p, c.cloned())),
                    false => None,
                }
            })
            .collect::<Vec<_>>()
        {
            if let Some(mut cell) = cell {
                cell += diff;
                self.search_grid.set(point, cell);
            }
        }
    }

    pub(crate) fn update_search_line(&mut self, line: &Line, diff: f32) {
        let dir = (line.end - line.start).normalized();
        let step_size = self.search_grid.scale();
        let radius = step_size * 2.0;
        let mut distance = 0.0;
        while distance < self.params.cam_range - radius / 2.0 {
            let pos = line.start + dir * distance;

            // Nearness is in range [0, 1]
            let nearness = distance / self.params.cam_range;

            for (point, mut cell) in self
                .search_grid
                .iter_circle(&Circle {
                    center: pos,
                    radius,
                })
                .filter_map(|(p, c)| Some((p, *c?)))
                .collect::<Vec<_>>()
            {
                // We weight points closer to the robot more
                cell += 2.0 * (diff * nearness);

                self.search_grid.set(point, cell);
            }

            distance += step_size;
        }
    }

    pub(crate) fn update_search_grid(&mut self, time: Duration) {
        const HEAT_WIDTH: f32 = PI / 4.0;
        const CAM_MULTPLIER: f32 = 20.0;

        // How often to update the search grid (multiplied on all changes to the cells)
        const UPDATE_INTERVAL: f32 = 0.1;

        // Only update the search grid every UPDATE_INTERVAL seconds
        if (time - self.last_search_grid_update).as_secs_f32() < UPDATE_INTERVAL {
            return;
        }
        self.last_search_grid_update = time;

        // Cool down the search grid within the view of the camera
        {
            let cone = Cone {
                center: self.pos,
                radius: self.params.cam_range,
                angle: self.angle,
                fov: self.params.cam_fov,
            };
            let lidar = self.lidar.within_fov(self.params.cam_fov);
            let diff = -1.0 * UPDATE_INTERVAL;
            self.update_search_cone(&cone, &lidar, diff);
            self.postbox.post(Message {
                sender_id: self.id,
                kind: MessageKind::CamDiff { cone, lidar, diff },
            });
        }

        // Heat up the search grid in the direction of the search items
        // detected by the camera
        {
            let CamData(cam) = self.cam.clone();
            for cam_point in cam {
                let angle = self.angle + cam_point.angle;
                let dir = Vec2::angled(angle);
                let start = self.pos;
                let end = start + dir * self.params.cam_range;
                let line = Line { start, end };

                let diff = CAM_MULTPLIER * cam_point.propability * UPDATE_INTERVAL;

                self.update_search_line(&line, diff);
                self.post(MessageKind::ShapeDiff {
                    shape: Shape::Line(line),
                    diff,
                });
            }
        }

        // Read incoming messages and update the search grid accordingly
        for (msg_id, msg) in self
            .recv()
            .into_iter()
            .map(|(id, msg)| (id, msg.kind.clone()))
            .collect::<Vec<_>>()
        {
            match msg {
                MessageKind::ShapeDiff { shape, diff } => {
                    match shape {
                        Shape::Cone(_cone) => todo!(),
                        Shape::Line(line) => self.update_search_line(&line, diff),
                        _ => {}
                    }
                    self.postbox.set_processed(msg_id);
                }
                MessageKind::CamDiff { cone, lidar, diff } => {
                    self.update_search_cone(&cone, &lidar, diff);
                    self.postbox.set_processed(msg_id);
                }
                MessageKind::Debug(_) => {}
            }
        }
    }
}

impl SearchRobot {
    /// Calculate the gradient of the heat map around the robot
    fn gradient(&mut self) -> Vec2 {
        let mut gradient = Vec2::ZERO;
        {
            // Get the heat under the robot
            let mut robot_heat: f32 = 0.0;
            {
                let mut robot_points = vec![];
                for (pos, cell) in self.search_grid.iter_circle(
                    &(Circle {
                        center: self.pos,
                        radius: self.params.diameter * 2.0,
                    }),
                ) {
                    robot_heat += cell.unwrap_or(&0.0);
                    robot_points.push((pos, cell));
                }
                robot_heat /= robot_points.len() as f32;
                self.debug("Robot Heat", DebugType::Number(robot_heat));
            }

            let mut points = vec![];
            let mut total_weight: f32 = 0.0;
            let mut total_cells = 0;

            for (pos, cell) in self.search_grid.iter_circle(
                &(Circle {
                    center: self.pos,
                    radius: self.params.lidar_range,
                }),
            ) {
                let angle = (pos - self.pos).angle() - self.angle;
                let dist = (pos - self.pos).length();
                if dist > self.lidar.interpolate(angle) {
                    continue;
                }

                // Skip cells which are out of bounds
                let Some(cell) = cell else {
                    continue;
                };

                // The relative position of the cell to the robot
                let vec = pos - self.pos;

                // Weight is the difference between the cell and the robot heat
                let weight = cell - robot_heat;

                // Ignore cells that are colder than the robot as this ends up
                // repelling the robot leading to situations where the robot
                // gets stuck at the edge of the map
                if weight <= 0.0 {
                    continue;
                }

                // We want the weight to be stronger the closer we are to the robot
                let nearness = 1.0 - vec.length() / self.params.lidar_range;
                let weight = weight * nearness;

                points.push((pos, weight));

                gradient += vec.normalized() * weight;

                total_cells += 1;
                total_weight += weight;
            }

            if total_cells == 0 {
                total_cells = 1;
            }
            gradient /= total_cells as f32;

            gradient *= GRADIENT_WEIGHT;

            self.debug("Gradient Values", DebugType::NumberPoints(points));
            self.debug(
                "Average Weight",
                DebugType::Number(total_weight / total_cells as f32),
            );
            self.debug("Gradient", DebugType::Vector(gradient));
            self.debug("Gradient Length", DebugType::Number(gradient.length()));
        }

        gradient
    }

    /// Calculate the lidar contribution to the control
    fn lidar(&mut self) -> Vec2 {
        assert!(self.params.lidar_range >= LIDAR_OBSTACLE_RANGE);
        let mut lidar_contribution = Vec2::ZERO;
        {
            let mut total_weight: f32 = 0.0;
            let LidarData(points) = self.lidar.clone();
            for point in points {
                let distance = point.distance.clamp(0.0, LIDAR_OBSTACLE_RANGE);
                let weight = -(1.0 - distance / LIDAR_OBSTACLE_RANGE).powi(2);
                lidar_contribution += Vec2::angled(point.angle + self.angle) * weight;
            }
            if total_weight == 0.0 {
                total_weight = 1.0;
            }
            lidar_contribution /= total_weight.abs();

            lidar_contribution *= LIDAR_WEIGHT;

            self.debug("Lidar Contribution", DebugType::Vector(lidar_contribution));
        }
        lidar_contribution
    }

    pub fn control_towards(&self, target: Vec2) -> Control {
        let angle_error = normalize_angle(self.angle - target.angle());
        let t = (angle_error * angle_error / ANGLE_THRESHOLD).clamp(0.0, 1.0);
        let speed = 1.0 - t;
        let steer = t * normalize_angle(target.angle() - self.angle);

        Control { speed, steer }
    }
}

pub(crate) fn show_search_grid(robot: &mut SearchRobot) {
    robot.debug("Search Grid", DebugType::Grid(robot.search_grid.clone()));
}

/// Search for the object using a gradient on the heat map
pub fn search(robot: &mut Box<dyn Robot>, time: Duration) -> Control {
    let robot = cast_robot::<SearchRobot>(robot);

    {
        let lidar = &robot.lidar;
        let soup = &mut robot.debug_soup;
        let params = &robot.params;
        debug::common_routines::show_lidar(soup, lidar);
        debug::common_routines::show_cam_range(soup, lidar, params);
    }
    show_search_grid(robot);

    robot.update_search_grid(time);

    let forward_bias = Vec2::angled(robot.angle) * FORWARD_BIAS;
    robot.debug("Forward Bias", DebugType::Vector(forward_bias));

    let mut target = Vec2::ZERO;
    target += forward_bias;
    target += robot.gradient();
    target += robot.lidar();

    robot.debug("Target", DebugType::Vector(target));

    robot.postbox.clean();

    robot.control_towards(target)
}
