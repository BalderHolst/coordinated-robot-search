#![allow(dead_code)]

use std::{
    collections::{HashMap, HashSet},
    f32::consts::PI,
    ops::Range,
    time::Instant,
};

use debug::DebugType;
pub use emath::{Pos2, Vec2};
use scaled_grid::ScaledGrid;
use shapes::{Circle, Cone, Line, Shape};
use utils::normalize_angle;

pub mod behaviors;
pub mod debug;
pub mod grid;
pub mod scaled_grid;
pub mod shapes;
mod utils;

/// A unique identifier for a robot
#[derive(Debug, Clone, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct RobotId(u32);

impl Copy for RobotId {}

impl RobotId {
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    pub fn as_u32(&self) -> u32 {
        self.0
    }
}

/// Kinds of messages that can be sent between robots
#[derive(Debug, Clone)]
pub enum MessageKind {
    ShapeDiff {
        shape: Shape,
        diff: f32,
    },
    CamDiff {
        cone: Cone,
        lidar: LidarData,
        diff: f32,
    },
    Debug(String),
}

/// A message sent between robots
#[derive(Debug, Clone)]
pub struct Message {
    /// The id of the robot that sent the message
    pub from: RobotId,

    /// The kind of message
    pub kind: MessageKind,
}

/// A point detected by the camera
#[derive(Debug, Clone)]
pub struct CamPoint {
    /// The angle of the point relative to the robot
    pub angle: f32,

    /// The probability of the search object being at this point
    pub propability: f32,
}

/// Data from the camera
#[derive(Debug, Clone, Default)]
pub struct CamData(pub Vec<CamPoint>);

/// A point detected by the lidar
#[derive(Debug, Clone, Default)]
pub struct LidarPoint {
    /// The angle of the point relative to the robot
    pub angle: f32,

    /// The distance to the object
    pub distance: f32,
}

/// Data from the lidar
#[derive(Debug, Clone, Default)]
pub struct LidarData(Vec<LidarPoint>);

impl LidarData {
    pub fn new(mut points: Vec<LidarPoint>) -> Self {
        points
            .iter_mut()
            .for_each(|p| p.angle = normalize_angle(p.angle));
        points.sort_by(|a, b| a.angle.total_cmp(&b.angle));
        Self(points)
    }

    pub fn points(&self) -> impl Iterator<Item = &LidarPoint> {
        self.0.iter()
    }

    pub fn into_points(self) -> Vec<LidarPoint> {
        self.0
    }

    pub fn within_fov(&self, fov: f32) -> Self {
        let range = -fov / 2.0..fov / 2.0;
        let points: Vec<_> = self
            .points()
            .filter(|&p| range.contains(&p.angle))
            .cloned()
            .collect();
        Self(points)
    }

    /// Interpolates the distance to an object at a given continuous angle
    pub fn interpolate(&self, angle: f32) -> f32 {
        let Self(points) = &self;

        if points.is_empty() {
            return 0.0;
        }

        if points.len() == 1 {
            return points[0].distance;
        }

        let angle = normalize_angle(angle);

        let mut i: usize = 0;
        for point in points {
            if point.angle > angle {
                break;
            }
            i += 1;
        }

        let a = points.get(i.saturating_sub(1));
        let b = points.get(i);

        match (a, b) {
            (None, None) => 0.0,
            (None, Some(p)) | (Some(p), None) => p.distance,
            (Some(a), Some(b)) if a.distance == b.distance => a.distance,
            (Some(a), Some(b)) => {
                let t = (angle - a.angle) / (b.angle - a.angle);
                a.distance + t * (b.distance - a.distance)
            }
        }
    }
}

#[test]
fn test_lidar_interpolate() {
    let points = vec![
        LidarPoint {
            angle: -PI / 2.0,
            distance: 1.0,
        },
        LidarPoint {
            angle: 0.0,
            distance: 2.0,
        },
        LidarPoint {
            angle: PI / 2.0,
            distance: 3.0,
        },
    ];
    let lidar = LidarData::new(points);

    assert_eq!(lidar.interpolate(-PI), 1.0);
    assert_eq!(lidar.interpolate(0.0), 2.0);
    assert_eq!(lidar.interpolate(PI / 4.0), 2.5);
}

/// Control signal to be sent to the robot
#[derive(Debug, Clone, Default)]
pub struct Control {
    pub speed: f32,
    pub steer: f32,
}

// TODO: Factor out parameters
#[derive(Clone)]
pub struct Robot {
    /// The id of the robot
    pub id: RobotId,

    /// The position of the robot
    pub pos: Pos2,

    /// The velocity of the robot
    pub vel: f32,

    /// The angle of the robot
    pub angle: f32,

    /// The angular velocity of the robot
    pub avel: f32,

    /// Size of the robot
    pub diameter: f32,

    /// The data from the camera. Angles and probability of objects.
    pub cam: CamData,

    /// Range (min and max distance) of camera object detection (in meters)
    pub cam_range: Range<f32>,

    /// Field of view of the camera
    pub cam_fov: f32,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    /// Range of the lidar sensor (in meters)
    pub lidar_range: f32,

    /// The messages from the other robots since the last call.
    pub incoming_msg: Vec<Message>,

    /// The indices of the messages that have been processed.
    pub processed_msgs: HashSet<usize>,

    /// The messages to be sent to the other robots.
    pub outgoing_msg: Vec<Message>,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,

    /// The time of the last search grid update
    pub last_search_grid_update: Instant,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    pub debug_soup: Option<HashMap<String, DebugType>>,
}

impl Default for Robot {
    fn default() -> Self {
        Self {
            id: Default::default(),
            pos: Default::default(),
            vel: Default::default(),
            angle: Default::default(),
            avel: Default::default(),
            diameter: 0.5,
            cam: Default::default(),
            cam_range: 0.8..3.0,
            cam_fov: PI / 2.0,
            lidar: Default::default(),
            lidar_range: 5.0,
            incoming_msg: Default::default(),
            processed_msgs: Default::default(),
            outgoing_msg: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Instant::now(),
            debug_soup: None,
        }
    }
}

impl Robot {
    /// Get the messages from the other robots since the last call
    pub(crate) fn recv(&mut self) -> impl Iterator<Item = (usize, &Message)> {
        self.incoming_msg
            .iter()
            .enumerate()
            .filter(|(i, msg)| !self.processed_msgs.contains(i) && msg.from != self.id)
    }

    /// Mark a message as processed
    pub(crate) fn set_processed(&mut self, idx: usize) {
        self.processed_msgs.insert(idx);
    }

    /// Clear the processed messages from the incoming messages list
    pub(crate) fn clear_processed(&mut self) {
        self.incoming_msg = self
            .incoming_msg
            .iter()
            .cloned()
            .enumerate()
            .filter(|(i, _)| !self.processed_msgs.contains(i))
            .map(|(_, msg)| msg)
            .collect();
        self.processed_msgs.clear();
    }

    /// Send a message to the other robots.
    pub(crate) fn post(&mut self, kind: MessageKind) {
        self.outgoing_msg.push(Message {
            from: self.id,
            kind,
        });
    }

    pub(crate) fn update_search_cone(&mut self, cone: &Cone, lidar: &LidarData, diff: f32) {
        for (point, cell) in self
            .search_grid
            .iter_cone(cone)
            .filter_map(|(p, c)| {
                let angle = (p - cone.center).angle();
                let distance = (p - cone.center).length();
                match distance <= lidar.interpolate(angle - self.angle) {
                    true => Some((p, c.cloned())),
                    false => None,
                }
            })
            .collect::<Vec<_>>()
        {
            if let Some(mut cell) = cell {
                cell -= diff;
                self.search_grid.set(point, cell);
            }
        }
    }

    pub(crate) fn update_search_line(&mut self, line: &Line, diff: f32) {
        let dir = (line.end - line.start).normalized();
        let step_size = self.search_grid.scale();
        let radius = step_size * 2.0;
        let start_distance = self.cam_range.start + radius / 2.0;
        let mut distance = start_distance;
        while distance < self.cam_range.end - radius / 2.0 {
            let pos = line.start + dir * distance;

            // Nearness is in range [0, 1]
            let nearness = (distance - start_distance) / (self.cam_range.end - start_distance);

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

    pub(crate) fn update_search_grid(&mut self, time: Instant) {
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
                radius: self.cam_range.clone(),
                angle: self.angle,
                fov: self.cam_fov,
            };
            let lidar = self.lidar.within_fov(self.cam_fov);
            let diff = 1.0 * UPDATE_INTERVAL;
            self.update_search_cone(&cone, &lidar, diff);
            self.post(MessageKind::CamDiff { cone, lidar, diff });
        }

        // Heat up the search grid in the direction of the search items
        // detected by the camera
        {
            let CamData(cam) = self.cam.clone();
            for cam_point in cam {
                let angle = self.angle + cam_point.angle;
                let dir = Vec2::angled(angle);
                let start = self.pos + dir * self.cam_range.start;
                let end = self.pos + dir * self.cam_range.end;
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
                    self.set_processed(msg_id);
                }
                MessageKind::CamDiff { cone, lidar, diff } => {
                    self.update_search_cone(&cone, &lidar, diff);
                    self.set_processed(msg_id);
                }
                MessageKind::Debug(_) => {}
            }
        }
    }

    pub(crate) fn debug<S: ToString>(&mut self, name: S, debug_type: DebugType) {
        if let Some(m) = &mut self.debug_soup {
            m.insert(name.to_string(), debug_type);
        }
    }
}
