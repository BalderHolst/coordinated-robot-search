#![allow(dead_code)]

use std::{f32::consts::PI, fmt::Display, ops::Range, time::Instant};

pub use emath::{Pos2, Vec2};
use scaled_grid::ScaledGrid;

pub mod behaviors;
pub mod grid;
pub mod scaled_grid;

fn normalize_angle(angle: f32) -> f32 {
    let mut angle = angle;
    while angle < -std::f32::consts::PI {
        angle += 2.0 * std::f32::consts::PI;
    }
    while angle > std::f32::consts::PI {
        angle -= 2.0 * std::f32::consts::PI;
    }
    angle
}

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

#[derive(Debug, Clone)]
pub enum MessageKind {
    Cone {
        center: Pos2,
        radius: Range<f32>,
        angle: f32,
        fov: f32,
    },
    Line {
        start: Pos2,
        end: Pos2,
    },
    String(String),
    Debug(String),
}

impl Display for MessageKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MessageKind::Line { start, end } => write!(f, "Line: {} -> {}", start, end),
            MessageKind::String(s) => write!(f, "{}", s),
            MessageKind::Debug(s) => write!(f, "DEBUG: {}", s),
            MessageKind::Cone {
                center,
                radius,
                angle,
                fov,
            } => write!(
                f,
                "Cone: center: {}, radius: {:?}, angle: {}, fov: {}",
                center, radius, angle, fov
            ),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Message {
    pub from: RobotId,
    pub kind: MessageKind,
}

#[derive(Debug, Clone)]
pub struct CamPoint {
    pub angle: f32,
    pub propability: f32,
}

#[derive(Debug, Clone, Default)]
pub struct CamData(pub Vec<CamPoint>);

#[derive(Debug, Clone, Default)]
pub struct LidarPoint {
    pub angle: f32,
    pub distance: f32,
}

#[derive(Debug, Clone, Default)]
pub struct LidarData(pub Vec<LidarPoint>);

#[derive(Debug, Clone, Default)]
pub struct Control {
    pub speed: f32,
    pub steer: f32,
}

#[derive(Debug, Clone)]
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
    pub cam_range: (f32, f32),

    /// Field of view of the camera
    pub cam_fov: f32,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    /// Range of the lidar sensor (in meters)
    pub lidar_range: f32,

    /// The messages from the other robots since the last call.
    pub incomming_msg: Vec<Message>,

    /// The messages to be sent to the other robots.
    pub outgoing_msg: Vec<Message>,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,
    pub last_search_grid_update: Instant,
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
            cam_range: (0.8, 3.0),
            cam_fov: PI / 2.0,
            lidar: Default::default(),
            lidar_range: 5.0,
            incomming_msg: Default::default(),
            outgoing_msg: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Instant::now(),
        }
    }
}

impl Robot {
    /// Get the messages from the other robots since the last call.
    pub(crate) fn recv(&self) -> &Vec<Message> {
        &self.incomming_msg
    }

    /// Send a message to the other robots.
    pub(crate) fn post(&mut self, msg: MessageKind) {
        self.outgoing_msg.push(Message {
            from: self.id,
            kind: msg,
        });
    }

    pub(crate) fn update_search_grid(&mut self, time: Instant) {
        const MAX_HEAT: f32 = 100.0;
        const MIN_HEAT: f32 = -100.0;

        const HEAT_WIDTH: f32 = PI / 4.0;

        // How often to update the search grid (multiplied on all changes to the cells)
        const UPDATE_INTERVAL: f32 = 0.1;

        if (time - self.last_search_grid_update).as_secs_f32() < UPDATE_INTERVAL {
            return;
        }
        self.last_search_grid_update = time;

        let CamData(cam) = self.cam.clone();
        let (min_distance, max_distance) = self.cam_range;

        let cone = self
            .search_grid
            .iter_circle(self.pos, self.cam_range.1)
            .filter(|(point, _cell)| {
                let offset = *point - self.pos;
                if offset.length() < min_distance {
                    return false;
                }
                let angle = offset.angle() - self.angle;
                let angle = normalize_angle(angle);
                angle.abs() < self.cam_fov / 2.0
            })
            .collect::<Vec<_>>();

        for (point, mut cell) in cone {
            cell -= 0.1 * UPDATE_INTERVAL;
            self.search_grid.set(point, cell);
        }

        let step_size = self.search_grid.scale();
        for cam_point in cam {
            let angle = self.angle + cam_point.angle;
            let dir = Vec2::angled(angle);

            let r = step_size * 2.0;
            let mut distance = min_distance + r / 2.0;
            while distance < max_distance - r / 2.0 {
                let pos = self.pos + dir * distance;

                for (point, mut cell) in self.search_grid.iter_circle(pos, r).collect::<Vec<_>>() {
                    cell += cam_point.propability * UPDATE_INTERVAL;
                    cell = cell.clamp(MIN_HEAT, MAX_HEAT);
                    self.search_grid.set(point, cell);
                }

                distance += step_size;
            }
        }
    }
}
