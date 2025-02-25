#![allow(dead_code)]

use std::{
    collections::{HashMap, HashSet},
    f32::consts::PI,
    fmt::Display,
    ops::Range,
    time::Instant,
};

use debug::DebugType;
pub use emath::{Pos2, Vec2};
use scaled_grid::ScaledGrid;
use shapes::{Cone, Line};

pub mod behaviors;
pub mod debug;
pub mod grid;
pub mod scaled_grid;
pub mod shapes;

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
    Cone { cone: Cone, diff: f32 },
    Line { line: Line, diff: f32 },
    String(String),
    Debug(String),
}

impl Display for MessageKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MessageKind::Line { line, .. } => write!(f, "Line: {} -> {}", line.start, line.end),
            MessageKind::String(s) => write!(f, "{}", s),
            MessageKind::Debug(s) => write!(f, "DEBUG: {}", s),
            MessageKind::Cone {
                cone:
                    Cone {
                        center,
                        radius,
                        angle,
                        fov,
                    },
                ..
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
pub struct RecvMessage {
    msg: Message,
    processed: bool,
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
    pub incomming_msg: Vec<Message>,
    pub processed_msgs: HashSet<usize>,

    /// The messages to be sent to the other robots.
    pub outgoing_msg: Vec<Message>,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,
    pub last_search_grid_update: Instant,

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
            incomming_msg: Default::default(),
            processed_msgs: Default::default(),
            outgoing_msg: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Instant::now(),
            debug_soup: None,
        }
    }
}

impl Robot {
    /// Get the messages from the other robots since the last call.
    pub(crate) fn recv(&mut self) -> impl Iterator<Item = (usize, &Message)> {
        self.incomming_msg
            .iter()
            .enumerate()
            .filter(|(i, msg)| !self.processed_msgs.contains(i) && msg.from != self.id)
    }

    /// Mark a message as processed.
    pub(crate) fn set_processed(&mut self, idx: usize) {
        self.processed_msgs.insert(idx);
    }

    pub(crate) fn clear_processed(&mut self) {
        self.incomming_msg = self
            .incomming_msg
            .iter()
            .cloned()
            .enumerate()
            .filter(|(i, _)| !self.processed_msgs.contains(i))
            .map(|(_, msg)| msg)
            .collect();
        self.processed_msgs.clear();
    }

    /// Send a message to the other robots.
    pub(crate) fn post(&mut self, msg: MessageKind) {
        self.outgoing_msg.push(Message {
            from: self.id,
            kind: msg,
        });
    }

    pub(crate) fn update_search_cone(&mut self, cone: &Cone, diff: f32) {
        let cone_iter = self
            .search_grid
            .iter_circle(cone.center, cone.radius.end)
            .filter(|(point, _cell)| {
                let offset = *point - cone.center;
                if offset.length() < cone.radius.start {
                    return false;
                }
                let angle = offset.angle() - cone.angle;
                let angle = normalize_angle(angle);
                angle.abs() < cone.fov / 2.0
            })
            .collect::<Vec<_>>();

        for (point, cell) in cone_iter {
            if let Some(mut cell) = cell {
                cell -= diff;
                self.search_grid.set(point, cell);
            }
        }
    }

    pub(crate) fn update_search_line(&mut self, line: &Line, diff: f32) {
        let dir = (line.end - line.start).normalized();
        let step_size = self.search_grid.scale();
        let r = step_size * 2.0;
        let start_distance = self.cam_range.start + r / 2.0;
        let mut distance = start_distance;
        while distance < self.cam_range.end - r / 2.0 {
            let pos = line.start + dir * distance;

            // Nearness is in range [0, 1]
            let nearness = (distance - start_distance) / (self.cam_range.end - start_distance);

            for (point, mut cell) in self
                .search_grid
                .iter_circle(pos, r)
                .filter_map(|(p, c)| Some((p, c?)))
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

        if (time - self.last_search_grid_update).as_secs_f32() < UPDATE_INTERVAL {
            return;
        }
        self.last_search_grid_update = time;

        let CamData(cam) = self.cam.clone();

        let cone = Cone {
            center: self.pos,
            radius: self.cam_range.clone(),
            angle: self.angle,
            fov: self.cam_fov,
        };

        {
            let diff = 1.0 * UPDATE_INTERVAL;
            self.update_search_cone(&cone, diff);
            self.post(MessageKind::Cone { cone, diff });
        }

        for cam_point in cam {
            let angle = self.angle + cam_point.angle;
            let dir = Vec2::angled(angle);
            let start = self.pos + dir * self.cam_range.start;
            let end = self.pos + dir * self.cam_range.end;
            let line = Line { start, end };

            let diff = CAM_MULTPLIER * cam_point.propability * UPDATE_INTERVAL;
            self.update_search_line(&line, diff);
            self.post(MessageKind::Line { line, diff });
        }

        // Read incoming messages
        for (msg_id, msg) in self
            .recv()
            .map(|(id, msg)| (id, msg.kind.clone()))
            .collect::<Vec<_>>()
        {
            match msg {
                MessageKind::Cone { cone, diff } => {
                    self.update_search_cone(&cone, diff);
                    self.set_processed(msg_id);
                }
                MessageKind::Line { line, diff } => {
                    self.update_search_line(&line, diff);
                    self.set_processed(msg_id);
                }
                _ => {}
            }
        }
    }

    pub(crate) fn debug<S: ToString>(&mut self, name: S, debug_type: DebugType) {
        if let Some(m) = &mut self.debug_soup {
            m.insert(name.to_string(), debug_type);
        }
    }
}
