#![allow(dead_code)]

use std::{collections::HashSet, f32::consts::PI, time::Instant};

use debug::{DebugSoup, DebugType};
pub use emath::{Pos2, Vec2};
use scaled_grid::ScaledGrid;
use serde::{Deserialize, Serialize};
use shapes::{Cone, Shape};
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
#[derive(Debug, Clone, Serialize, Deserialize)]
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

#[cfg(feature = "serde-bin")]
const ENDIANNESS: serde_binary::binary_stream::Endian = serde_binary::binary_stream::Endian::Little;

#[cfg(feature = "serde-bin")]
impl TryFrom<Vec<u8>> for MessageKind {
    type Error = serde_binary::Error;
    fn try_from(value: Vec<u8>) -> Result<Self, Self::Error> {
        serde_binary::from_slice(&value, ENDIANNESS)
    }
}

#[cfg(feature = "serde-bin")]
impl TryFrom<MessageKind> for Vec<u8> {
    type Error = serde_binary::Error;
    fn try_from(value: MessageKind) -> Result<Self, Self::Error> {
        serde_binary::to_vec(&value, ENDIANNESS)
    }
}

/// A message sent between robots
#[derive(Debug, Clone)]
pub struct Message {
    /// The id of the robot that sent the message
    pub sender_id: RobotId,

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
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LidarPoint {
    /// The angle of the point relative to the robot
    pub angle: f32,

    /// The distance to the object
    pub distance: f32,
}

/// Data from the lidar. Points have angles within the range [-PI, PI].
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
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

/// Parameters which are constant for a type of robot
#[derive(Clone)]
pub struct RobotParameters {
    /// Size of the robot
    pub diameter: f32,

    /// Range (min and max distance) of camera object detection (in meters)
    pub cam_range: f32,

    /// Field of view of the camera
    pub cam_fov: f32,

    /// Range of the lidar sensor (in meters)
    pub lidar_range: f32,
}

impl RobotParameters {
    fn turtlebot4() -> Self {
        Self {
            diameter: 0.5,
            cam_range: 3.0,
            cam_fov: PI / 2.0,
            lidar_range: 5.0,
        }
    }
}

impl Default for RobotParameters {
    fn default() -> Self {
        Self::turtlebot4()
    }
}

/// Manages incoming and outgoing messages
#[derive(Clone, Default)]
pub struct Postbox {
    incoming_msg: Vec<Message>,
    processed_msgs: HashSet<usize>,
    outgoing_msg: Vec<Message>,
}

impl Postbox {
    pub fn new() -> Self {
        Self::default()
    }

    /// Get the messages from the other robots
    pub fn recv(&mut self) -> impl Iterator<Item = (usize, &Message)> {
        self.incoming_msg
            .iter()
            .enumerate()
            .filter(|(i, _msg)| !self.processed_msgs.contains(i))
    }

    /// Mark a message as processed
    pub fn set_processed(&mut self, idx: usize) {
        self.processed_msgs.insert(idx);
    }

    /// Clear the processed messages from the incoming messages list
    pub fn clean(&mut self) {
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
    pub fn post(&mut self, msg: Message) {
        self.outgoing_msg.push(msg);
    }

    /// Deposit messages into the postbox to be received by the robot
    pub fn deposit(&mut self, msgs: impl Iterator<Item = Message>) {
        self.incoming_msg.extend(msgs);
    }

    /// Gather the messages ready to be sent to the other robots
    pub fn empty(&mut self) -> Vec<Message> {
        std::mem::take(&mut self.outgoing_msg)
    }
}

pub trait Robot: Send + Sync {
    fn id(&self) -> &RobotId;

    fn set_id(&mut self, id: RobotId);
    fn set_world_size(&mut self, size: Vec2);

    fn params(&self) -> &RobotParameters;
    fn set_params(&mut self, params: RobotParameters);

    fn get_postbox(&self) -> &Postbox;
    fn get_postbox_mut(&mut self) -> &mut Postbox;

    fn post(&mut self, kind: MessageKind) {
        let sender_id = *self.id();
        self.get_postbox_mut().post(Message { sender_id, kind });
    }

    fn recv(&mut self) -> Vec<(usize, &Message)> {
        let id = *self.id();
        self.get_postbox_mut()
            .recv()
            .filter(move |(_, msg)| msg.sender_id != id)
            .collect()
    }

    fn get_debug_soup(&self) -> &DebugSoup;
    fn get_debug_soup_mut(&mut self) -> &mut DebugSoup;

    fn debug(&mut self, name: &'static str, debug_type: DebugType) {
        self.get_debug_soup_mut().add(name, debug_type);
    }

    fn debug_enabled(&self) -> bool {
        self.get_debug_soup().is_active()
    }

    fn input_pos(&mut self, pos: Pos2);
    fn input_angle(&mut self, angle: f32);

    fn input_cam(&mut self, cam: CamData);
    fn input_lidar(&mut self, lidar: LidarData);

    fn clone_box(&self) -> Box<dyn Robot>;

    fn behavior(&mut self, time: Instant) -> Control;
}
