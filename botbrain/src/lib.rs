#![warn(missing_docs)]

//! This crate defines the core implementation of robot search behaviors.
//!
//! To run a select a behavior you need to select a robot and a behavior function to run the robot on.
//!
//! # Example
//! ```
//! # let get_pose = || botbrain::RobotPose::default();
//! # let get_cam = || botbrain::camera::CamData::default();
//! # let get_lidar = || botbrain::lidar::LidarData::default();
//! # let get_time = || std::time::Duration::default();
//! # let recv_messages = || vec![];
//! # let send_messages = |msgs: Vec<botbrain::messaging::Message>| {};
//! # let do_control = |control: botbrain::Control| {};
//! use botbrain::{
//!     behaviors::{Behavior, RobotKind},
//!     Map, RobotId,
//! };
//!
//! // Choose a robot
//! let robot_kind = RobotKind::Dumb;
//!
//! // Select a behavior function
//! let behavior = robot_kind.get_behavior_fn("circle");
//!
//! // Create a new robot
//! let mut robot = robot_kind.create_fn()();
//!
//! // Initialize the robot
//! robot.set_id(RobotId::new(0));
//! robot.set_map(Map::empty()); // <- Make sure to set an actual map
//!
//! // Control loop
//! loop {
//!
//!     // Get sensor data (implemented by user)
//!     let pose = get_pose();
//!     let cam = get_cam();
//!     let lidar = get_lidar();
//!
//!     // Get incomming messages(implemented by user)
//!     let input_messages = recv_messages();
//!
//!     // Input the sensor data and messages into the robot
//!     robot.input_pose(pose);
//!     robot.input_cam(cam);
//!     robot.input_lidar(lidar);
//!     robot.input_msgs(input_messages);
//!
//!     // Run the behavior function
//!     let time = get_time();
//!     let (control, output_messages) = behavior(&mut robot, time);
//!
//!     // Send the control signal to the robot motor controller (implemented by user)
//!     do_control(control);
//!
//!     // Send messages to other robots (implemented by user)
//!     send_messages(output_messages);
//!
//!     # break;
//! }
//!
//! ```
//!

use std::any::Any;

use camera::CamData;
use debug_soup::{DebugItem, DebugSoup};
pub use emath::{Pos2, Vec2};
use lidar::LidarData;
use messaging::{Message, MessageKind, Postbox};
use scaled_grid::ScaledGrid;
use serde::{Deserialize, Serialize};

#[cfg(feature = "rl")]
pub use burn;

pub mod behaviors;
pub mod camera;
pub mod debug_soup;
pub mod grid;
pub mod lidar;
pub mod messaging;
pub mod params;
pub mod scaled_grid;
pub mod shapes;
pub mod utils;

/// A trait representing a robot. This is the interface that all robots must implement.
///
/// Methods starting with `set_` should be called after creating the robot to set the
/// internal parameters of the robot.
///
/// Methods starting with `input_` are used to input data into the robot and should be
/// called before calling a behavior function on the robot.
pub trait Robot: Send {
    /// Get the id of the robot
    fn get_id(&self) -> &RobotId;

    /// Set the id of the robot
    fn set_id(&mut self, id: RobotId);

    /// The map the robot is operating in
    fn set_map(&mut self, map: Map);

    /// Get the robot postbox
    fn get_postbox(&self) -> &Postbox;

    /// Get the robot postbox mutably
    fn get_postbox_mut(&mut self) -> &mut Postbox;

    /// Post a message to the other robots
    fn post(&mut self, kind: MessageKind) {
        let sender_id = *self.get_id();
        self.get_postbox_mut().post(Message { sender_id, kind });
    }

    /// Receive messages from the other robots
    fn recv(&mut self) -> Vec<(usize, &Message)> {
        let id = *self.get_id();
        self.get_postbox_mut()
            .recv()
            .filter(move |(_, msg)| msg.sender_id != id)
            .collect()
    }

    /// Get the debug soup
    fn get_debug_soup(&self) -> &DebugSoup;

    /// Get the debug soup mutably
    fn get_debug_soup_mut(&mut self) -> &mut DebugSoup;

    /// Add a debug object to the debug soup
    fn debug(&mut self, category: &'static str, name: &'static str, debug_type: DebugItem) {
        self.get_debug_soup_mut().add(category, name, debug_type);
    }

    /// Check if the debug soup is active
    fn debug_enabled(&self) -> bool {
        self.get_debug_soup().is_active()
    }

    /// Input the angle of the robot
    fn input_pose(&mut self, pose: RobotPose);

    /// Input data from the camera
    fn input_cam(&mut self, cam: CamData);

    /// Input data from the lidar
    fn input_lidar(&mut self, lidar: LidarData);

    /// Input a messages from other robots
    fn input_msgs(&mut self, msgs: Vec<Message>) {
        let id = *self.get_id();
        self.get_postbox_mut()
            .deposit(msgs.into_iter().filter(|msg| msg.sender_id != id));
    }

    /// Clone a dynamic instance of the [Robot] trait
    fn clone_box(&self) -> Box<dyn Robot>;

    /// Get the robot as [Any]. This is used for downcasting the robot to a specific type.
    fn any(&self) -> &dyn Any;

    /// Get the robot as mutable [Any]. This is used for downcasting the robot to a specific mutable type.
    fn any_mut(&mut self) -> &mut dyn Any;
}

/// A unique identifier for a robot
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct RobotId(u32);

impl Copy for RobotId {}

impl RobotId {
    /// Create a new [RobotId]
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the underlying id as a [u32]
    pub fn as_u32(&self) -> u32 {
        self.0
    }
}

/// Control signal to be sent to the robot.
/// Ranges for speed and steering values are set in the [params] module.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Control {
    /// Desired speed of the robot
    pub speed: f32,
    /// Desired steering of the robot
    pub steer: f32,
}

/// A cell within the robot world map
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum MapCell {
    /// The cell is free
    #[default]
    Free,
    /// The cell is occupied by an object
    Obstacle,
}

/// Robot world map
pub type Map = ScaledGrid<MapCell>;

/// The pose of a robot
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RobotPose {
    /// Position of the robot in meters
    pub pos: Pos2,
    /// Angle of the robot in radians
    pub angle: f32,
}

/// A reference to an arbitrary robot
pub type RobotRef = Box<dyn Robot>;

/// Cast a robot to a specific type
fn cast_robot<T: Robot + 'static>(robot: &mut RobotRef) -> &mut T {
    robot
        .any_mut()
        .downcast_mut()
        .expect("We should always be downcasting to the correct robot")
}
