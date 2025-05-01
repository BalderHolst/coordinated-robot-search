use std::any::Any;

#[cfg(feature = "rl")]
pub use burn;
use camera::CamData;
use debug_soup::{DebugSoup, DebugType};
pub use emath::{Pos2, Vec2};
use lidar::LidarData;
use messaging::{Message, MessageKind, Postbox};
use scaled_grid::ScaledGrid;
use serde::{Deserialize, Serialize};

pub mod behaviors;
pub mod camera;
pub mod debug_soup;
pub mod grid;
pub mod lidar;
pub mod messaging;
pub mod params;
pub mod scaled_grid;
pub mod shapes;
mod utils;

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

    // The world the robot is operating in
    fn set_world(&mut self, world: Map);

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
    fn debug(&mut self, category: &'static str, name: &'static str, debug_type: DebugType) {
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
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    pub fn as_u32(&self) -> u32 {
        self.0
    }
}

/// Control signal to be sent to the robot
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Control {
    pub speed: f32,
    pub steer: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum MapCell {
    /// The cell is free
    #[default]
    Free,
    /// The cell is occupied by an object
    Obstacle,
}

pub type Map = ScaledGrid<MapCell>;

/// The pose of a robot
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RobotPose {
    pub pos: Pos2,
    pub angle: f32,
}

pub type RobotRef = Box<dyn Robot>;

/// Cast a robot to a specific type
fn cast_robot<T: Robot + 'static>(robot: &mut Box<dyn Robot>) -> &mut T {
    robot
        .any_mut()
        .downcast_mut()
        .expect("We should always be downcasting to the correct robot")
}
