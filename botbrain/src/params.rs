//! Robot parameters

use std::ops::Range;

/// Diameter of the robot in meters
pub const DIAMETER: f32 = 0.35;

/// Radius of the robot in meters
pub const RADIUS: f32 = DIAMETER / 2.0;

/// Range of the camera's object detection in meters
pub const CAM_RANGE: f32 = 3.0; // TODO: Probably should be higher

/// Angle of the camera's field of view in radians
pub const CAM_FOV: f32 = 1.25;

/// Lidar range in meters
pub const LIDAR_RANGE: f32 = 10.0;

/// Range in which two robots can communicate in meters
pub const COMMUNICATION_RANGE: f32 = 30.0;

/// Possible speed values a behavior can set
pub const SPEED_RANGE: Range<f32> = 0.0..1.0;

/// Maximum steer command.
pub const MAX_STEER: f32 = 1.0;

/// Possible steering values a behavior can set.
pub const STEER_RANGE: Range<f32> = -MAX_STEER..MAX_STEER;
