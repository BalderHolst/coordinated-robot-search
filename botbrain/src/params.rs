use std::ops::Range;

pub const DIAMETER: f32 = 0.5;
pub const RADIUS: f32 = DIAMETER / 2.0;
pub const CAM_RANGE: f32 = 3.0; // TODO: Probably should be higher
pub const CAM_FOV: f32 = 1.25;
pub const LIDAR_RANGE: f32 = 5.0;
pub const COMMUNICATION_RANGE: f32 = 30.0;
pub const SPEED_RANGE: Range<f32> = 0.0..1.0;
pub const STEER_RANGE: Range<f32> = -1.0..1.0;
