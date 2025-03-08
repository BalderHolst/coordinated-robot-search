//! Utility functions for the botbrain library

use std::f32::consts::PI;

/// Normalize an angle to be in the range [-PI, PI]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut angle = angle;
    while angle < -PI {
        angle += 2.0 * PI;
    }
    while angle > PI {
        angle -= 2.0 * PI;
    }
    angle
}
