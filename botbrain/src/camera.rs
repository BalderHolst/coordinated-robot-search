//! Contains camera data format that the robot behaviors use

use crate::shapes::Cone;

/// An object detected by the camera
#[derive(Debug, Default, Clone)]
pub struct CamData {
    /// The cone containing the search object
    pub cone: Cone,

    /// The probability of the search object being in this cone
    pub probability: f32,
}
