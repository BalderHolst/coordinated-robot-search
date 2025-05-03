//! Contains camera data format that the robot behaviors use

use crate::shapes::Cone;

/// Objects detected by the camera
#[derive(Debug, Default, Clone)]
pub struct CamData(Vec<CamPoint>);

impl CamData {
    /// Creates a new `CamData` object
    pub fn new() -> Self {
        Self(Vec::new())
    }

    /// Get the objects detected by the camera
    pub fn points(&self) -> &[CamPoint] {
        &self.0
    }

    /// Check if the camera data is empty
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl From<Vec<CamPoint>> for CamData {
    fn from(points: Vec<CamPoint>) -> Self {
        Self(points)
    }
}

/// An object detected by the camera
#[derive(Debug, Default, Clone)]
pub struct CamPoint {
    /// The cone containing the search object
    pub cone: Cone,

    /// The probability of the search object being in this cone
    pub probability: f32,
}
