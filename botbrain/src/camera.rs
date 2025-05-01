use crate::shapes::Cone;

/// A point detected by the camera
#[derive(Debug, Clone)]
pub struct CamPoint {
    /// The angle of the point relative to the robot
    pub angle: f32,

    /// The probability of the search object being at this point
    pub probability: f32,
}

/// An object detected by the camera
#[derive(Debug, Clone)]
pub struct CamCone {
    /// The cone containing the search object
    pub cone: Cone,

    /// The probability of the search object being in this cone
    pub probability: f32,
}

/// Data from the camera
#[derive(Debug, Clone)]
pub enum CamData {
    Cone(CamCone),
    Points(Vec<CamPoint>),
}

impl Default for CamData {
    fn default() -> Self {
        Self::Points(vec![])
    }
}
