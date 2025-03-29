//! Various shapes

use emath::Pos2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
// #[serde(untagged)]
pub enum Shape {
    Circle(Circle),
    Cone(Cone),
    WideLine(WideLine),
    Line(Line),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Circle {
    pub center: Pos2,
    pub radius: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cone {
    pub center: Pos2,
    pub radius: f32,
    pub angle: f32,
    pub fov: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Line {
    pub start: Pos2,
    pub end: Pos2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WideLine {
    pub start: Pos2,
    pub end: Pos2,
    pub width: f32,
}
