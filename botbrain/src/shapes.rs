//! Various shapes

use std::f32::consts::PI;

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

impl Shape {
    pub fn area(&self) -> f32 {
        match self {
            Shape::Circle(circle) => PI * circle.radius.powi(2),
            Shape::Cone(cone) => PI * cone.radius.powi(2) * (cone.fov / 360.0),
            Shape::WideLine(wline) => wline.width * wline.line.length(),
            Shape::Line(line) => line.length(),
        }
    }
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

impl Line {
    pub fn length(&self) -> f32 {
        (self.end - self.start).length()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WideLine {
    pub line: Line,
    pub width: f32,
}
