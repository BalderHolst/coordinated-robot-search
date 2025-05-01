//! Various shapes

use std::f32::consts::PI;

use emath::Pos2;
use serde::{Deserialize, Serialize};

/// A simple geometric shape
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(untagged)] // FIX: Figure out why this crashes in ros_agent
#[allow(missing_docs)]
pub enum Shape {
    Circle(Circle),
    Cone(Cone),
    WideLine(WideLine),
    Line(Line),
}

impl Shape {
    /// Area of the shape
    pub fn area(&self) -> f32 {
        match self {
            Shape::Circle(circle) => PI * circle.radius.powi(2),
            Shape::Cone(cone) => PI * cone.radius.powi(2) * (cone.fov / 360.0),
            Shape::WideLine(wline) => wline.width * wline.line.length(),
            Shape::Line(line) => line.length(),
        }
    }
}

/// A geometric circle
#[cfg_attr(feature = "bin-msgs", repr(C))]
#[cfg_attr(feature = "bin-msgs", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    /// Center of the circle
    pub center: Pos2,
    /// Radius of the circle
    pub radius: f32,
}

/// A geometric cone
#[cfg_attr(feature = "bin-msgs", repr(C))]
#[cfg_attr(feature = "bin-msgs", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Cone {
    /// "Tip" of the cone
    pub center: Pos2,
    /// Radius of the cone
    pub radius: f32,
    /// Angle that the cone points towards in radians
    pub angle: f32,
    /// Field of view of the cone in degrees
    pub fov: f32,
}

/// A line between two points
#[cfg_attr(feature = "bin-msgs", repr(C))]
#[cfg_attr(feature = "bin-msgs", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Line {
    /// Start point
    pub start: Pos2,
    /// End point
    pub end: Pos2,
}

impl Line {
    /// Cartesian length of the line
    pub fn length(&self) -> f32 {
        (self.end - self.start).length()
    }
}

/// A [Line] with a width
#[cfg_attr(feature = "bin-msgs", repr(C))]
#[cfg_attr(feature = "bin-msgs", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct WideLine {
    /// Line
    pub line: Line,
    /// Width of the line
    pub width: f32,
}

#[cfg(feature = "bin-msgs")]
mod bincode_impls {
    use super::*;
    use bincode::{de::read::Reader, enc::write::Writer};

    macro_rules! impl_bincode {
        ($ty:ident) => {
            impl bincode::Encode for $ty {
                fn encode<E: bincode::enc::Encoder>(
                    &self,
                    encoder: &mut E,
                ) -> Result<(), bincode::error::EncodeError> {
                    encoder.writer().write(bytemuck::bytes_of(self))
                }
            }

            impl bincode::Decode for $ty {
                fn decode<D: bincode::de::Decoder>(
                    decoder: &mut D,
                ) -> Result<Self, bincode::error::DecodeError> {
                    let mut bytes = [0u8; std::mem::size_of::<$ty>()];
                    decoder.reader().read(&mut bytes)?;
                    Ok(*bytemuck::from_bytes(&bytes))
                }
            }

            impl<'de> bincode::BorrowDecode<'de> for $ty {
                fn borrow_decode<D: bincode::de::Decoder>(
                    decoder: &mut D,
                ) -> Result<Self, bincode::error::DecodeError> {
                    let mut bytes = [0u8; std::mem::size_of::<$ty>()];
                    decoder.reader().read(&mut bytes)?;
                    Ok(*bytemuck::from_bytes(&bytes))
                }
            }
        };
    }

    impl_bincode!(Line);
    impl_bincode!(Cone);
    impl_bincode!(Circle);
    impl_bincode!(WideLine);
}
