use emath::{Pos2, Vec2};

#[derive(Clone, Debug)]
pub enum DebugType {
    Radius(f32),
    Vector(Vec2),
    Vectors(Vec<Vec2>),
    WeightedVectors(Vec<(Vec2, f32)>),
    Point(Pos2),
    Points(Vec<Pos2>),
    WeightedPoints(Vec<(Pos2, f32)>),
    NumberPoints(Vec<(Pos2, f32)>),
    Number(f32),
}
