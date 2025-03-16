use std::collections::BTreeMap;

use emath::{Pos2, Vec2};

use crate::scaled_grid::ScaledGrid;

#[derive(Clone)]
struct InnerSoup {
    unnamed: BTreeMap<&'static str, DebugType>,
    named: BTreeMap<&'static str, BTreeMap<&'static str, DebugType>>,
}

impl InnerSoup {
    fn new() -> Self {
        Self {
            unnamed: BTreeMap::new(),
            named: BTreeMap::new(),
        }
    }
}

#[derive(Clone)]
pub struct DebugSoup(Option<InnerSoup>);

impl DebugSoup {
    pub fn new_active() -> Self {
        Self(Some(InnerSoup::new()))
    }

    pub fn new_inactive() -> Self {
        Self(None)
    }

    pub fn activate(&mut self) {
        self.0 = Some(InnerSoup::new());
    }

    pub fn deactivate(&mut self) {
        self.0 = None;
    }

    pub fn iter(&self) -> impl Iterator<Item = (&'static str, &'static str, &DebugType)> {
        let unnamed_iter = self
            .0
            .iter()
            .flat_map(|inner| inner.unnamed.iter().map(|(key, value)| (key, value)));
        let named_iter = self
            .0
            .iter()
            .flat_map(|inner| inner.named.iter())
            .filter_map(|(category, map)| match *category {
                "" => None,
                _ => Some(map.iter().map(move |(key, value)| (*category, *key, value))),
            })
            .flatten();
        unnamed_iter
            .map(|(key, value)| ("", *key, value))
            .chain(named_iter)
    }

    pub fn add(&mut self, category: &'static str, key: &'static str, value: DebugType) {
        if let Some(inner) = &mut self.0 {
            match category {
                "" => {
                    inner.unnamed.insert(key, value);
                }
                _ => {
                    inner
                        .named
                        .entry(category)
                        .or_insert_with(BTreeMap::new)
                        .insert(key, value);
                }
            }
        }
    }

    pub fn remove(&mut self, category: &'static str, key: &'static str) {
        if let Some(map) = &mut self.0 {
            match category {
                "" => {
                    map.unnamed.remove(key);
                }
                _ => {
                    map.named.entry(category).and_modify(|m| {
                        m.remove(key);
                    });
                }
            }
        }
    }

    pub fn is_active(&self) -> bool {
        self.0.is_some()
    }
}

impl Default for DebugSoup {
    fn default() -> Self {
        Self::new_inactive()
    }
}

/// A type of debug information that can be shown in a simulator
#[derive(Clone, Debug)]
pub enum DebugType {
    Radius(f32),

    /// Vector from the robot's position with an angle relative to the robot's orientation
    Vector(Vec2),

    /// Multiple vectors from the robot's position with an angle relative to the robot's orientation
    Vectors(Vec<Vec2>),

    /// Multiple weighted vectors from the robot's position with an angle relative to the robot's orientation
    WeightedVectors(Vec<(Vec2, f32)>),

    /// A vector field with vectors originating from the given global positions and angles
    VectorField(Vec<(Pos2, Vec2)>),

    /// A global point
    Point(Pos2),

    /// Multiple global points
    Points(Vec<Pos2>),

    /// Multiple weighted global points
    WeightedPoints(Vec<(Pos2, f32)>),

    /// Rays originating from the robot with a given (angle, length) pair.
    /// The angle is relative to robot's orientation.
    RobotRays(Vec<(f32, f32)>),

    /// A line with coordinates relative to the robot's position
    RobotLine(Vec<Vec2>),

    NumberPoints(Vec<(Pos2, f32)>),
    Number(f32),

    Grid(ScaledGrid<f32>),
}

pub mod common_routines {
    use emath::Vec2;

    use crate::{DebugSoup, LidarData, RobotParameters};

    use super::DebugType;

    pub(crate) fn show_lidar(soup: &mut DebugSoup, lidar: &LidarData) {
        if !soup.is_active() {
            return;
        }
        let rays = lidar.points().map(|p| (p.angle, p.distance)).collect();
        soup.add("Sensors", "Lidar", DebugType::RobotRays(rays));
    }

    pub(crate) fn show_cam_range(
        soup: &mut DebugSoup,
        lidar: &LidarData,
        params: &RobotParameters,
    ) {
        if !soup.is_active() {
            return;
        }

        const CAM_RANGE_STEPS: usize = 20;
        let step_size = params.cam_fov / CAM_RANGE_STEPS as f32;
        let points = (0..CAM_RANGE_STEPS)
            .map(|i| {
                let angle = i as f32 * step_size - params.cam_fov / 2.0;
                let dist = lidar.interpolate(angle).min(params.cam_range);
                Vec2::angled(angle) * dist
            })
            .collect();
        soup.add("Sensors", "Cam Range", DebugType::RobotLine(points));
    }
}
