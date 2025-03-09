use std::collections::HashMap;

use emath::{Pos2, Vec2};

use crate::scaled_grid::ScaledGrid;

#[derive(Clone)]
pub struct DebugSoup(pub Option<HashMap<&'static str, DebugType>>);

impl DebugSoup {
    pub fn new_active() -> Self {
        Self(Some(HashMap::with_capacity(0)))
    }

    pub fn new_inactive() -> Self {
        Self(None)
    }

    pub fn activate(&mut self) {
        self.0 = Some(HashMap::with_capacity(0));
    }

    pub fn deactivate(&mut self) {
        self.0 = None;
    }

    pub fn add(&mut self, key: &'static str, value: DebugType) {
        if let Some(map) = &mut self.0 {
            map.insert(key, value);
        }
    }

    pub fn remove(&mut self, key: &str) {
        if let Some(map) = &mut self.0 {
            map.remove(key);
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
    Vector(Vec2),
    Vectors(Vec<Vec2>),
    WeightedVectors(Vec<(Vec2, f32)>),
    Point(Pos2),
    Points(Vec<Pos2>),
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
        soup.add("Lidar", DebugType::RobotRays(rays));
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
        soup.add("Cam Range", DebugType::RobotLine(points));
    }
}
