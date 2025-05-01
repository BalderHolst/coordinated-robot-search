//! [DebugSoup] is a structure that holds debug information about the robot's state.
//! It can be used to visualize various aspects of the robot's behavior during simulation.

use std::{collections::BTreeMap, sync::Arc};

use emath::{Pos2, Vec2};

use crate::{scaled_grid::ScaledGrid, Map};

#[derive(Clone)]
struct InnerSoup {
    unnamed: BTreeMap<&'static str, Arc<DebugItem>>,
    named: BTreeMap<&'static str, BTreeMap<&'static str, Arc<DebugItem>>>,
}

impl InnerSoup {
    fn new() -> Self {
        Self {
            unnamed: BTreeMap::new(),
            named: BTreeMap::new(),
        }
    }
}

/// A structure that holds debug information about the robot's state. It is a disableable
/// key value store containing [DebugItem]s. The debug soup only stores the raw data and
/// it is up to the user to visualize it. The soup can be deactivated to increase the
/// performance and reduce memory usage when running a behavior.
#[derive(Clone)]
pub struct DebugSoup(Option<InnerSoup>);

impl DebugSoup {
    /// Create a new active debug soup
    pub fn new_active() -> Self {
        Self(Some(InnerSoup::new()))
    }

    /// Create a new inactive debug soup (no debug information will be stored)
    pub fn new_inactive() -> Self {
        Self(None)
    }

    /// Activate the debug soup
    pub fn activate(&mut self) {
        self.0 = Some(InnerSoup::new());
    }

    /// Deactivate the debug soup
    pub fn deactivate(&mut self) {
        self.0 = None;
    }

    /// Iterate over all debug items in the soup. It returns an iterator of tuples `(category, key, value)`.
    pub fn iter(&self) -> impl Iterator<Item = (&'static str, &'static str, &Arc<DebugItem>)> {
        let unnamed_iter = self.0.iter().flat_map(|inner| inner.unnamed.iter());
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

    /// Get a debug item by category and key. Returns `None` if the item is not found.
    pub fn get(&self, category: &'static str, key: &'static str) -> Option<Arc<DebugItem>> {
        if let Some(inner) = &self.0 {
            match category {
                "" => inner.unnamed.get(key),
                _ => inner.named.get(category).and_then(|m| m.get(key)),
            }
            .cloned()
        } else {
            None
        }
    }

    /// Add a new debug item to the soup. If the category is empty, the item will be stored in the unnamed category.
    pub fn add(&mut self, category: &'static str, key: &'static str, value: DebugItem) {
        if let Some(inner) = &mut self.0 {
            let value = Arc::new(value);
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

    /// Remeve a debug item from the soup. If the category is empty, the item will be removed from the unnamed category.
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

    /// Check if the debug soup is active
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
pub enum DebugItem {
    /// A circle with a given radius around to the robot's position
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

    /// Line with coordinates relative to the global map
    GlobalLine(Vec<Pos2>),

    /// Global line with coordinates relative to the global map with a weight
    NumberPoints(Vec<(Pos2, f32)>),

    /// A simple number
    Number(f32),

    /// A simple integer
    Int(i32),

    /// A robot map
    Map(Map),

    /// A scaled float grid
    Grid(ScaledGrid<f32>),
}

pub(crate) mod common_routines {
    //! Common routines for showing [botbrain] data structures as debug information

    use emath::Vec2;

    use crate::{params, DebugSoup, LidarData};

    use super::DebugItem;

    pub(crate) fn show_lidar(soup: &mut DebugSoup, lidar: &LidarData) {
        if !soup.is_active() {
            return;
        }
        let rays = lidar.points().map(|p| (p.angle, p.distance)).collect();
        soup.add("Sensors", "Lidar", DebugItem::RobotRays(rays));
    }

    pub(crate) fn show_cam_range(soup: &mut DebugSoup, lidar: &LidarData) {
        if !soup.is_active() {
            return;
        }

        const CAM_RANGE_STEPS: usize = 20;
        let step_size = params::CAM_FOV / CAM_RANGE_STEPS as f32;
        let points = (0..CAM_RANGE_STEPS)
            .map(|i| {
                let angle = i as f32 * step_size - params::CAM_FOV / 2.0;
                let dist = lidar.interpolate(angle).min(params::CAM_RANGE);
                Vec2::angled(angle) * dist
            })
            .collect();
        soup.add("Sensors", "Cam Range", DebugItem::RobotLine(points));
    }
}
