use serde::{Deserialize, Serialize};

use crate::utils::normalize_angle;

/// A point detected by the lidar
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LidarPoint {
    /// The angle of the point relative to the robot
    pub angle: f32,

    /// The distance to the object
    pub distance: f32,
}

/// Data from the lidar. Points have angles within the range [-PI, PI].
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LidarData(pub Vec<LidarPoint>);

impl LidarData {
    pub fn new(mut points: Vec<LidarPoint>) -> Self {
        points
            .iter_mut()
            .for_each(|p| p.angle = normalize_angle(p.angle));
        points.sort_by(|a, b| a.angle.total_cmp(&b.angle));
        Self(points)
    }

    pub fn points(&self) -> impl Iterator<Item = &LidarPoint> {
        self.0.iter()
    }

    pub fn into_points(self) -> Vec<LidarPoint> {
        self.0
    }

    pub fn within_fov(&self, fov: f32) -> Self {
        let range = -fov / 2.0..fov / 2.0;
        let points: Vec<_> = self
            .points()
            .filter(|&p| range.contains(&p.angle))
            .cloned()
            .collect();
        Self(points)
    }

    /// Interpolates the distance to an object at a given continuous angle
    pub fn interpolate(&self, angle: f32) -> f32 {
        let Self(points) = &self;

        if points.is_empty() {
            return 0.0;
        }

        if points.len() == 1 {
            return points[0].distance;
        }

        let angle = normalize_angle(angle);

        let mut i: usize = 0;
        for point in points {
            if point.angle > angle {
                break;
            }
            i += 1;
        }

        let a = points.get(i.saturating_sub(1));
        let b = points.get(i);

        match (a, b) {
            (None, None) => 0.0,
            (None, Some(p)) | (Some(p), None) => p.distance,
            (Some(a), Some(b)) if a.distance == b.distance => a.distance,
            (Some(a), Some(b)) => {
                let t = (angle - a.angle) / (b.angle - a.angle);
                a.distance + t * (b.distance - a.distance)
            }
        }
    }

    pub fn shortest_ray(&self) -> Option<LidarPoint> {
        self.points()
            .max_by(|a, b| a.distance.total_cmp(&b.distance))
            .cloned()
    }
}

#[test]
fn test_lidar_interpolate() {
    use std::f32::consts::PI;

    let points = vec![
        LidarPoint {
            angle: -PI / 2.0,
            distance: 1.0,
        },
        LidarPoint {
            angle: 0.0,
            distance: 2.0,
        },
        LidarPoint {
            angle: PI / 2.0,
            distance: 3.0,
        },
    ];
    let lidar = LidarData::new(points);

    assert_eq!(lidar.interpolate(-PI), 1.0);
    assert_eq!(lidar.interpolate(0.0), 2.0);
    assert_eq!(lidar.interpolate(PI / 4.0), 2.5);
}
