use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::{Pos2, Vec2};

use crate::{
    behaviors::rl::{action::Action, network},
    utils::normalize_angle,
    LidarData,
};

use super::{utils::ArrayWriter, RlRobot, State};

/// RL State representation using a mix of coordinate types
///
/// [`super::PolarState`] tries to use polar coordinates for everything
/// to make it easier for the model to learn parameter relationships.
#[derive(Debug, Clone)]
pub struct SmallState {
    pos: Pos2,
    angle: f32,
    lidar: LidarData,
    search_gradient: Vec2,
    // global_gradient: Vec2,
    closest_robot: Vec2,
    group_center: Vec2,
    group_spread: Vec2,
}

impl SmallState {
    const LIDAR_RAYS: usize = 12;
    const POSE_SIZE: usize = 3;
    const SEARCH_GRADIENT_SIZE: usize = 2;
    const GROUP_SIZE: usize = 6;

    /// Returns the interpolated lidar data in a list of `(angle, distance)` pairs
    pub fn lidar_rays(&self) -> [(f32, f32); Self::LIDAR_RAYS] {
        let mut data = [(0.0, 0.0); Self::LIDAR_RAYS];

        for (i, (a, d)) in data.iter_mut().enumerate() {
            *a = i as f32 * 2.0 * PI / Self::LIDAR_RAYS as f32;
            *d = self.lidar.interpolate(*a);
        }

        data
    }

    fn lidar_data(&self) -> [f32; Self::LIDAR_RAYS] {
        let mut data = [0.0; Self::LIDAR_RAYS];
        for (i, (_, d)) in self.lidar_rays().iter().enumerate() {
            data[i] = *d;
        }
        data
    }

    fn pose_data(&self) -> [f32; Self::POSE_SIZE] {
        [self.pos.x, self.pos.y, self.angle]
    }

    fn search_gradient_data(&self) -> [f32; Self::SEARCH_GRADIENT_SIZE] {
        [self.search_gradient.x, self.search_gradient.y]
    }

    fn group_data(&self) -> [f32; Self::GROUP_SIZE] {
        [
            self.group_center.x,
            self.group_center.y,
            self.group_spread.x,
            self.group_spread.y,
            self.closest_robot.x,
            self.closest_robot.y,
        ]
    }
}

impl State for SmallState {
    const SIZE: usize =
        Self::LIDAR_RAYS + Self::POSE_SIZE + Self::SEARCH_GRADIENT_SIZE + Self::GROUP_SIZE;

    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1> {
        let mut data = [0.0; Self::SIZE];
        {
            let mut w = ArrayWriter::new(&mut data);
            w.write_array(&self.lidar_data());
            w.write_array(&self.pose_data());
            w.write_array(&self.search_gradient_data());
            w.write_array(&self.group_data());
        }
        Tensor::from_floats(data, device)
    }

    fn from_robot<B: Backend, A: Action, N: network::Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self {
        // Map the robot position to the range [-1, 1]
        let pos = Pos2 {
            x: 2.0 * robot.pos.x / robot.search_grid.width(),
            y: 2.0 * robot.pos.y / robot.search_grid.height(),
        };

        let n = usize::max(robot.others.len(), 1);

        let group_center = robot
            .others
            .values()
            .map(|(p, _)| *p)
            .fold(Vec2::ZERO, |acc, p| acc + (p - robot.pos))
            / n as f32;

        let group_spread = robot
            .others
            .values()
            .map(|(p, _)| *p)
            .fold(Vec2::ZERO, |acc, p| {
                let center = robot.pos + group_center;
                let d = Vec2 {
                    x: p.x - center.x,
                    y: p.y - center.y,
                }
                .abs();
                acc + d
            })
            / n as f32;

        let closest_robot = robot
            .others
            .values()
            .map(|(p, _)| *p)
            .min_by(|a, b| robot.pos.distance(*a).total_cmp(&robot.pos.distance(*b)))
            .unwrap_or(robot.pos)
            - robot.pos;

        Self {
            pos,
            angle: normalize_angle(robot.angle),
            lidar: robot.lidar.clone(),
            search_gradient: robot.search_gradient,
            closest_robot,
            group_center,
            group_spread,
        }
    }
}
