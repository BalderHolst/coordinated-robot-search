use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::{Pos2, Vec2};

use crate::{
    behaviors::rl::{action::Action, model::Network},
    utils::normalize_angle,
    LidarData,
};

use super::{RlRobot, State};

#[derive(Debug, Clone)]
pub struct SmallSoloState {
    pos: Pos2,
    angle: f32,
    lidar: LidarData,
    search_gradient: Vec2,
}

impl State for SmallSoloState {
    const SIZE: usize = Self::LIDAR_RAYS + Self::POSE_SIZE + Self::SEARCH_GRADIENT_SIZE;

    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1> {
        Tensor::cat(
            vec![
                Tensor::from_floats(self.lidar_data(), device),
                Tensor::from_floats(self.pose_data(), device),
                Tensor::from_floats(self.search_gradient_data(), device),
            ],
            0,
        )
    }

    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self {
        // Map the robot position to the range [-1, 1]
        let pos = Pos2 {
            x: 2.0 * robot.pos.x / robot.search_grid.width(),
            y: 2.0 * robot.pos.y / robot.search_grid.height(),
        };
        SmallSoloState::new(
            pos,
            normalize_angle(robot.angle),
            robot.lidar.clone(),
            robot.search_gradient,
        )
    }
}

impl SmallSoloState {
    const LIDAR_RAYS: usize = 12;
    const POSE_SIZE: usize = 3;
    const SEARCH_GRADIENT_SIZE: usize = 2;

    pub fn new(pos: Pos2, angle: f32, lidar: LidarData, search_gradient: Vec2) -> Self {
        Self {
            pos,
            angle,
            lidar,
            search_gradient,
        }
    }

    /// Returns the interpolated lidar data in a list of `(angle, distance)` pairs
    pub fn lidar_rays(&self) -> [(f32, f32); Self::LIDAR_RAYS] {
        let mut data = [(0.0, 0.0); Self::LIDAR_RAYS];

        for (i, (a, d)) in data.iter_mut().enumerate() {
            *a = i as f32 * 2.0 * PI / Self::LIDAR_RAYS as f32;
            *d = self.lidar.interpolate(*a);
        }

        data
    }

    pub fn lidar_data(&self) -> [f32; Self::LIDAR_RAYS] {
        let mut data = [0.0; Self::LIDAR_RAYS];
        for (i, (_, d)) in self.lidar_rays().iter().enumerate() {
            data[i] = *d;
        }
        data
    }

    pub fn pose_data(&self) -> [f32; Self::POSE_SIZE] {
        [self.pos.x, self.pos.y, self.angle]
    }

    pub fn search_gradient_data(&self) -> [f32; Self::SEARCH_GRADIENT_SIZE] {
        [self.search_gradient.x, self.search_gradient.y]
    }
}
