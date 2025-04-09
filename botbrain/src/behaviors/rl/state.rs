use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::{Pos2, Vec2};

use crate::LidarData;

pub trait State: Clone + Send {
    const SIZE: usize;
    fn to_tensor<B: Backend>(&self) -> Tensor<B, 1>;
}

#[derive(Debug, Clone)]
pub struct RlState {
    pos: Pos2,
    angle: f32,
    lidar: LidarData,
    search_gradient: Vec2,
    // group_center: Pos2,
    // group_spread: Vec2,
    // global_gradient: Vec2,
}

impl State for RlState {
    const SIZE: usize = Self::LIDAR_RAYS + Self::POSE_SIZE + Self::SEARCH_GRADIENT_SIZE;

    fn to_tensor<B: Backend>(&self) -> Tensor<B, 1> {
        let device = Default::default();
        Tensor::cat(
            vec![
                Tensor::from_floats(self.lidar_data(), &device),
                Tensor::from_floats(self.pose_data(), &device),
                Tensor::from_floats(self.search_gradient_data(), &device),
            ],
            0,
        )
    }
}

impl RlState {
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
