use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::{Pos2, Vec2};
use rand::Rng;

use crate::{Control, LidarData};

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

impl RlState {
    const LIDAR_RAYS: usize = 12;
    const POSE_SIZE: usize = 3;
    const SEARCH_GRADIENT_SIZE: usize = 2;

    pub const SIZE: usize = Self::LIDAR_RAYS + Self::POSE_SIZE + Self::SEARCH_GRADIENT_SIZE;

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

    pub fn to_tensor<B: Backend>(&self) -> Tensor<B, 1> {
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

#[derive(Debug, Clone, Copy)]
pub struct RlAction(usize);

impl<B: Backend> From<Tensor<B, 2>> for RlAction {
    fn from(tensor: Tensor<B, 2>) -> Self {
        (tensor.argmax(1).to_data().as_slice::<i32>().unwrap()[0] as usize).into()
    }
}

impl<B: Backend> From<Tensor<B, 1>> for RlAction {
    fn from(tensor: Tensor<B, 1>) -> Self {
        let i = tensor.argmax(0).to_data();
        let i = i.as_slice::<i32>().unwrap();
        let i = i[0] as usize;
        i.into()
    }
}

impl From<usize> for RlAction {
    fn from(i: usize) -> Self {
        assert!(i < RlAction::SIZE, "Invalid action: {}", i);
        Self(i)
    }
}

impl From<RlAction> for usize {
    fn from(action: RlAction) -> Self {
        action.0
    }
}

impl RlAction {
    /// The number of discrete actions
    pub const SIZE: usize = Self::STEERS.len() * Self::SPEEDS.len();

    const STEERS: [f32; 5] = [-1.0, -0.5, 0.0, 0.5, 1.0];
    const SPEEDS: [f32; 3] = [0.1, 0.5, 1.0];

    pub fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    pub fn control(&self) -> Control {
        let Self(i) = self;
        let speed = Self::SPEEDS[i % Self::SPEEDS.len()];
        let steer = Self::STEERS[i / Self::SPEEDS.len()];
        Control { speed, steer }
    }
}

impl Default for RlAction {
    fn default() -> Self {
        (Self::SIZE / 2 + 1).into()
    }
}
