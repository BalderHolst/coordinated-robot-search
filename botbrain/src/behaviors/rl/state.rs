use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::Pos2;
use rand::Rng;

use crate::{utils::normalize_angle, Control, LidarData};

pub const LIDAR_RAYS: usize = 10;

#[derive(Debug, Clone)]
pub struct RlState {
    pub pos: Pos2,
    pub angle: f32,
    pub lidar: LidarData,
    // group_center: Pos2,
    // group_spread: Vec2,
    // search_gradient: Vec2,
    // global_gradient: Vec2,
}

impl RlState {
    pub fn new(pos: Pos2, angle: f32, lidar: LidarData) -> Self {
        Self { pos, angle, lidar }
    }

    /// Returns the interpolated lidar data in a list of `(angle, distance)` pairs
    pub fn lidar_rays(&self) -> [(f32, f32); LIDAR_RAYS] {
        let mut data = [(0.0, 0.0); LIDAR_RAYS];

        for (i, (a, d)) in data.iter_mut().enumerate() {
            *a = normalize_angle(i as f32 * 2.0 * PI / LIDAR_RAYS as f32);
            *d = self.lidar.interpolate(*a);
        }

        data
    }

    pub fn lidar_data(&self) -> [f32; LIDAR_RAYS] {
        let mut data = [0.0; LIDAR_RAYS];
        for (i, (_, d)) in self.lidar_rays().iter().enumerate() {
            data[i] = *d;
        }
        data
    }

    pub fn pose_data(&self) -> [f32; 3] {
        [self.pos.x, self.pos.y, self.angle]
    }

    pub fn to_tensor<B: Backend>(&self) -> Tensor<B, 1> {
        // let device = Default::default();

        // let lidar_data = self.lidar_data();
        // let pose_data = self.pose_data();

        // let lidar_tensor = Tensor::from_floats(lidar_data, &device);
        // let pose_tensor = Tensor::from_floats(pose_data, &device);

        // Tensor::cat(vec![pose_tensor, lidar_tensor], 0)

        let rays = self.lidar_rays();
        let Some(shortest_ray) = rays.iter().min_by(|a, b| a.1.total_cmp(&b.1)) else {
            eprintln!("[WARNING]: No lidar data: {:?}", rays);
            return Tensor::from_floats([0.0, 0.0], &Default::default());
        };

        Tensor::from_floats([shortest_ray.0, shortest_ray.1], &Default::default())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RlAction(usize);

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
    const SIZE: usize = 7;

    /// The maximum steering command
    const MAX_STEER: f32 = 1.0;

    pub fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    pub fn control(&self) -> Control {
        let Self(i) = self;

        let speed = 0.5;
        let steer = *i as f32 / (Self::SIZE - 1) as f32 * 2.0 * Self::MAX_STEER - Self::MAX_STEER;

        Control { speed, steer }
    }
}

impl Default for RlAction {
    fn default() -> Self {
        (Self::SIZE / 2 + 1).into()
    }
}
