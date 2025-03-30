use std::{f32::consts::PI, marker::PhantomData};

use burn::{prelude::Backend, tensor::Tensor};
use emath::Pos2;
use rand::Rng;

use crate::{Control, LidarData};

pub struct RlState<B: Backend> {
    pub pos: Pos2,
    pub angle: f32,
    pub lidar: LidarData,
    // group_center: Pos2,
    // group_spread: Vec2,
    // search_gradient: Vec2,
    // global_gradient: Vec2,
    backend: PhantomData<B>,
}

impl<B: Backend> RlState<B> {
    pub fn new(pos: Pos2, angle: f32, lidar: LidarData) -> Self {
        Self {
            pos,
            angle,
            lidar,
            backend: PhantomData,
        }
    }

    pub fn lidar_data<const RAYS: usize>(&self) -> [f32; RAYS] {
        let mut data = [0.0; RAYS];

        for (i, d) in data.iter_mut().enumerate() {
            let angle = i as f32 * 2.0 * PI / RAYS as f32;
            *d = self.lidar.interpolate(angle);
        }

        data
    }

    pub fn pose_data(&self) -> [f32; 3] {
        [self.pos.x, self.pos.y, self.angle]
    }

    pub fn to_tensor<const LIDAR_RAYS: usize>(&self) -> Tensor<B, 1> {
        let device = Default::default();

        let lidar_data = self.lidar_data::<LIDAR_RAYS>();
        let pose_data = self.pose_data();

        let lidar_tensor = Tensor::from_floats(lidar_data, &device);
        let pose_tensor = Tensor::from_floats(pose_data, &device);

        Tensor::cat(vec![pose_tensor, lidar_tensor], 1)
    }
}

pub struct RlAction(usize);

impl<B: Backend> From<Tensor<B, 1>> for RlAction {
    fn from(tensor: Tensor<B, 1>) -> Self {
        let i = tensor.argmax(1).to_data(); // TODO: I an not sure that `1` is correct
        let i = i.as_slice::<i64>().unwrap();
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

    fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    fn control(&self) -> Control {
        let Self(i) = self;

        let speed = 0.5;
        let steer = *i as f32 / (Self::SIZE - 1) as f32 * 2.0 * Self::MAX_STEER - Self::MAX_STEER;

        Control { speed, steer }
    }
}
