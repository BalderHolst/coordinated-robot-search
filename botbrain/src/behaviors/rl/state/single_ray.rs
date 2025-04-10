use burn::prelude::*;

use crate::{
    behaviors::rl::{action::Action, RlRobot},
    params, LidarPoint,
};

use super::State;

#[derive(Debug, Clone)]
pub struct RayState(LidarPoint);

impl State for RayState {
    const SIZE: usize = 2;

    fn from_robot<A: Action>(robot: &RlRobot<Self, A>) -> Self {
        let ray = robot
            .lidar
            .points()
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
            .cloned()
            .unwrap_or(LidarPoint {
                angle: 0.0,
                distance: params::LIDAR_RANGE,
            });
        Self(ray)
    }

    fn to_tensor<B: burn::prelude::Backend>(
        &self,
        device: &B::Device,
    ) -> burn::prelude::Tensor<B, 1> {
        let Self(ray) = self;
        Tensor::from_floats([ray.angle, ray.distance], device)
    }
}
