use burn::prelude::*;

use crate::{
    behaviors::rl::{action::Action, network::Network, RlRobot},
    lidar::LidarPoint,
    params,
};

use super::State;

/// Minimal state representation using only the shortest lidar ray
/// and the robot's angle.
///
/// This is used as a proof of concept for the RL agent as it can
/// learn to avoid obstacles.
#[derive(Debug, Clone)]
pub struct RayState {
    angle: f32,
    ray: LidarPoint,
}

impl State for RayState {
    const SIZE: usize = 3;

    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self {
        let ray = robot
            .lidar
            .points()
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
            .cloned()
            .unwrap_or(LidarPoint {
                angle: 0.0,
                distance: params::LIDAR_RANGE,
            });
        let angle = ray.angle;
        Self { angle, ray }
    }

    fn to_tensor<B: burn::prelude::Backend>(
        &self,
        device: &B::Device,
    ) -> burn::prelude::Tensor<B, 1> {
        let Self { angle, ray } = self;
        Tensor::from_floats([*angle, ray.angle, ray.distance], device)
    }
}
