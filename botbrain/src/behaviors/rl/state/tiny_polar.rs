use burn::{prelude::Backend, tensor::Tensor};
use emath::Vec2;

use crate::{
    behaviors::rl::{action::Action, network::Network},
    debug_soup::{DebugItem, DebugSoup},
    lidar::LidarPoint,
    utils::normalize_angle,
};

use super::{utils::ArrayWriter, RlRobot, State};

/// Tiny RL State representation using polar coordinates
#[derive(Debug, Clone)]
pub struct TinyPolarState {
    angle: f32,
    lidar_ray: LidarPoint,
    search_gradient: Vec2,
}

impl TinyPolarState {
    const SHORTEST_RAY_SIZE: usize = 2;
    const POSE_SIZE: usize = 1;
    const SEARCH_GRADIENT_SIZE: usize = 2;

    fn lidar_data(&self) -> [f32; Self::SHORTEST_RAY_SIZE] {
        [self.lidar_ray.angle, self.lidar_ray.distance]
    }

    fn pose_data(&self) -> [f32; Self::POSE_SIZE] {
        [self.angle]
    }

    fn search_gradient_data(&self) -> [f32; Self::SEARCH_GRADIENT_SIZE] {
        let angle = self.search_gradient.angle() - self.angle;
        let length = self.search_gradient.length();
        [angle, length]
    }

    /// Adds the state to a debug soup for visualization
    pub fn visualize(&self, soup: &mut DebugSoup) {
        if !soup.is_active() {
            return;
        }

        let category = "RL State";

        soup.add(category, "Robot Angle", DebugItem::Number(self.angle));

        soup.add(
            category,
            "Shortest Lidar Ray",
            DebugItem::RobotRays(vec![(self.lidar_ray.angle, self.lidar_ray.distance)]),
        );

        soup.add(
            category,
            "Search Gradient",
            DebugItem::Vector(self.search_gradient),
        );
    }
}

impl State for TinyPolarState {
    const SIZE: usize = Self::SHORTEST_RAY_SIZE + Self::POSE_SIZE + Self::SEARCH_GRADIENT_SIZE;

    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1> {
        let data: [f32; Self::SIZE] = {
            let mut w = ArrayWriter::new();
            w.write_array(&self.lidar_data());
            w.write_array(&self.pose_data());
            w.write_array(&self.search_gradient_data());
            w.finish()
        };
        Tensor::from_floats(data, device)
    }

    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self {
        let angle = normalize_angle(robot.angle);
        let search_gradient = robot.search_gradient;
        let lidar_ray = robot.lidar.shortest_ray().unwrap_or_default();

        Self {
            angle,
            search_gradient,
            lidar_ray,
        }
    }
}
