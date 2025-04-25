use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::{Pos2, Vec2};

use crate::{
    behaviors::rl::{action::Action, network::Network},
    utils::normalize_angle,
    LidarData,
};

use super::{utils::ArrayWriter, RlRobot, State};

#[derive(Debug, Clone)]
pub struct PolarState {
    pos: Pos2,
    angle: f32,
    lidar: LidarData,
    search_gradient: Vec2,
    // global_gradient: Vec2,
    closest_robot: Vec2,
    group_center: Vec2,
    group_spread: Vec2,
}

impl PolarState {
    const LIDAR_RAYS: usize = 12;
    const SHORTEST_RAY_SIZE: usize = 2;
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

    pub fn lidar_data(&self) -> [f32; Self::LIDAR_RAYS + Self::SHORTEST_RAY_SIZE] {
        let mut data = [0.0; Self::LIDAR_RAYS + 2];
        for (i, (_, d)) in self.lidar_rays().iter().enumerate() {
            data[i] = *d;
        }

        let shortest_ray = self.lidar.shortest_ray().unwrap_or_default();
        data[Self::LIDAR_RAYS] = shortest_ray.distance;
        data[Self::LIDAR_RAYS + 1] = shortest_ray.angle;

        data
    }

    pub fn pose_data(&self) -> [f32; Self::POSE_SIZE] {
        let towards_origin = -self.pos.to_vec2();
        [
            towards_origin.angle() - self.angle,
            towards_origin.length(),
            self.angle,
        ]
    }

    pub fn search_gradient_data(&self) -> [f32; Self::SEARCH_GRADIENT_SIZE] {
        let angle = self.search_gradient.angle() - self.angle;
        let length = self.search_gradient.length();
        [angle, length]
    }

    pub fn group_data(&self) -> [f32; Self::GROUP_SIZE] {
        let group_center_angle = self.group_center.angle() - self.angle;
        let group_center_length = self.group_center.length();
        let group_spread_angle = self.group_spread.angle() - self.angle;
        let group_spread_length = self.group_spread.length();
        let closest_robot_angle = self.closest_robot.angle() - self.angle;
        let closest_robot_length = self.closest_robot.length();
        [
            group_center_angle,
            group_center_length,
            group_spread_angle,
            group_spread_length,
            closest_robot_angle,
            closest_robot_length,
        ]
    }
}

impl State for PolarState {
    const SIZE: usize = Self::LIDAR_RAYS
        + Self::SHORTEST_RAY_SIZE
        + Self::POSE_SIZE
        + Self::SEARCH_GRADIENT_SIZE
        + Self::GROUP_SIZE;

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

    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
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
