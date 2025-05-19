use std::f32::consts::PI;

use burn::{prelude::Backend, tensor::Tensor};
use emath::Vec2;

use crate::{
    behaviors::rl::{action::Action, network::Network},
    debug_soup::{DebugItem, DebugSoup},
    utils::normalize_angle,
    LidarData,
};

use super::{utils::ArrayWriter, RlRobot, State};

/// RL State representation using polar coordinates
#[derive(Debug, Clone)]
pub struct PolarState {
    angle: f32,
    avg_angle: f32,
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
    const POSE_SIZE: usize = 1;
    const SEARCH_GRADIENT_SIZE: usize = 2;
    const GROUP_SIZE: usize = 7;

    /// Returns the interpolated lidar data in a list of `(angle, distance)` pairs
    pub fn lidar_rays(&self) -> [(f32, f32); Self::LIDAR_RAYS] {
        let mut data = [(0.0, 0.0); Self::LIDAR_RAYS];

        for (i, (a, d)) in data.iter_mut().enumerate() {
            *a = i as f32 * 2.0 * PI / Self::LIDAR_RAYS as f32;
            *d = self.lidar.interpolate(*a);
        }

        data
    }

    fn lidar_data(&self) -> [f32; Self::LIDAR_RAYS + Self::SHORTEST_RAY_SIZE] {
        let mut data = [0.0; Self::LIDAR_RAYS + 2];
        for (i, (_, d)) in self.lidar_rays().iter().enumerate() {
            data[i] = *d;
        }

        let shortest_ray = self.lidar.shortest_ray().unwrap_or_default();
        data[Self::LIDAR_RAYS] = shortest_ray.distance;
        data[Self::LIDAR_RAYS + 1] = shortest_ray.angle;

        data
    }

    fn pose_data(&self) -> [f32; Self::POSE_SIZE] {
        [self.angle]
    }

    fn search_gradient_data(&self) -> [f32; Self::SEARCH_GRADIENT_SIZE] {
        let angle = self.search_gradient.angle() - self.angle;
        let length = self.search_gradient.length();
        [angle, length]
    }

    fn group_data(&self) -> [f32; Self::GROUP_SIZE] {
        let group_angle = normalize_angle(self.avg_angle - self.angle);
        let group_center_angle = self.group_center.angle() - self.angle;
        let group_center_length = self.group_center.length();
        let group_spread_angle = self.group_spread.angle() - self.angle;
        let group_spread_length = self.group_spread.length();
        let closest_robot_angle = self.closest_robot.angle() - self.angle;
        let closest_robot_length = self.closest_robot.length();
        [
            group_angle,
            group_center_angle,
            group_center_length,
            group_spread_angle,
            group_spread_length,
            closest_robot_angle,
            closest_robot_length,
        ]
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
            "Interpolated Lidar",
            DebugItem::RobotRays(self.lidar_rays().to_vec()),
        );

        soup.add(
            category,
            "Shortest Lidar Ray",
            DebugItem::RobotRays(vec![self
                .lidar
                .shortest_ray()
                .map(|r| (r.angle, r.distance))
                .unwrap_or((0.0, 0.0))]),
        );

        soup.add(
            category,
            "Search Gradient",
            DebugItem::Vector(self.search_gradient),
        );

        soup.add(
            category,
            "Closest Robot",
            DebugItem::Vector(self.closest_robot),
        );

        soup.add(
            category,
            "Avg Angle Value",
            DebugItem::Number(self.avg_angle),
        );

        soup.add(
            category,
            "Avg Angle",
            DebugItem::Vector(Vec2::angled(self.avg_angle)),
        );

        soup.add(
            category,
            "Group Center",
            DebugItem::Vector(self.group_center),
        );

        soup.add(
            category,
            "Group Spread X",
            DebugItem::Number(self.group_spread.x),
        );

        soup.add(
            category,
            "Group Spread Y",
            DebugItem::Number(self.group_spread.y),
        );
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
        let other_positions = robot.others.values().map(|(p, _)| *p).collect::<Vec<_>>();

        let closest_robot = other_positions
            .iter()
            .min_by(|a, b| robot.pos.distance(**a).total_cmp(&robot.pos.distance(**b)))
            .cloned()
            .unwrap_or(robot.pos)
            - robot.pos;

        let mut robot_positions = other_positions;
        robot_positions.push(robot.pos);

        let n = robot_positions.len();

        let group_center = robot_positions
            .iter()
            .fold(Vec2::ZERO, |acc, p| acc + p.to_vec2())
            / n as f32
            - robot.pos.to_vec2();

        let group_spread = robot_positions.iter().fold(Vec2::ZERO, |acc, p| {
            let center = robot.pos + group_center;
            let d = Vec2 {
                x: p.x - center.x,
                y: p.y - center.y,
            }
            .abs();
            acc + d
        }) / n as f32;

        let robot_angle = normalize_angle(robot.angle);

        let avg_angle = (robot
            .others
            .values()
            .map(|(_, angle)| Vec2::angled(*angle))
            .fold(Vec2::ZERO, |acc, p| acc + p)
            + Vec2::angled(robot_angle))
        .angle();

        Self {
            angle: robot_angle,
            avg_angle,
            lidar: robot.lidar.clone(),
            search_gradient: robot.search_gradient,
            closest_robot,
            group_center,
            group_spread,
        }
    }
}
