//! This module contains the robot behaviors.

mod avoid_obstacles;
pub mod circle;
mod nothing;
pub mod search;

use avoid_obstacles::AvoidObstaclesRobot;
use circle::CircleRobot;
#[cfg(feature = "cli")]
use clap;
use nothing::NothingRobot;
use search::SearchRobot;

use super::*;

#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[derive(Clone, Debug)]
pub enum Behavior {
    Nothing,
    Circle,
    AvoidObstacles,
    Search,
}

impl Behavior {
    pub fn create_robot(&self) -> Box<dyn Robot> {
        match self {
            Behavior::Nothing => Box::new(NothingRobot::default()),
            Behavior::Circle => Box::new(CircleRobot::default()),
            Behavior::AvoidObstacles => Box::new(AvoidObstaclesRobot::default()),
            Behavior::Search => Box::new(SearchRobot::default()),
        }
    }
}
