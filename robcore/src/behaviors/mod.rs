//! This module contains the robot behaviors.

mod avoid_obstacles;
pub mod search;

use std::time::Instant;

#[cfg(feature = "cli")]
use clap;

use super::*;

pub type BehaviorFn = fn(&mut Robot, Instant) -> Control;

#[cfg(feature = "cli")]
#[derive(clap::ValueEnum, Clone, Debug)]
pub enum Behavior {
    Nothing,
    Circle,
    AvoidObstacles,
    Search,
}

#[cfg(feature = "cli")]
impl Behavior {
    pub fn get_fn(&self) -> BehaviorFn {
        match self {
            Self::Nothing => nothing,
            Self::Circle => circle,
            Self::AvoidObstacles => avoid_obstacles::avoid_obstacles,
            Self::Search => search::search,
        }
    }
}

/// Move in a circle.
pub fn circle(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 1.0,
        steer: 0.5,
    }
}

/// Do nothing.
pub fn nothing(_robot: &mut Robot, _time: Instant) -> Control {
    Control {
        speed: 0.0,
        steer: 0.0,
    }
}
