use std::time::Instant;

use clap::{self, Parser};

pub type BehaviorFn = fn(&mut robcore::Robot, Instant) -> robcore::Control;

#[derive(Parser)]
pub struct Args {
    #[arg(index = 1)]
    pub behavior: Behavior,

    #[arg(short, long)]
    pub paused: bool,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum Behavior {
    Circle,
    AvoidObstacles,
    Search,
}

impl Behavior {
    pub fn get_fn(&self) -> BehaviorFn {
        match self {
            Self::Circle => robcore::behaviors::circle,
            Self::AvoidObstacles => robcore::behaviors::avoid_obstacles,
            Self::Search => robcore::behaviors::search,
        }
    }
}
