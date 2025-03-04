use std::{path::PathBuf, time::Instant};

use clap::{self, Parser};

pub type BehaviorFn = fn(&mut robcore::Robot, Instant) -> robcore::Control;

#[derive(Parser)]
pub struct Args {
    #[arg(index = 1)]
    pub world: PathBuf,

    /// What behavior to run on the robots
    #[arg(index = 2)]
    pub behavior: Behavior,

    /// Start the simulation paused
    #[arg(short, long)]
    pub paused: bool,

    /// Number of threads to use for simulation
    #[arg(short('j'), long, default_value = "4")]
    pub threads: usize,

    // TODO: This does not work for some reason
    /// Target frames per second
    #[arg(long("fps"), default_value = "60")]
    pub target_fps: f32,

    /// Target simulation steps per second
    #[arg(long("sps"), default_value = "60")]
    pub target_sps: f32,
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
