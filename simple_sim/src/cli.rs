use std::path::PathBuf;

use botbrain::behaviors::Behavior;

use clap::{self, Parser};

fn default_threads() -> usize {
    match std::thread::available_parallelism() {
        Ok(n) => n.into(),
        Err(e) => {
            eprintln!("Error getting number of threads: {e}. Defaulting to 1.");
            1
        }
    }
}

#[derive(Parser)]
pub struct Args {
    #[arg(index = 1)]
    pub world: PathBuf,

    /// What behavior to run on the robots
    #[arg(index = 2, value_parser = clap::value_parser!(Behavior))]
    pub behavior: Behavior,

    /// Start the simulation paused
    #[arg(short, long)]
    pub paused: bool,

    /// Number of threads to use for simulation
    #[arg(short('j'), long, default_value_t = default_threads())]
    pub threads: usize,

    // TODO: This does not work for some reason
    /// Target frames per second
    #[arg(long("fps"), default_value = "60")]
    pub target_fps: f32,

    /// Target simulation steps per second
    #[arg(long("sps"), default_value = "60")]
    pub target_sps: f32,
}
