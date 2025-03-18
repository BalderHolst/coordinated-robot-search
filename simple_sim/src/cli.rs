use std::path::PathBuf;

use botbrain::{behaviors::Behavior, RobotPose};

use clap::{self, Args, Parser, Subcommand};
use serde::{Deserialize, Serialize};

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
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,

}

#[derive(Subcommand)]
pub enum Command {
    Run(RunArgs),
    Scenario(ScenarioArgs),
}

#[derive(Args, Clone)]
pub struct RunArgs {

    /// The world file to load
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

#[derive(Args)]
pub struct ScenarioArgs {

    /// The scenario file to load
    #[arg(short, long)]
    scenario: PathBuf,

    /// The output file to write the results to
    #[arg(short, long)]
    output: Option<PathBuf>,
}

#[derive(Serialize, Deserialize)]
#[serde(transparent)]
struct Robot(RobotPose);

#[derive(Serialize, Deserialize)]
struct Scenario {
    /// The world file to load
    world: PathBuf,

    /// What behavior to run on the robots
    behavior: Behavior,

    /// The duration of the scenario in seconds
    duration: f64,

    /// The robots to use in the scenario
    robots: Vec<Robot>,
}
