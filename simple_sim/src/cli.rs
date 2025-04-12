use std::path::PathBuf;

use botbrain::behaviors::Behavior;

use clap::{self, Args, Parser, Subcommand};

#[cfg(not(feature = "single-thread"))]
pub fn default_threads() -> usize {
    match std::thread::available_parallelism() {
        Ok(n) => n.into(),
        Err(e) => {
            eprintln!("Error getting number of threads: {e}. Defaulting to 1.");
            1
        }
    }
}

#[derive(Clone, Parser)]
pub struct GlobArgs {
    // TODO: This does not work for some reason
    /// Target frames per second
    #[arg(long("fps"), default_value = "60")]
    pub target_fps: f32,

    /// Target simulation steps per second
    #[arg(long("sps"), default_value = "60")]
    pub target_sps: f32,

    /// Number of threads to use for simulation
    #[cfg(not(feature = "single-thread"))]
    #[arg(short('j'), long, default_value_t = default_threads())]
    pub threads: usize,

    /// Start the simulation paused
    #[arg(short, long)]
    pub paused: bool,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Clone, Subcommand)]
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
}

#[derive(Args, Clone)]
pub struct ScenarioArgs {
    /// The scenario file to load
    #[arg(index = 1)]
    pub scenario: clap_stdin::FileOrStdin,

    /// Parse the scenario as json
    #[arg(long)]
    pub json: bool,

    /// Overide the behavior in the scenario file
    #[arg(short, long, value_parser = clap::value_parser!(Behavior))]
    pub behavior: Option<Behavior>,

    /// Generate a json file describing the scenario
    #[arg(short, long)]
    pub description: Option<PathBuf>,

    /// The output file to write the results to. Specify multiple paths by separating them with a ':'.
    #[arg(short, long)]
    pub output: Option<String>,

    /// Run the scenario headless
    #[arg(long)]
    pub headless: bool,

    /// How often to print the simulation time (in seconds)
    #[arg(short('p'), long, default_value = "1.0")]
    pub print_interval: f64,
}
