use std::path::PathBuf;

use botbrain::behaviors::Behavior;

use clap::{self, Args, Parser, Subcommand};

use crate::gui::Theme;

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
    /// Theme to use for the UI
    #[arg(short, long, default_value = "dark")]
    pub theme: Theme,

    /// The scale of the UI
    #[arg(long, default_value = "1.0")]
    pub ui_scale: f32,

    // FIX: This does not work for some reason
    /// Target frames per second
    #[arg(long("fps"), default_value = "60")]
    pub target_fps: f32,

    /// Target simulation steps per second
    #[arg(long("sps"), default_value = "60")]
    pub target_sps: f32,

    /// Start the simulation paused
    #[arg(short, long)]
    pub paused: bool,

    /// Number of threads to use for simulation
    #[cfg(not(feature = "single-thread"))]
    #[arg(short('j'), long, default_value_t = default_threads())]
    pub threads: usize,

    /// Deactivate `DebugSoup` for all robots
    #[cfg(not(feature = "single-thread"))]
    #[arg(long, default_value = "false")]
    pub no_debug_soup: bool,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Clone, Subcommand)]
pub enum Command {
    Run(RunArgs),
    Scenario(ScenarioArgs),
    WorldToJson(WorldToJsonArgs),
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

#[derive(Args, Clone)]
pub struct WorldToJsonArgs {
    /// The world file to load (".ron" format)
    #[arg(index = 1)]
    pub input: PathBuf,

    /// The output file to write the json to
    #[arg(index = 2)]
    pub output: PathBuf,
}
