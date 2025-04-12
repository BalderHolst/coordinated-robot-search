use std::path::PathBuf;

use botbrain::behaviors::{rl::REACT_HZ, RobotKind};
use clap::{Args, Parser, Subcommand};

#[derive(Parser)]
pub struct Cli {
    #[clap(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    Train(TrainArgs),
    WorldGen(WorldGenArgs),
}

#[derive(Args)]
pub struct TrainArgs {
    #[arg(short, long)]
    pub model: PathBuf,

    #[arg(short, long)]
    pub robot: RobotKind,

    #[arg(short, long)]
    pub world: PathBuf,

    #[arg(long, default_value_t = (600.0 * REACT_HZ) as usize)]
    pub max_steps: usize,

    #[arg(long, default_value_t = 1000)]
    pub episodes: usize,

    #[arg(long, default_value_t = 6000.0)]
    pub eps_half: f64,

    #[arg(long, default_value_t = 0.05)]
    pub eps_end: f64,

    #[arg(long, default_value_t = 4)]
    pub max_robots: usize,
}

#[derive(Args)]
pub struct WorldGenArgs {}
