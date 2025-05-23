use std::{path::PathBuf, str::FromStr};

use botbrain::{
    behaviors::{rl::REACT_HZ, RobotKind},
    Vec2,
};
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
    WorldToImg(WorldToImgArgs),
    WorldToJson(WorldToJsonArgs),
    PlaceRobots(PlaceRobotsArgs),
}

#[derive(Args)]
pub struct TrainArgs {
    #[arg(short, long)]
    pub model: PathBuf,

    #[arg(short, long)]
    pub robot: RobotKind,

    #[arg(short, long)]
    pub world_dir: PathBuf,

    #[arg(long, default_value_t = (240.0 * REACT_HZ) as usize)]
    pub max_steps: usize,

    #[arg(long, default_value_t = 1000)]
    pub episodes: usize,

    #[arg(long, default_value_t = 10000.0)]
    pub eps_half: f64,

    #[arg(long, default_value_t = 0.05)]
    pub eps_end: f64,

    #[arg(long, default_value_t = 4)]
    pub max_robots: usize,
}

fn tuple_parser<T: FromStr>(s: &str) -> Result<(T, T), String> {
    let s = s.trim_start_matches('(').trim_end_matches(')');
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(format!("Expected two comma-separated values, got {}", s));
    }
    let x = parts[0]
        .parse::<T>()
        .map_err(|_| format!("Invalid value: {}", parts[0]))?;
    let y = parts[1]
        .parse::<T>()
        .map_err(|_| format!("Invalid value: {}", parts[1]))?;
    Ok((x, y))
}

fn vec2_parser(s: &str) -> Result<Vec2, String> {
    let (x, y) = tuple_parser(s)?;
    Ok(Vec2::new(x, y))
}

#[derive(Args)]
pub struct WorldGenArgs {
    /// Output directory
    #[arg(short, long)]
    pub output: PathBuf,

    /// Overwrite existing files
    #[arg(short, long)]
    pub force: bool,

    /// Number of worlds to generate
    #[arg(short, default_value_t = 1)]
    pub n: usize,

    /// Size of the worlds. Format: "x,y" or "(x,y)"
    #[arg(long, default_value = "(20.0,20.0)", value_parser = vec2_parser)]
    pub size: Vec2,

    /// Minimum number of obstacles in the world
    #[arg(long, default_value_t = 0.1)]
    pub scale: f32,

    /// Render the worlds to images in the supplied directory
    #[arg(short, long)]
    pub render: Option<PathBuf>,
}

#[derive(Args)]
pub struct WorldToImgArgs {
    /// Input file or directory
    #[arg(short, long)]
    pub input: PathBuf,

    /// Output file or directory
    #[arg(short, long)]
    pub output: PathBuf,

    /// Overwrite existing files
    #[arg(short, long)]
    pub force: bool,

    /// Theme to use for rendering
    #[arg(short, long, default_value = "light")]
    pub theme: simple_sim::gui::Theme,
}

#[derive(Args)]
pub struct PlaceRobotsArgs {
    /// World file to place robots in
    #[arg(short, long)]
    pub world: PathBuf,

    /// Number of robots to place
    #[arg(short, long)]
    pub n_robots: usize,

    /// Seed for random number generation
    #[arg(short, long)]
    pub seed: Option<u64>,
}

#[derive(Args)]
pub struct WorldToJsonArgs {
    /// Input file
    #[arg(short, long)]
    pub input: PathBuf,

    /// Output file
    #[arg(short, long)]
    pub output: PathBuf,

    /// Overwrite existing files
    #[arg(short, long)]
    pub force: bool,
}
