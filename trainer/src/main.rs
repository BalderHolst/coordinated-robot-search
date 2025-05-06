use burn::backend::{Autodiff, Wgpu};
use clap::Parser;
use cli::Cli;

mod cli;
mod place_robots;
mod train;
mod world_gen;
mod world_to_img;

type B = Wgpu;

fn main() -> Result<(), String> {
    let args = Cli::parse();

    match args.command {
        cli::Command::Train(args) => train::run::<B, Autodiff<B>>(args),
        cli::Command::WorldGen(args) => world_gen::world_gen(args),
        cli::Command::WorldToImg(args) => world_to_img::world_to_img(args),
        cli::Command::PlaceRobots(args) => place_robots::place_robots(&args),
        cli::Command::WorldToJson(args) => {
            let input = std::fs::read_to_string(&args.input)
                .map_err(|e| format!("Failed to read file {}: {}", args.input.display(), e))?;
            let desc = ron::from_str::<simple_sim::world::description::ObjectDescription>(&input)
                .map_err(|e| {
                format!("Failed to parse RON file {}: {}", args.input.display(), e)
            })?;
            let output = serde_json::to_string_pretty(
                &simple_sim::world::description::WorldDescription::Objs(desc),
            )
            .map_err(|e| format!("Failed to serialize world to JSON: {}", e))?;
            std::fs::write(&args.output, output)
                .map_err(|e| format!("Failed to write JSON file {}: {}", args.output.display(), e))
        }
    }
}
