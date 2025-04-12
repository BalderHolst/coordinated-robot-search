use burn::backend::{Autodiff, Wgpu};
use clap::Parser;
use cli::Cli;

mod cli;
mod train;
mod world_gen;

type B = Wgpu;

fn main() -> Result<(), String> {
    let args = Cli::parse();

    match args.command {
        cli::Command::Train(args) => {
            train::run::<B, Autodiff<B>>(args)?;
        }
        cli::Command::WorldGen(args) => {
            world_gen::world_gen(args)?;
        }
    }

    Ok(())
}
