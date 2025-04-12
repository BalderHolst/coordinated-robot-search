use burn::backend::{Autodiff, Wgpu};
use clap::Parser;
use cli::Cli;

mod cli;
mod train;

type B = Wgpu;

fn main() -> Result<(), String> {
    let args = Cli::parse();

    match args.command {
        cli::Command::Train(args) => {
            train::run::<B, Autodiff<B>>(args)?;
        }
        cli::Command::WorldGen(_args) => {
            println!("WorldGen command is not implemented yet.");
        }
    }

    Ok(())
}
