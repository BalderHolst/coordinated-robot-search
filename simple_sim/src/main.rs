use clap::Parser;

mod cli;
mod gui;
mod sim;
mod world;

fn main() -> Result<(), String> {
    let args = cli::Cli::parse();
    match args.command {
        cli::Command::Run(args) => gui::run_normally(args),
        cli::Command::Scenario(args) => todo!(),
    }
}
