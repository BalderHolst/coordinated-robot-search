use clap::Parser;

mod cli;
mod gui;
mod sim;
mod world;

fn main() {
    let args = cli::Args::parse();
    gui::run(args);
}
