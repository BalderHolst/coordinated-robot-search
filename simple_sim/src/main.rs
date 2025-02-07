use clap::Parser;

mod gui;
mod sim;
mod grid;
mod cli;
mod world;

fn main() {
    let args = cli::Args::parse();
    gui::run(args);
}
