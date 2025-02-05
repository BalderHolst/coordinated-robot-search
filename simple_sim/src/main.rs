use clap::Parser;

mod gui;
mod sim;
mod grid;
mod cli;

fn main() {
    let args = cli::Args::parse();
    gui::run(args);
}
