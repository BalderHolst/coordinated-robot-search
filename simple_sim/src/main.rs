#![allow(dead_code)] // TODO: Remove this

use clap::Parser;

mod cli;
mod grid;
mod gui;
mod sim;
mod world;

fn main() {
    let args = cli::Args::parse();
    gui::run(args);
}
