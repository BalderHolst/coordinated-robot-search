use clap::Parser;

mod cli;
mod gui;
mod scenario;
mod sim;
mod utils;
mod world;

fn main() -> Result<(), String> {
    let args = cli::GlobArgs::parse();
    match args.command.clone() {
        cli::Command::Run(run_args) => gui::run_interactive(args, run_args.clone()),
        cli::Command::Scenario(scenario_args) => {
            scenario::run_scenario(args, scenario_args.clone())
        }
        cli::Command::WorldToJson(world_args) => {
            scenario::convert_scenario_to_json(&world_args.input, &world_args.output)
        }
    }
}
