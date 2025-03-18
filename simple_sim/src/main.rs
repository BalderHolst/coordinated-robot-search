use clap::Parser;
use scenario::Scenario;
use world::world_from_path;

mod cli;
mod gui;
mod scenario;
mod sim;
mod world;

fn main() -> Result<(), String> {
    let args = cli::Cli::parse();
    match args.command {
        cli::Command::Run(args) => gui::run_interactive(args),
        cli::Command::Scenario(args) => {
            let scenario = match std::fs::read_to_string(&args.scenario) {
                Ok(s) => ron::de::from_str::<Scenario>(&s)
                    .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
                Err(e) => Err(e.to_string())?,
            };

            let world_path = args
                .scenario
                .parent()
                .ok_or("Scenario file has no parent directory")?
                .join(&scenario.world);

            let world = world_from_path(&world_path)?;

            match args.headless {
                true => gui::run_scenario(args, world, scenario)?,
                false => sim::run_headless(scenario, world, args.threads, args.print_interval),
            }

            Ok(())
        }
    }
}
