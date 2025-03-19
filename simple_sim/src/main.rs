use std::path::PathBuf;

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
            let scenario = match args.scenario.clone().contents() {
                Ok(s) => ron::de::from_str::<Scenario>(&s)
                    .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
                Err(e) => Err(e.to_string())?,
            };

            let root = match args.scenario.is_file() {
                true => PathBuf::from(args.scenario.filename())
                    .parent()
                    .ok_or("Scenario file has no parent directory".to_string())?
                    .to_path_buf(),
                false => PathBuf::from(std::env::current_dir().map_err(|e| e.to_string())?),
            };

            let world_path = root.join(&scenario.world);

            let world = world_from_path(&world_path)?;

            let mut sim = sim::Simulator::new(sim::SimArgs {
                world,
                behavior: scenario.behavior.clone(),
                threads: args.threads,
            });

            for robot in &scenario.robots {
                sim.add_robot(robot.clone());
            }

            let mut data = match args.headless {
                false => gui::run_scenario(sim, scenario, args.clone())?,
                true => sim::run_scenario_headless(sim, scenario, args.print_interval),
            };

            if let Some(out_path) = args.output {
                for out_path in out_path.split(':') {
                    let out_path = PathBuf::from(out_path);
                    if let Err(e) = utils::save_df(&mut data, &out_path) {
                        eprintln!("Failed to save data to '{}': {}", out_path.display(), e);
                    }
                }
            }

            Ok(())
        }
    }
}

mod utils {
    use polars::prelude::*;
    use std::path::PathBuf;

    pub fn save_df(df: &mut DataFrame, path: &PathBuf) -> Result<(), String> {
        let file = || {
            std::fs::File::create(path)
                .map_err(|e| format!("Failed to create file '{}': {}", path.display(), e))
        };

        match path.extension().map(|ext| ext.to_str()).flatten() {
            Some("json") => JsonWriter::new(file()?)
                .finish(df)
                .map_err(|e| format!("Failed to serialize data to JSON: {}", e)),
            Some("csv") => CsvWriter::new(file()?)
                .finish(df)
                .map_err(|e| format!("Failed to serialize data to CSV: {}", e)),
            Some("ipc") => IpcWriter::new(file()?)
                .finish(df)
                .map_err(|e| format!("Failed to serialize data to IPC: {}", e)),
            Some("parquet") => match ParquetWriter::new(file()?).finish(df) {
                Ok(_) => Ok(()),
                Err(e) => Err(format!("Failed to serialize data to Parquet: {}", e)),
            },
            Some(other) => Err(format!("Unknown extension: '{}'", other)),
            None => Err(format!(
                "File '{}' has no extension. Could not determine what file to write.",
                path.display()
            )),
        }
    }
}
