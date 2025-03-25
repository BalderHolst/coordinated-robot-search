use std::path::PathBuf;

use clap::Parser;
use scenario::Scenario;
use world::{description::WorldDescription, world_from_path};

mod cli;
mod gui;
mod scenario;
mod sim;
mod world;

fn main() -> Result<(), String> {
    let args = cli::GlobArgs::parse();
    match args.command.clone() {
        cli::Command::Run(run_args) => gui::run_interactive(args, run_args.clone()),
        cli::Command::Scenario(scenario_args) => {
            let scenario = match scenario_args.scenario.clone().contents() {
                Ok(s) if scenario_args.json => serde_json::from_str(&s)
                    .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
                Ok(s) => ron::de::from_str::<Scenario>(&s)
                    .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
                Err(e) => Err(e.to_string())?,
            };

            if let Some(desc_path) = &scenario_args.description {
                let json = serde_json::to_string_pretty(&scenario).unwrap();
                std::fs::write(desc_path, json)
                    .map_err(|e| format!("Could not write description file: {e}"))?;
            }

            let world = match &scenario.world {
                scenario::ScenarioWorld::Path(path) => world_from_path(path)?,
                scenario::ScenarioWorld::ObjDesc(desc) => {
                    WorldDescription::Objs(desc.clone()).create()
                }
            };

            let mut sim = sim::Simulator::new(sim::SimArgs {
                world,
                behavior: scenario_args
                    .behavior
                    .clone()
                    .unwrap_or(scenario.behavior.clone()),
                threads: args.threads,
            });

            for robot in &scenario.robots {
                sim.add_robot(robot.clone());
            }

            let mut data = match scenario_args.headless {
                false => gui::run_scenario(sim, scenario, args.clone())?,
                true => sim::run_scenario_headless(sim, scenario, scenario_args.clone()),
            };

            if let Some(out_path) = scenario_args.output {
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

        match path.extension().and_then(|ext| ext.to_str()) {
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
