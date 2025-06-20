use std::path::PathBuf;

use arrow_array::RecordBatch;
use botbrain::{behaviors::Behavior, RobotPose};
use serde::{Deserialize, Serialize};

use crate::{
    cli::{GlobArgs, ScenarioArgs},
    gui, sim,
    world::{desc_from_path, description::WorldDescription, world_from_path},
};

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
pub enum ScenarioWorld {
    Path(PathBuf),
    Desc(WorldDescription),
}

#[derive(Serialize, Deserialize)]
pub struct Scenario {
    /// The title of the scenario
    pub title: String,

    /// The world to load. Either path to world, or a description of the world
    pub world: ScenarioWorld,

    /// What behavior to run on the robots
    pub behavior: Behavior,

    /// The duration of the scenario in seconds
    pub duration: f32,

    /// The robots to use in the scenario
    pub robots: Vec<RobotPose>,
}

impl Scenario {
    pub fn embed_world(self) -> Result<Self, String> {
        let world_desc = match self.world {
            ScenarioWorld::Path(path) => desc_from_path(&path)?,
            ScenarioWorld::Desc(desc) => desc,
        };

        Ok(Self {
            world: ScenarioWorld::Desc(world_desc),
            ..self
        })
    }
}

pub fn convert_scenario_to_json(input: &PathBuf, output: &PathBuf) -> Result<(), String> {
    let ron_scenario = std::fs::read_to_string(input)
        .map_err(|e| format!("Failed to read scenario file '{}': {}", input.display(), e))?;

    let scenario = ron::de::from_str::<Scenario>(&ron_scenario)
        .map_err(|e| format!("Failed to parse scenario file '{}': {}", input.display(), e))?;

    let scenario = scenario
        .embed_world()
        .map_err(|e| format!("Error embedding world: {e}"))?;

    let json = serde_json::to_string_pretty(&scenario)
        .map_err(|e| format!("Failed to serialize to JSON: {}", e))?;
    std::fs::write(output, json).map_err(|e| format!("Failed to write JSON file: {}", e))?;
    Ok(())
}

pub fn run_scenario(args: GlobArgs, scenario_args: ScenarioArgs) -> Result<(), String> {
    let scenario_root = match scenario_args.scenario.filename() {
        "-" => std::env::current_dir()
            .map_err(|e| format!("Failed to get current directory: {}", e))?,
        path => PathBuf::from(path)
            .parent()
            .ok_or(format!(
                "Failed to get parent directory of scenario file '{}'",
                path
            ))?
            .to_path_buf(),
    };

    let mut scenario = match scenario_args.scenario.clone().contents() {
        Ok(s) if scenario_args.json => serde_json::from_str(&s)
            .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
        Ok(s) => ron::de::from_str::<Scenario>(&s)
            .map_err(|e| format!("Error deserializing scenario file: {e}"))?,
        Err(e) => Err(e.to_string())?,
    };

    if let Some(desc_path) = &scenario_args.description {
        scenario = scenario
            .embed_world()
            .map_err(|e| format!("Error reading world file: {e}"))?;

        let json = serde_json::to_string_pretty(&scenario).unwrap();
        std::fs::write(desc_path, json).map_err(|e| {
            format!(
                "Could not write description file to '{}': {}",
                desc_path.display(),
                e
            )
        })?;
    }

    let world = match &scenario.world {
        ScenarioWorld::Path(path) => {
            let world_path = scenario_root.join(path).canonicalize().map_err(|e| {
                format!(
                    "Failed to canonicalize world path '{}': {}",
                    path.display(),
                    e
                )
            })?;
            world_from_path(&world_path)?
        }
        ScenarioWorld::Desc(desc) => desc.clone().create(),
    };

    let mut sim = sim::Simulator::new(sim::SimArgs {
        world,
        behavior: scenario_args
            .behavior
            .clone()
            .unwrap_or(scenario.behavior.clone()),
        #[cfg(not(feature = "single-thread"))]
        threads: args.threads,
        #[cfg(not(feature = "single-thread"))]
        no_debug_soup: args.no_debug_soup,
    });

    for robot in &scenario.robots {
        sim.add_robot(robot.clone());
    }

    let mut data = match scenario_args.headless {
        false => gui::run_scenario(sim, scenario, args.clone())?,
        true => sim::run_scenario_headless(sim, scenario, scenario_args.clone()),
    };

    if let Some(out_path) = scenario_args.output {
        let out_path = PathBuf::from(out_path);
        if let Err(e) = save_batch(&mut data, &out_path) {
            eprintln!("Failed to save data to '{}': {}", out_path.display(), e);
        }
    }

    Ok(())
}

fn save_batch(batch: &mut RecordBatch, path: &PathBuf) -> Result<(), String> {
    let file = || {
        std::fs::File::create(path)
            .map_err(|e| format!("Failed to create file '{}': {}", path.display(), e))
    };

    match path.extension().and_then(|ext| ext.to_str()) {
        Some("ipc") => {
            let mut file = file()?;
            let mut writer = arrow_ipc::writer::FileWriter::try_new(&mut file, &batch.schema())
                .map_err(|e| {
                    format!(
                        "Failed to create IPC writer for file '{}': {}",
                        path.display(),
                        e
                    )
                })?;
            writer.write(batch).map_err(|e| {
                format!("Failed to write batch to file '{}': {}", path.display(), e)
            })?;
            writer.finish().map_err(|e| {
                format!(
                    "Failed to finish writing batch to file '{}': {}",
                    path.display(),
                    e
                )
            })
        }
        Some(other) => Err(format!("Unknown extension: '{}'", other)),
        None => Err(format!(
            "File '{}' has no extension. Could not determine what file to write.",
            path.display()
        )),
    }
}
