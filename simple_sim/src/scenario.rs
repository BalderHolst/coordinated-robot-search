use std::{io::Write, path::PathBuf};

use botbrain::{behaviors::Behavior, RobotPose};
use serde::{Deserialize, Serialize};

use crate::sim;

#[derive(Serialize)]
pub struct TrialData {
    states: Vec<sim::SimulatorState>,
    coverage: Vec<f32>,
}

impl TrialData {

    pub fn new() -> Self {
        Self {
            states: Vec::new(),
            coverage: Vec::new(),
        }
    }

    pub fn add_state(&mut self, state: sim::SimulatorState, coverage: f32) {
        self.states.push(state);
        self.coverage.push(coverage);
    }

    pub fn dump_to_file(&self, path: &PathBuf) -> Result<(), String> {
        let file = std::fs::File::create(path)
            .map_err(|e| format!("Failed to create file '{}': {}", path.display(), e))?;

        let wtr = std::io::BufWriter::new(file);

        serde_json::to_writer(wtr, self)
            .map_err(|e| format!("Failed to serialize data to JSON: {}", e))
    }
}

#[derive(Serialize, Deserialize)]
pub struct Scenario {
    /// The world file to load. Relative to the scenario file.
    pub world: PathBuf,

    /// What behavior to run on the robots
    pub behavior: Behavior,

    /// The duration of the scenario in seconds
    pub duration: f32,

    /// The robots to use in the scenario
    pub robots: Vec<RobotPose>,
}
