use std::path::PathBuf;

use botbrain::{behaviors::Behavior, RobotPose};
use serde::{Deserialize, Serialize};

use polars::prelude::*;

use crate::sim;

#[derive(Serialize)]
pub struct TrialData {
    states: Vec<sim::SimulatorState>,
    coverage: Vec<f32>,
}

#[derive(Clone, Default)]
struct RobotData {
    x: Vec<f32>,
    y: Vec<f32>,
    angle: Vec<f32>,
    vel: Vec<f32>,
    avel: Vec<f32>,
    steer: Vec<f32>,
    speed: Vec<f32>,
}

impl RobotData {
    fn push(&mut self, pose: &RobotPose, vel: f32, avel: f32, steer: f32, speed: f32) {
        self.x.push(pose.pos.x);
        self.y.push(pose.pos.y);
        self.angle.push(pose.angle);
        self.vel.push(vel);
        self.avel.push(avel);
        self.steer.push(steer);
        self.speed.push(speed);
    }
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

    fn to_df(&self) -> DataFrame {
        let robot_count = self.states.get(0).map_or(0, |s| s.agents.len());

        assert_eq!(self.states.len(), self.coverage.len());

        let mut time = Vec::with_capacity(self.states.len());
        let mut robot_data = vec![RobotData::default(); robot_count];

        for state in &self.states {
            time.push(state.time.as_secs_f64());
            for (n, agent) in state.agents.iter().enumerate() {
                if let Some(data) = robot_data.get_mut(n) {
                    data.push(
                        &agent.state.pose,
                        agent.state.vel,
                        agent.state.avel,
                        agent.control.steer,
                        agent.control.speed,
                    );
                }
            }
        }

        assert_eq!(self.states.len(), time.len());

        let robot_cols = robot_data
            .iter()
            .enumerate()
            .flat_map(|(n, data)| {
                let prefix = format!("robot_{}", n);
                vec![
                    Column::new(format!("{}/x", prefix).into(), data.x.clone()),
                    Column::new(format!("{}/y", prefix).into(), data.y.clone()),
                    Column::new(format!("{}/angle", prefix).into(), data.angle.clone()),
                    Column::new(format!("{}/vel", prefix).into(), data.vel.clone()),
                    Column::new(format!("{}/avel", prefix).into(), data.avel.clone()),
                    Column::new(format!("{}/steer", prefix).into(), data.steer.clone()),
                    Column::new(format!("{}/speed", prefix).into(), data.speed.clone()),
                ]
            })
            .collect::<Vec<_>>();

        let cols = vec![
            Column::new("time".into(), time),
            Column::new("coverage".into(), self.coverage.clone()),
        ]
        .into_iter()
        .chain(robot_cols.into_iter())
        .collect();

        DataFrame::new(cols).expect("Failed to create DataFrame")
    }

    pub fn dump_to_file(&self, path: &PathBuf) -> Result<(), String> {
        let file = || {
            std::fs::File::create(path)
                .map_err(|e| format!("Failed to create file '{}': {}", path.display(), e))
        };

        match path.extension().map(|ext| ext.to_str()).flatten() {
            Some("json") => serde_json::to_writer(std::io::BufWriter::new(file()?), self)
                .map_err(|e| format!("Failed to serialize data to JSON: {}", e)),
            Some("csv") => CsvWriter::new(file()?)
                .finish(&mut self.to_df())
                .map_err(|e| format!("Failed to serialize data to CSV: {}", e)),
            Some("ipc") => IpcWriter::new(file()?)
                .finish(&mut self.to_df())
                .map_err(|e| format!("Failed to serialize data to IPC: {}", e)),
            Some("parquet") => match ParquetWriter::new(file()?).finish(&mut self.to_df()) {
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
