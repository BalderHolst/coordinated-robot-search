use std::path::PathBuf;

use botbrain::{behaviors::Behavior, RobotPose};
use serde::{Deserialize, Serialize};

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
