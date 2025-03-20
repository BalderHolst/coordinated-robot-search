use std::path::PathBuf;

use botbrain::{behaviors::Behavior, RobotPose};
use serde::{Deserialize, Serialize};

use crate::world::description::ObjectDescription;

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
pub enum ScenarioWorld {
    Path(PathBuf),
    ObjDesc(ObjectDescription),
}

#[derive(Serialize, Deserialize)]
pub struct Scenario {
    /// The world to load. Either path to world, or a description of the world
    pub world: ScenarioWorld,

    /// What behavior to run on the robots
    pub behavior: Behavior,

    /// The duration of the scenario in seconds
    pub duration: f32,

    /// The robots to use in the scenario
    pub robots: Vec<RobotPose>,
}
