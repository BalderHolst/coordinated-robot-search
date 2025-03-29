use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use botbrain::behaviors::rl::model::Model;
use botbrain::behaviors::rl::RlRobot;
use botbrain::behaviors::BehaviorOutput;
use botbrain::behaviors::{rl::model::ModelConfig, Behavior};
use botbrain::{burn, Robot};

use burn::backend::Wgpu;
use burn::config::Config;
use burn::optim::AdamConfig;

use simple_sim::{
    cli::default_threads,
    sim::{SimArgs, Simulator},
    world::world_from_path,
};

type MyBackend = Wgpu;

#[derive(Config)]
pub struct TrainingConfig {
    #[config(default = 64)]
    pub batch_size: usize,
    #[config(default = 4)]
    pub num_workers: usize,
    #[config(default = 42)]
    pub seed: u64,
    #[config(default = 1e-4)]
    pub lr: f64,

    pub model: ModelConfig,

    optimizer: AdamConfig,
}

fn main() {
    let world_path = PathBuf::from("../simple_sim/worlds/objectmap/medium_empty.ron");
    let world = world_from_path(&world_path).unwrap();

    let model_config = ModelConfig::new();
    let optimizer = AdamConfig::new();
    let config = TrainingConfig::new(model_config, optimizer);

    let behavior = Behavior::parse("rl").unwrap();

    let threads = default_threads();
    let sim = Simulator::new(SimArgs {
        world,
        behavior,
        threads,
    });

    // Setup the model
    let device = Default::default();
    let model = ModelConfig::new().init::<MyBackend>(&device);

    println!("{}", model);
}
