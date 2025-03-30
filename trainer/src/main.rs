use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use botbrain::behaviors::rl::RlRobot;
use botbrain::behaviors::{rl::model::ModelConfig, Behavior};
use botbrain::{burn, Pos2, Robot, RobotPose};

use burn::backend::Wgpu;
use burn::config::Config;
use burn::optim::AdamConfig;

use simple_sim::{
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
    let train_config = TrainingConfig::new(model_config, optimizer);

    let behavior = Behavior::parse("rl").unwrap();

    // Setup the model
    let device = Default::default();
    let model = train_config.model.init::<MyBackend>(&device);
    let model_ref = Rc::new(RefCell::new(model));

    let poses = vec![RobotPose {
        pos: Pos2::new(0.0, 0.0),
        angle: 0.0,
    }];

    let robots = poses
        .into_iter()
        .map(|pose| {
            let robot = RlRobot::from_model(model_ref.clone());
            let robot = Box::new(robot) as Box<dyn Robot>;
            (pose, robot)
        })
        .collect();

    let sim = Simulator::with_robots(SimArgs { world, behavior }, robots);

    println!("{}", model_ref.borrow());
}
