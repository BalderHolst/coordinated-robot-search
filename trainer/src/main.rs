mod enviornment;
mod memory;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use botbrain::behaviors::rl::model::{Model, ModelRef};
use botbrain::behaviors::rl::state::{RlAction, RlState};
use botbrain::behaviors::rl::RlRobot;
use botbrain::behaviors::RobotKind;
use botbrain::behaviors::{rl::model::ModelConfig, Behavior};
use botbrain::burn::optim::Adam;
use botbrain::burn::prelude::Backend;
use botbrain::{burn, Pos2, Robot, RobotPose};

use burn::backend::Wgpu;
use burn::config::Config;
use burn::optim::AdamConfig;

use enviornment::Enviornment;
use memory::Memory;
use simple_sim::world::World;
use simple_sim::{
    sim::{SimArgs, Simulator},
    world::world_from_path,
};

type MyBackend = Wgpu;

type SwarmState = Vec<RlState>;
type SwarmAction = Vec<RlAction>;

#[derive(Config)]
pub struct TrainingConfig {
    #[config(default = 32)]
    pub batch_size: usize,
    #[config(default = 42)]
    pub seed: u64,
    #[config(default = 0.001)]
    pub lr: f64,
    #[config(default = 0.999)]
    pub gamma: f64,
    #[config(default = 0.005)]
    pub tau: f64,

    pub model: ModelConfig,

    pub optimizer: AdamConfig,
}

fn main() {
    let world_path = PathBuf::from("../simple_sim/worlds/objectmap/medium_empty.ron");
    let world = world_from_path(&world_path).unwrap();

    let model_config = ModelConfig::new();
    let optimizer_config = AdamConfig::new();
    let train_config = TrainingConfig::new(model_config, optimizer_config);

    // Setup the model
    let device = Default::default();
    let model = train_config.model.init::<MyBackend>(&device);

    let poses = vec![RobotPose {
        pos: Pos2::new(0.0, 0.0),
        angle: 0.0,
    }];

    train(poses, world, model, train_config, 100);
}

const MEMORY_SIZE: usize = 4096;
const MAX_STEPS: usize = 5000;
const EPS_DECAY: f64 = 1000.0;
const EPS_START: f64 = 0.9;
const EPS_END: f64 = 0.05;

fn train(
    init_poses: Vec<RobotPose>,
    world: World,
    model: Model<MyBackend>,
    config: TrainingConfig,
    num_episodes: usize,
) {
    let behavior = Behavior::parse("rl").unwrap().with_name("training");

    let robots = init_poses
        .into_iter()
        .map(|pose| {
            let robot = RlRobot::new_controlled();
            let robot = Box::new(robot) as Box<dyn Robot>;
            (pose, robot)
        })
        .collect();

    let sim = Simulator::with_robots(SimArgs { world, behavior }, robots);

    let env = Enviornment::new(sim, MAX_STEPS);

    println!("{}", model);

    let mut optimizer = config.optimizer.init();

    let mut memory = memory::Memory::<MEMORY_SIZE>::new();

    let mut policy_net = model.clone();

    for episode in 0..num_episodes {
        let mut episode_done = false;
        let mut episode_reward: f32 = 0.0;
        let mut episode_duration = 0_usize;
        let mut states = env.states();

        let step: usize = 0;

        while !episode_done {
            let eps = EPS_END + (EPS_START - EPS_END) * f64::exp(-(step as f64) / EPS_DECAY);

            let actions = states
                .iter()
                .map(|state| model.react_with_exploration(*state, eps))
                .collect::<Vec<_>>();

            let snapshot = env.step(actions);

            episode_reward += snapshot.reward;

            memory.push(
                states,
                snapshot.state,
                actions,
                snapshot.reward.clone(),
                snapshot.done,
            );

            if config.batch_size < memory.len() {
                policy_net =
                    agent.train::<MEMORY_SIZE>(policy_net, &memory, &mut optimizer, &config);
            }

            step += 1;
            episode_duration += 1;

            if snapshot.done() || episode_duration >= E::MAX_STEPS {
                env.reset();
                episode_done = true;

                println!(
                    "{{\"episode\": {}, \"reward\": {:.4}, \"duration\": {}}}",
                    episode, episode_reward, episode_duration
                );
            } else {
                states = *snapshot.state();
            }
        }
    }
}

fn train_model<B: Backend, const CAP: usize>(
    target_model: Model<B>,
    policy_model: Model<B>,
    memory: &Memory<CAP>,
    optimizer: Adam,
    config: TrainingConfig,
) -> Model<B> {

    let batch = memory .random_batch(config.batch_size);

    let state_action_values = batch
        .iter()
        .map(|(state, next_state, action, reward, done)| {
        })
        .collect::<Vec<_>>();



    // let (states, next_states, actions, rewards, dones) = batch.into_iter().cloned().unzip();

    todo!()
}
