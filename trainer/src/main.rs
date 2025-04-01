mod enviornment;
mod memory;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use botbrain::behaviors::rl::model::{Model, ModelRef};
use botbrain::behaviors::rl::state::{RlAction, RlState};
use botbrain::behaviors::rl::{RlRobot, REACT_HZ};
use botbrain::behaviors::RobotKind;
use botbrain::behaviors::{rl::model::ModelConfig, Behavior};
use botbrain::burn::module::AutodiffModule;
use botbrain::burn::optim::{Adam, Optimizer};
use botbrain::burn::prelude::Backend;
use botbrain::burn::tensor;
use botbrain::burn::tensor::backend::AutodiffBackend;
use botbrain::{burn, Pos2, Robot, RobotPose};

use burn::backend::{Autodiff, Wgpu};
use burn::config::Config;
use burn::optim::adaptor::OptimizerAdaptor;
use burn::optim::{AdamConfig, GradientsParams};

use burn::tensor::{Int, Tensor};
use enviornment::Enviornment;
use memory::Memory;
use simple_sim::world::World;
use simple_sim::{
    sim::{SimArgs, Simulator},
    world::world_from_path,
};

type MyBackend = Autodiff<Wgpu>;

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
const MAX_STEPS: usize = (600.0 * REACT_HZ) as usize;
const EPS_DECAY: f64 = 1000.0;
const EPS_START: f64 = 0.9;
const EPS_END: f64 = 0.05;

fn train(
    init_poses: Vec<RobotPose>,
    world: World,
    target_net: Model<MyBackend>,
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

    let mut env = Enviornment::new(sim, MAX_STEPS);

    println!("{}", target_net);

    let mut optimizer = config.optimizer.init::<MyBackend, Model<MyBackend>>();

    let mut memory = memory::Memory::<MEMORY_SIZE>::new();

    let mut policy_net = target_net.clone();

    for episode in 0..num_episodes {
        let mut episode_done = false;
        let mut episode_reward: f32 = 0.0;
        let mut episode_duration = 0_usize;
        let mut states = env.states();

        let mut step: usize = 0;

        while !episode_done {
            let eps = EPS_END + (EPS_START - EPS_END) * f64::exp(-(step as f64) / EPS_DECAY);

            let actions = states
                .iter()
                .map(|state| target_net.react_with_exploration(state.clone(), eps))
                .collect::<Vec<_>>();

            let snapshot = env.step(actions.clone());

            episode_reward += snapshot.reward;

            memory.push(
                states.clone(),
                snapshot.state,
                actions,
                snapshot.reward.clone(),
                snapshot.done,
            );

            if config.batch_size < memory.len() {
                train_model(
                    &policy_net,
                    &mut target_net.clone(),
                    &memory,
                    &mut optimizer,
                    &config,
                );
            }

            step += 1;
            episode_duration += 1;

            if snapshot.done {
                env.reset();
                episode_done = true;

                println!(
                    "{{\"episode\": {}, \"reward\": {:.4}, \"duration\": {}}}",
                    episode, episode_reward, episode_duration
                );
            }
            {
                let sim = env.sim();
                println!("Time: {:.3}, Coverage: {:.3}", sim.state.time.as_secs_f32(), sim.state.diagnostics.coverage());
                // State is incremented automatically by the simulator
            }
        }
    }
}

fn train_model<B: AutodiffBackend, const CAP: usize>(
    target_model: &Model<B>,
    policy_model: &mut Model<B>,
    memory: &Memory<CAP>,
    optimizer: &mut OptimizerAdaptor<Adam, Model<B>, B>,
    config: &TrainingConfig,
) {
    let (states, next_states, actions, rewards, dones) = memory.random_batch(config.batch_size);

    // For memory in batch
    for i in 0..states.len() {
        let swarm_state = &states[i];
        let next_swarm_state = &next_states[i];
        let swarm_action = &actions[i];

        let reward = Tensor::<B, 1>::from([rewards[i].clone()]);
        let done = dones[i];

        if done {
            continue;
        }

        // For each robot
        for j in 0..swarm_state.len() {
            let robot_state = &swarm_state[j];
            let next_robot_state = &next_swarm_state[j];
            let robot_action = Tensor::<B, 1, Int>::from([usize::from(swarm_action[j])]);

            let next_state_tensor = next_robot_state.to_tensor::<20, B>();
            let next_q_values = policy_model.forward(next_state_tensor).detach();
            let target = reward.clone() + next_q_values.max().mul_scalar(config.gamma);

            let q_values = target_model.forward(robot_state.to_tensor::<20, B>());
            let q_value = q_values.select(0, robot_action);
            let loss = (q_value - target).abs();

            let grads = loss.backward();
            let grads2 = GradientsParams::from_grads(grads, policy_model);

            *policy_model = optimizer.step(config.lr, policy_model.clone(), grads2);
        }
    }
}
