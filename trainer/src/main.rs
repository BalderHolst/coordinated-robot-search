mod enviornment;
mod memory;

use std::io::Write;
use std::path::PathBuf;
use std::{fmt, fs};

use botbrain::behaviors::rl::model::Model;
use botbrain::behaviors::rl::state::{RlAction, RlState};
use botbrain::behaviors::{
    rl::{model::ModelConfig, REACT_HZ},
    Behavior,
};
use botbrain::burn::optim::{Adam, Optimizer};
use botbrain::burn::tensor::backend::AutodiffBackend;
use botbrain::{burn, Pos2, RobotPose};

use burn::backend::{Autodiff, Wgpu};
use burn::config::Config;
use burn::module::Module;
use burn::optim::adaptor::OptimizerAdaptor;
use burn::optim::{AdamConfig, GradientsParams};

use burn::tensor::{Int, Tensor};
use clap::Parser;
use enviornment::Enviornment;
use memory::Memory;
use serde::{Deserialize, Serialize};
use simple_sim::world::world_from_path;
use simple_sim::world::World;

type MyBackend = Autodiff<Wgpu>;

type SwarmState = Vec<RlState>;
type SwarmAction = Vec<RlAction>;

const EPISODES: usize = 200;
const MEMORY_SIZE: usize = 4096;
const MAX_STEPS: usize = (600.0 * REACT_HZ) as usize;
const EPS_DECAY: f64 = 0.97;
const EPS_START: f64 = 0.9;
const EPS_END: f64 = 0.05;

const UPDATE_TARGET_FREQ: usize = 100;

#[derive(Parser)]
struct Cli {
    #[arg(short, long)]
    model: PathBuf,
}

#[derive(Config)]
pub struct TrainingConfig {
    #[config(default = 32)]
    pub batch_size: usize,
    #[config(default = 0.01)]
    pub lr: f64,
    #[config(default = 0.95)]
    pub gamma: f64,
    #[config(default = 0.005)]
    pub tau: f64,

    pub model: ModelConfig,

    pub optimizer: AdamConfig,
}

fn main() {
    let args = Cli::parse();

    let world_path = PathBuf::from("../simple_sim/worlds/objectmap/small_empty.ron");
    let world = world_from_path(&world_path).unwrap();

    let model_config = ModelConfig::new();
    let optimizer_config = AdamConfig::new();
    let train_config = TrainingConfig::new(model_config, optimizer_config);

    // Setup the model
    let device = Default::default();

    let mut model = train_config.model.init::<MyBackend>(&device);

    // Load the model if it exists
    if args.model.exists() {
        println!("Loading model from '{}'", args.model.display());
        model = model
            .load_file(
                &args.model,
                &burn::record::DefaultRecorder::default(),
                &device,
            )
            .unwrap();
    }

    let poses = vec![RobotPose {
        pos: Pos2::new(0.0, 0.0),
        angle: 0.0,
    }];

    train(poses, world, model, train_config, EPISODES, args.model);
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct EpisodeStats {
    episode: usize,
    eps: f64,
    reward: f32,
    coverage: f32,
    steps: usize,
    sim_time: f32,
    avg_loss: f32,
}

impl fmt::Display for EpisodeStats {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "episode: {}, eps: {:.2}, reward: {:.3}, coverage: {:.2}%, steps: {}, sim_time: {:.0}s, avg_loss: {:.3}",
            self.episode,
            self.eps,
            self.reward,
            self.coverage * 100.0,
            self.steps,
            self.sim_time,
            self.avg_loss,
        )
    }
}

fn train(
    init_poses: Vec<RobotPose>,
    world: World,
    mut target_net: Model<MyBackend>,
    config: TrainingConfig,
    num_episodes: usize,
    model_file: PathBuf,
) {
    let stats_file = model_file.with_extension("json");

    let stem = model_file.file_stem().unwrap().to_str().unwrap();
    let config_file = model_file.with_file_name(format!("{}-config.json", stem));

    serde_json::to_writer(
        fs::File::create(config_file.clone()).unwrap(),
        &config,
    ).unwrap();

    let behavior = Behavior::parse("rl").unwrap().with_name("training");

    let mut env = Enviornment::new(world, behavior, init_poses, MAX_STEPS);

    println!("{}", target_net);

    let mut optimizer = config.optimizer.init::<MyBackend, Model<MyBackend>>();

    let mut memory = memory::Memory::<MEMORY_SIZE>::new();

    let mut policy_net = target_net.clone();

    let mut stats = Vec::new();

    let start_time = std::time::Instant::now();

    let mut eps = EPS_START;

    for episode in 0..num_episodes {
        let mut episode_done = false;
        let mut episode_reward: f32 = 0.0;

        let mut total_avg_loss: f32 = 0.0;

        let mut step: usize = 0;

        print!("[{}/{}]", episode + 1, num_episodes);
        _ = std::io::stdout().flush();

        while !episode_done {

            // Get environment state
            let states = env.states();

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
                total_avg_loss += train_model(
                    &target_net,
                    &mut policy_net,
                    &memory,
                    &mut optimizer,
                    &config,
                );
            }

            // Update target network
            if step % UPDATE_TARGET_FREQ == 0 {
                println!("========== UPDATE TARGET NETWORK ==========");
                target_net = policy_net.clone();
            }

            step += 1;

            if snapshot.done {
                let stat = EpisodeStats {
                    episode,
                    eps,
                    reward: episode_reward,
                    steps: step,
                    coverage: env.sim().state.diagnostics.coverage(),
                    sim_time: env.sim().state.time.as_secs_f32(),
                    avg_loss: total_avg_loss / step as f32,
                };

                env.reset();
                memory.clear();
                episode_done = true;

                println!(" [time: {:.0}s] {}", start_time.elapsed().as_secs(), stat);

                stats.push(stat);
            }
            {
                // State is incremented automatically by the simulator
            }

        }

        eps = f64::max(EPS_END, eps * EPS_DECAY);

        // Save stats
        match serde_json::to_string(&stats) {
            Ok(s) => {
                _ = fs::write(stats_file.clone(), s).map_err(|e| {
                    eprintln!("Failed to write stats: {}", e);
                });
            }
            Err(e) => {
                eprintln!("Failed to serialize stats: {}", e);
            }
        }

        // Save the model
        if let Err(e) = policy_net
            .clone()
            .save_file(&model_file, &burn::record::DefaultRecorder::default())
        {
            eprintln!("Failed to save model: {}", e);
        }
    }
}

fn train_model<B: AutodiffBackend, const CAP: usize>(
    target_model: &Model<B>,
    policy_model: &mut Model<B>,
    memory: &Memory<CAP>,
    optimizer: &mut OptimizerAdaptor<Adam, Model<B>, B>,
    config: &TrainingConfig,
) -> f32 {
    let (states, next_states, actions, rewards, dones) = memory.random_batch(config.batch_size);

    let mut loss_total: f32 = 0.0;
    let mut loss_count: usize = 0;

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

            let next_state_tensor = next_robot_state.to_tensor::<B>();
            let next_q_values = policy_model.forward(next_state_tensor).detach();
            let target = reward.clone() + next_q_values.max().mul_scalar(config.gamma);

            let q_values = target_model.forward(robot_state.to_tensor::<B>());
            let q_value = q_values.select(0, robot_action);
            let loss = (q_value - target).abs();

            loss_total += loss.clone().to_data().as_slice::<f32>().unwrap()[0];
            loss_count += 1;

            let grads = loss.backward();
            let grads2 = GradientsParams::from_grads(grads, policy_model);

            *policy_model = optimizer.step(config.lr, policy_model.clone(), grads2);
        }
    }

    loss_total / loss_count as f32
}
