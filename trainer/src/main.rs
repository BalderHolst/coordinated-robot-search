mod enviornment;
mod memory;
mod utils;

use std::path::PathBuf;
use std::{fmt, fs};

use botbrain::behaviors::rl::model::Model;
use botbrain::behaviors::rl::state::{RlAction, RlState};
use botbrain::behaviors::rl::{model::ModelConfig, REACT_HZ};
use botbrain::burn::tensor::backend::AutodiffBackend;
use botbrain::burn;

use burn::backend::{Autodiff, Wgpu};
use burn::config::Config;
use burn::grad_clipping::GradientClippingConfig;
use burn::module::Module;
use burn::nn::loss::{MseLoss, Reduction};
use burn::optim::adaptor::OptimizerAdaptor;
use burn::optim::{AdamW, AdamWConfig};

use clap::Parser;
use enviornment::Enviornment;
use memory::{get_batch, sample_indices, Memory};
use serde::{Deserialize, Serialize};
use simple_sim::world::world_from_path;
use simple_sim::world::World;
use utils::{
    ref_to_action_tensor, ref_to_not_done_tensor, ref_to_reward_tensor, ref_to_state_tensor,
    update_parameters,
};

type MyBackend = Autodiff<Wgpu>;

type SwarmState = Vec<RlState>;
type SwarmAction = Vec<RlAction>;

const EPISODES: usize = 1000;
const MEMORY_SIZE: usize = 4096;
const MAX_STEPS: usize = (600.0 * REACT_HZ) as usize;

const EPS_HALF: f64 = 6000.0;
const EPS_END: f64 = 0.10;

#[derive(Parser)]
struct Cli {
    #[arg(short, long)]
    model: PathBuf,
}

#[derive(Config)]
pub struct TrainingConfig {
    #[config(default = 32)]
    pub batch_size: usize,
    #[config(default = 0.001)]
    pub lr: f64,
    #[config(default = 0.995)]
    pub gamma: f64,
    #[config(default = 0.005)]
    pub tau: f64,

    pub clip_grad: Option<GradientClippingConfig>,

    pub model: ModelConfig,
}

fn main() {
    let args = Cli::parse();

    let world_path = PathBuf::from("../simple_sim/worlds/objectmap/small_simple.ron");
    let world = world_from_path(&world_path).unwrap();

    let model_config = ModelConfig::new();
    let train_config = TrainingConfig::new(model_config)
        .with_clip_grad(Some(GradientClippingConfig::Value(100.0)));

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

    train(1, world, model, train_config, EPISODES, args.model);
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
    actions: Vec<usize>,
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

fn train<B: AutodiffBackend>(
    robots: usize,
    world: World,
    model: Model<B>,
    config: TrainingConfig,
    num_episodes: usize,
    model_file: PathBuf,
) {
    let stats_file = model_file.with_extension("json");
    let mut stats = Vec::new();

    let mut env = Enviornment::new(world, robots);

    let mut step: usize = 0;

    let mut target_net = model.clone();
    let mut policy_net = model.clone();

    let mut memory = Memory::<B, MEMORY_SIZE>::default();

    let mut optimizer = AdamWConfig::new()
        .with_grad_clipping(config.clip_grad.clone())
        .init();

    for episode in 0..num_episodes {
        let mut episode_done = false;
        let mut episode_reward: f32 = 0.0;
        let mut episode_duration = 0_usize;
        let mut state = env.states()[0].clone();

        let mut total_loss: f32 = 0.0;
        let mut action_stats = vec![0; RlAction::SIZE];

        let mut eps = 0.0;
        let eps_base = f64::powf(2.0, 1.0 / EPS_HALF);

        while !episode_done {

            eps = (1.0 - EPS_END) * f64::powf(eps_base, -(step as f64)) + EPS_END;

            let action = policy_net.react_with_exploration(&state, eps);
            action_stats[usize::from(action)] += 1;

            let snapshot = env.step(vec![action]);

            episode_reward += snapshot.reward;

            memory.push(
                state.clone(),
                snapshot.state[0].clone(),
                action,
                snapshot.reward.clone(),
                snapshot.done,
            );

            if config.batch_size < memory.len() {
                let loss;
                (policy_net, target_net, loss) =
                    train_agent(policy_net, target_net, &mut memory, &mut optimizer, &config);
                total_loss += loss;
            }

            episode_duration += 1;

            if snapshot.done || episode_duration >= MAX_STEPS {
                episode_done = true;
            } else {
                state = snapshot.state[0].clone();
            }

            step += 1;
        }

        let stat = EpisodeStats {
            episode,
            eps,
            reward: episode_reward,
            coverage: env.sim().state.diagnostics.coverage(),
            steps: episode_duration,
            sim_time: env.sim().state.time.as_secs_f32(),
            avg_loss: total_loss / episode_duration as f32,
            actions: action_stats,
        };
        println!("{stat}");

        stats.push(stat);

        env.reset();

        // Save model
        if let Err(e) = policy_net
            .clone()
            .save_file(&model_file, &burn::record::DefaultRecorder::default())
        {
            eprintln!("Failed to save model: {}", e);
        }

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
    }
}

fn train_agent<B: AutodiffBackend, const CAP: usize>(
    mut policy_net: Model<B>,
    mut target_net: Model<B>,
    memory: &Memory<B, CAP>,
    optimizer: &mut OptimizerAdaptor<AdamW, Model<B>, B>,
    config: &TrainingConfig,
) -> (Model<B>, Model<B>, f32) {
    let sample_indices = sample_indices((0..memory.len()).collect(), config.batch_size);
    let state_batch = get_batch(memory.states(), &sample_indices, ref_to_state_tensor);
    let action_batch = get_batch(memory.actions(), &sample_indices, ref_to_action_tensor);
    let state_action_values = policy_net.forward(state_batch).gather(1, action_batch);

    let next_state_batch = get_batch(memory.next_states(), &sample_indices, ref_to_state_tensor);
    let next_state_values = target_net.forward(next_state_batch).max_dim(1).detach();

    let not_done_batch = get_batch(memory.dones(), &sample_indices, ref_to_not_done_tensor);
    let reward_batch = get_batch(memory.rewards(), &sample_indices, ref_to_reward_tensor);

    let expected_state_action_values =
        (next_state_values * not_done_batch).mul_scalar(config.gamma) + reward_batch;

    let loss = MseLoss.forward(
        state_action_values,
        expected_state_action_values,
        Reduction::Mean,
    );

    policy_net = update_parameters(loss.clone(), policy_net, optimizer, config.lr);

    target_net = Model::soft_update(target_net, &policy_net, config.tau);

    let loss = loss.to_data().as_slice::<f32>().unwrap()[0];

    (policy_net, target_net, loss)
}
