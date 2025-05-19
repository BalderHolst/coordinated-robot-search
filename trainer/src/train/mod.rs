mod enviornment;
mod memory;
mod utils;

use std::marker::PhantomData;
use std::{fmt, fs};

use botbrain::behaviors::rl;
use botbrain::behaviors::rl::robots::medium_polar::MediumPolarRlRobot;
use botbrain::behaviors::rl::robots::small_polar::SmallPolarRlRobot;
use botbrain::{
    behaviors::{
        rl::{
            action::Action,
            network::{AutodiffNetwork, Network},
            robots::{minimal::MinimalRlRobot, small::SmallRlRobot},
            state::State,
            RlRobot,
        },
        Behavior, RobotKind,
    },
    burn,
};

use burn::record::BinFileRecorder;
use burn::{
    config::Config,
    grad_clipping::GradientClippingConfig,
    nn::loss::{MseLoss, Reduction},
    optim::{adaptor::OptimizerAdaptor, AdamW, AdamWConfig},
    prelude::*,
    tensor::backend::AutodiffBackend,
};

use enviornment::Enviornment;
use memory::{get_batch, sample_indices, Memory};
use serde::{Deserialize, Serialize};
use utils::{
    ref_to_action_tensor, ref_to_not_done_tensor, ref_to_reward_tensor, ref_to_state_tensor,
    update_parameters,
};

use crate::cli::TrainArgs;

const MEMORY_SIZE: usize = 4096;

#[derive(Config)]
pub struct TrainingConfig {
    #[config(default = 32)]
    pub batch_size: usize,
    #[config(default = 0.001)]
    pub lr: f64,
    #[config(default = 0.990)]
    pub gamma: f64,
    #[config(default = 0.005)]
    pub tau: f64,

    pub clip_grad: Option<GradientClippingConfig>,
}

pub fn run<B: Backend, DB: AutodiffBackend>(args: TrainArgs) -> Result<(), String> {
    let train_config =
        TrainingConfig::new().with_clip_grad(Some(GradientClippingConfig::Value(100.0)));

    match args.robot {
        RobotKind::MinimalRl => {
            type R<B> = PhantomData<MinimalRlRobot<B>>;
            let (r, dr): (R<B>, R<DB>) = (PhantomData, PhantomData);
            train(train_config, args.episodes, args, r, dr);
        }
        RobotKind::SmallRl => {
            type R<B> = PhantomData<SmallRlRobot<B>>;
            let (r, dr): (R<B>, R<DB>) = (PhantomData, PhantomData);
            train(train_config, args.episodes, args, r, dr);
        }
        RobotKind::SmallPolarRl => {
            type R<B> = PhantomData<SmallPolarRlRobot<B>>;
            let (r, dr): (R<B>, R<DB>) = (PhantomData, PhantomData);
            train(train_config, args.episodes, args, r, dr);
        }
        RobotKind::MediumPolarRl => {
            type R<B> = PhantomData<MediumPolarRlRobot<B>>;
            let (r, dr): (R<B>, R<DB>) = (PhantomData, PhantomData);
            train(train_config, args.episodes, args, r, dr);
        }
        other => return Err(format!("Only RL robots can be trained. Got: '{}'", other)),
    };

    Ok(())
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
    robots: usize,
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

type SwarmState<S> = Vec<S>;
type SwarmAction<A> = Vec<A>;

type Recorder = BinFileRecorder<rl::network::RecorderSettings>;

fn train<
    B: Backend,
    S: State,
    A: Action,
    N: Network<B, S, A>,
    DB: AutodiffBackend,
    DN: AutodiffNetwork<DB, S, A>,
>(
    config: TrainingConfig,
    num_episodes: usize,
    args: TrainArgs,
    _robot_ty: PhantomData<RlRobot<B, S, A, N>>,
    _diff_robot_ty: PhantomData<RlRobot<DB, S, A, DN>>,
) {
    let model_file = args.model;

    let device = Default::default();

    let mut network = DN::init(&device);

    // Load the model if it exists
    if model_file.exists() {
        let recorder = Recorder::default();
        network = network
            .load_file(&model_file, &recorder, &device)
            .expect("Failed to load model");
    }

    println!("Model loaded from '{}':", model_file.display());
    println!("{network:?}");

    let stats_file = model_file.with_extension("json");
    let mut stats = Vec::new();

    // Create behavior
    let behavior = Behavior::parse(args.robot.get_name()).unwrap();

    println!("Using behavior: {}", behavior.name());

    let mut env = Enviornment::<B, S, A, N>::new(args.world_dir, args.max_robots, behavior);

    println!("Enviornment created with {} robots", env.num_robots());

    let mut step: usize = 0;

    let mut target_net = network.clone();
    let mut policy_net = network;

    let mut memory = Memory::<DB, S, A, MEMORY_SIZE>::default();

    let mut optimizer: OptimizerAdaptor<AdamW, DN, DB> = AdamWConfig::new()
        .with_grad_clipping(config.clip_grad.clone())
        .init();

    let mut episode_done;
    let mut episode_reward: f32;
    let mut episode_duration;
    let mut state: SwarmState<S>;

    let mut total_loss: f32;
    let mut action_stats;

    let mut eps;
    let eps_base = f64::powf(2.0, 1.0 / args.eps_half);

    for episode in 0..num_episodes {
        episode_done = false;
        episode_reward = 0.0;
        episode_duration = 0_usize;
        state = env.states().clone();

        total_loss = 0.0;
        action_stats = vec![0; A::SIZE];

        eps = 0.0;

        while !episode_done {
            eps = (1.0 - args.eps_end) * f64::powf(eps_base, -(step as f64)) + args.eps_end;

            let action: SwarmAction<A> = state
                .iter()
                .map(|s| {
                    let action = policy_net.react_with_exploration(s, eps);
                    action_stats[action.clone().into()] += 1;
                    action
                })
                .collect();

            let snapshot = env.step(action.clone());

            episode_reward += snapshot.reward;

            for ((s, next_s), a) in state.iter().zip(snapshot.state.iter()).zip(action.iter()) {
                memory.push(
                    s.clone(),
                    next_s.clone(),
                    a.clone(),
                    snapshot.reward,
                    snapshot.done,
                );
            }

            if config.batch_size < memory.len() {
                let loss;
                (policy_net, target_net, loss) = train_agent(
                    policy_net,
                    target_net,
                    &memory,
                    &mut optimizer,
                    &config,
                    &device,
                );
                total_loss += loss;
            }

            episode_duration += 1;

            if snapshot.done || episode_duration >= args.max_steps {
                episode_done = true;
            } else {
                state = snapshot.state;
            }

            step += 1;
        }

        let stat = EpisodeStats {
            episode,
            eps,
            reward: episode_reward,
            coverage: env.sim().state.diagnostics.coverage_grid.coverage(),
            steps: episode_duration,
            sim_time: env.sim().state.time.as_secs_f32(),
            avg_loss: total_loss / episode_duration as f32,
            actions: action_stats,
            robots: state.len(),
        };
        println!("{stat}");

        stats.push(stat);

        env.reset();

        println!("Created new enviornment with {} robots.", env.num_robots());

        // Save model
        if let Err(e) = policy_net
            .clone()
            .save_file(&model_file, &Recorder::default())
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

fn train_agent<
    B: AutodiffBackend,
    S: State,
    A: Action,
    N: AutodiffNetwork<B, S, A>,
    const CAP: usize,
>(
    mut policy_net: N,
    mut target_net: N,
    memory: &Memory<B, S, A, CAP>,
    optimizer: &mut OptimizerAdaptor<AdamW, N, B>,
    config: &TrainingConfig,
    device: &B::Device,
) -> (N, N, f32) {
    let sample_indices = sample_indices((0..memory.len()).collect(), config.batch_size);
    let state_batch = get_batch(memory.states(), &sample_indices, |state: &S| {
        ref_to_state_tensor(state, device)
    });
    let action_batch = get_batch(memory.actions(), &sample_indices, ref_to_action_tensor);
    let state_action_values = policy_net.forward(state_batch).gather(1, action_batch);

    let next_state_batch = get_batch(memory.next_states(), &sample_indices, |state: &S| {
        ref_to_state_tensor(state, device)
    });
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

    target_net = N::soft_update(target_net, &policy_net, config.tau);

    let loss = loss.to_data().as_slice::<f32>().unwrap()[0];

    (policy_net, target_net, loss)
}
