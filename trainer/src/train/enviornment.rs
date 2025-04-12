use std::{f32::consts::PI, io, marker::PhantomData, path::PathBuf};

use botbrain::{
    behaviors::{
        rl::{action::Action, model::Network, state::State, RlRobot, REACT_HZ},
        Behavior,
    },
    params::{self, RADIUS},
    shapes::Circle,
    Pos2, Robot, RobotPose,
};
use rand::Rng;
use simple_sim::{
    sim::{SimArgs, Simulator},
    world::World,
};

use burn::prelude::*;

const MAX_INACTIVE_SECS: f32 = 20.0;

const COLLISION_REWARD: f32 = -5.0;

pub struct Enviornment<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    max_robots: usize,
    behavior: Behavior,
    world_dir: PathBuf,
    sim: Simulator,
    last_coverage_increase: f32,

    _backend: PhantomData<B>,
    _state: PhantomData<S>,
    _action: PhantomData<A>,
    _network: PhantomData<N>,
}

fn random_pose(world: &World) -> RobotPose {
    let mut rng = rand::rng();
    let (min, max) = world.bounds();

    let mut spawnable = false;
    let mut pos = Pos2::default();

    while !spawnable {
        pos = Pos2 {
            x: rng.random_range(min.x..=max.x),
            y: rng.random_range(min.y..=max.y),
        };

        print!("[INFO] Trying to spawn robot at {:?}", pos);

        spawnable = true;
        for cell_pos in world.iter_circle(&Circle {
            center: pos,
            radius: params::RADIUS,
        }) {
            let cell = world.get(cell_pos);
            if !matches!(cell, Some(simple_sim::world::Cell::Empty)) {
                spawnable = false;
                println!();
                break;
            }
        }
    }

    println!("\t-> SUCCESS!");

    RobotPose {
        pos,
        angle: rng.random_range(-PI..=PI),
    }
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> Enviornment<B, S, A, N> {
    pub fn new(world_dir: PathBuf, max_robots: usize, behavior: Behavior) -> Self {
        std::env::set_var("MODEL_PATH", ""); // Silence empty model warnings
        let sim = Self::create_sim(max_robots, &world_dir, behavior.clone());

        Self {
            max_robots,
            sim,
            behavior,
            world_dir,
            last_coverage_increase: 0.0,
            _backend: PhantomData,
            _state: PhantomData,
            _action: PhantomData,
            _network: PhantomData,
        }
    }

    pub fn num_robots(&self) -> usize {
        self.sim.state.robot_states.len()
    }

    fn random_world_file(world_dir: &PathBuf) -> PathBuf {
        loop {
            let files: io::Result<_> = (|| {
                let mut entries = vec![];
                for file in std::fs::read_dir(world_dir)? {
                    let file = file?;
                    let path = file.path();
                    if matches!(
                        path.extension().and_then(|s| s.to_str()),
                        Some("ron") | Some("yaml")
                    ) {
                        entries.push(path);
                    }
                }
                Ok(entries)
            })();

            match files {
                Ok(files) if !files.is_empty() => {
                    let idx = rand::rng().random_range(0..files.len());
                    return files[idx].clone();
                }
                Ok(_) => {
                    eprintln!("No valid world files found in '{}'.", world_dir.display());
                }
                Err(e) => {
                    eprintln!(
                        "Error reading world directory '{}': {}",
                        world_dir.display(),
                        e
                    );
                }
            }
            eprintln!("\nRetrying in 1 second...");
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
    }

    fn load_random_world(world_dir: &PathBuf) -> World {
        loop {
            let path = Self::random_world_file(world_dir);
            match simple_sim::world::world_from_path(&path) {
                Ok(w) => {
                    println!("Loaded world from '{}'", path.display());
                    return w;
                }
                Err(e) => {
                    eprintln!("Error loading world from '{}': {}", path.display(), e);
                    eprintln!("\nRetrying in 1 second...");
                    std::thread::sleep(std::time::Duration::from_secs(1));
                }
            };
        }
    }

    fn create_sim(max_robots: usize, world_dir: &PathBuf, behavior: Behavior) -> Simulator {
        let world = Self::load_random_world(world_dir);
        let sim_args = SimArgs { world, behavior };
        let n = rand::rng().random_range(1..=max_robots);
        let poses: Vec<RobotPose> = (0..n).map(|_| random_pose(&sim_args.world)).collect();
        let robots = Self::poses_to_robots(poses);
        Simulator::with_robots(sim_args.clone(), robots)
    }

    fn poses_to_robots(poses: Vec<RobotPose>) -> Vec<(RobotPose, Box<dyn Robot>)> {
        poses
            .iter()
            .map(|pose| {
                let robot: Box<dyn Robot> = Box::new(RlRobot::<B, S, A, N>::new_controlled());
                (pose.clone(), robot)
            })
            .collect()
    }

    pub fn reset(&mut self) {
        self.sim = Self::create_sim(self.max_robots, &self.world_dir, self.behavior.clone());
    }

    pub fn sim(&self) -> &Simulator {
        &self.sim
    }

    pub fn states(&self) -> Vec<S> {
        self.sim
            .robots()
            .iter()
            .map(|r| {
                let r = r.any().downcast_ref::<RlRobot<B, S, A, N>>().unwrap();
                r.state()
            })
            .collect()
    }

    pub fn step(&mut self, actions: Vec<A>) -> Snapshot<S> {
        assert_eq!(actions.len(), self.sim.robots().len());

        let mut reward = 0.0;
        let mut done = false;

        self.sim
            .robots_mut()
            .iter_mut()
            .zip(actions)
            .for_each(|(r, a)| {
                let r = r.any_mut().downcast_mut::<RlRobot<B, S, A, N>>().unwrap();
                r.model.set_action(a);
            });

        // TODO: Maybe store the last coverage instead of recalculating it here
        let before_coverage = self.sim.state.diagnostics.coverage();

        // Step internal simulator
        self.sim.step_duration(1.0 / REACT_HZ);

        let after_coverage = self.sim.state.diagnostics.coverage();

        if after_coverage > before_coverage {
            self.last_coverage_increase = self.sim.state.time.as_secs_f32();
        }

        if self.sim.state.time.as_secs_f32() > self.last_coverage_increase + MAX_INACTIVE_SECS {
            println!("Inactive for too long!");
            done = true;
        }

        let botbrain::Vec2 { x: w, y: h } = self.sim.world().size();

        reward += (after_coverage - before_coverage) * w * h;

        for robot in self.sim.robots() {
            let robot = robot.any().downcast_ref::<RlRobot<B, S, A, N>>().unwrap();
            let shortest = robot
                .lidar
                .points()
                .fold(f32::MAX, |acc, p| acc.min(p.distance));
            if shortest < RADIUS + 0.1 {
                reward += COLLISION_REWARD;
            }
        }

        let state = self.states();

        done |= after_coverage >= 0.95;

        Snapshot {
            state,
            reward,
            done,
        }
    }
}

pub struct Snapshot<S: State> {
    pub state: Vec<S>,
    pub reward: f32,
    pub done: bool,
}
