use std::f32::consts::PI;

use botbrain::{
    behaviors::{
        rl::{
            state::{RlAction, RlState},
            RlRobot, REACT_HZ,
        },
        Behavior,
    },
    params::RADIUS,
    shapes::Circle,
    Pos2, Robot, RobotPose,
};
use rand::Rng;
use simple_sim::{
    sim::{SimArgs, Simulator},
    world::World,
};

const MAX_INACTIVE_SECS: f32 = 20.0;

const COLLISION_REWARD: f32 = -5.0;
const COVERAGE_REWARD_MULTIPLIER: f32 = 100.0;

pub struct Enviornment {
    num_robots: usize,
    sim_args: SimArgs,
    sim: Simulator,
    last_coverage_increase: f32,
}

const ROBOT_SPACE: f32 = 4.0;
fn random_pose(world: &World) -> RobotPose {
    let mut rng = rand::rng();
    let (min, max) = world.bounds();
    'outer: loop {
        let pos = Pos2 {
            x: rng.random_range(min.x..=max.x),
            y: rng.random_range(min.y..=max.y),
        };

        for cell_pos in world.iter_circle(&Circle {
            center: pos,
            radius: ROBOT_SPACE,
        }) {
            let cell = world.get(cell_pos);
            if !matches!(cell, Some(simple_sim::world::Cell::Empty)) {
                continue 'outer;
            }
        }

        return RobotPose {
            pos,
            angle: rng.random_range(-PI..=PI),
        };
    }
}

fn poses_to_robots(poses: Vec<RobotPose>) -> Vec<(RobotPose, Box<dyn Robot>)> {
    poses
        .iter()
        .map(|pose| {
            let robot = RlRobot::new_controlled();
            let robot = Box::new(robot) as Box<dyn Robot>;
            (pose.clone(), robot)
        })
        .collect()
}

impl Enviornment {
    pub fn new(world: World, num_robots: usize) -> Self {
        let behavior = Behavior::parse("rl").unwrap().with_name("training");

        let poses: Vec<RobotPose> = (0..num_robots).map(|_| random_pose(&world)).collect();
        let robots = poses_to_robots(poses);

        let sim_args = SimArgs { world, behavior };
        let sim = Simulator::with_robots(sim_args.clone(), robots);

        // Set MODEL_PATH variable
        std::env::set_var("MODEL_PATH", "");

        Enviornment {
            num_robots,
            sim,
            sim_args,
            last_coverage_increase: 0.0,
        }
    }

    pub fn reset(&mut self) {
        let poses: Vec<RobotPose> = (0..self.num_robots)
            .map(|_| random_pose(&self.sim_args.world))
            .collect();
        let robots = poses_to_robots(poses);
        self.sim = Simulator::with_robots(self.sim_args.clone(), robots);
    }

    pub fn sim(&self) -> &Simulator {
        &self.sim
    }

    pub fn states(&self) -> Vec<RlState> {
        self.sim
            .robots()
            .iter()
            .map(|r| {
                let r = r.any().downcast_ref::<RlRobot>().unwrap();
                r.state()
            })
            .collect()
    }

    pub fn step(&mut self, actions: Vec<RlAction>) -> Snapshot {
        assert_eq!(actions.len(), self.sim.robots().len());

        let mut reward = 0.0;
        let mut done = false;

        self.sim
            .robots_mut()
            .iter_mut()
            .zip(actions)
            .for_each(|(r, a)| {
                let r = r.any_mut().downcast_mut::<RlRobot>().unwrap();
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

        reward += (after_coverage - before_coverage) * COVERAGE_REWARD_MULTIPLIER;

        for robot in self.sim.robots() {
            let robot = robot.any().downcast_ref::<RlRobot>().unwrap();
            let shortest = robot
                .lidar
                .points()
                .fold(f32::MAX, |acc, p| acc.min(p.distance));
            if shortest < RADIUS + 0.1 {
                println!("COLLISION!!!");
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

pub struct Snapshot {
    pub state: Vec<RlState>,
    pub reward: f32,
    pub done: bool,
}
