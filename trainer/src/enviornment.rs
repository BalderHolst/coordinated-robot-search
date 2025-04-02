use botbrain::{
    behaviors::{
        rl::{
            state::{RlAction, RlState},
            RlRobot, REACT_HZ,
        },
        Behavior,
    },
    Robot, RobotPose,
};
use simple_sim::{
    sim::{SimArgs, Simulator},
    world::World,
};

const MAX_INACTIVE_SECS: f32 = 10.0;
const INACTIVE_REWARD: f32 = -10.0;

const COVERAGE_REWARD_MULTIPLIER: f32 = 100.0;

pub struct Enviornment {
    step: usize,
    max_steps: usize,
    sim_args: SimArgs,
    init_poses: Vec<RobotPose>,
    sim: Simulator,
    last_coverage_increase: f32,
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
    pub fn new(
        world: World,
        behavior: Behavior,
        init_poses: Vec<RobotPose>,
        max_steps: usize,
    ) -> Self {
        let robots = poses_to_robots(init_poses.clone());

        let sim_args = SimArgs { world, behavior };
        let sim = Simulator::with_robots(sim_args.clone(), robots);

        Enviornment {
            sim,
            sim_args,
            init_poses,
            max_steps,
            step: 0,
            last_coverage_increase: 0.0,
        }
    }

    pub fn reset(&mut self) {
        let robots = poses_to_robots(self.init_poses.clone());
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

        // Get reward
        // TODO: Maybe refine reward
        let after_coverage = self.sim.state.diagnostics.coverage();

        if after_coverage > before_coverage {
            self.last_coverage_increase = self.sim.state.time.as_secs_f32();
        }

        if self.sim.state.time.as_secs_f32() > self.last_coverage_increase + MAX_INACTIVE_SECS {
            return Snapshot {
                state: self.states(),
                reward: INACTIVE_REWARD,
                done: true,
            };
        }

        let reward = (after_coverage - before_coverage) * COVERAGE_REWARD_MULTIPLIER;

        let state = self.states();

        // TODO: Add terminal state at 100% coverage
        let done = self.step >= self.max_steps || after_coverage >= 0.95;

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
