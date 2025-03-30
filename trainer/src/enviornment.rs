use botbrain::{
    behaviors::rl::{state::RlState, RlRobot},
    burn::prelude::Backend,
};
use simple_sim::sim::Simulator;

struct Enviornment {
    step: usize,
    max_steps: usize,
    sim: Simulator,
}

impl Enviornment {
    pub fn new(sim: Simulator, max_steps: usize) -> Self {
        Enviornment {
            sim,
            max_steps,
            step: 0,
        }
    }

    pub fn reset(&mut self) -> Snapshot {
        todo!()
    }

    pub fn step(&mut self, action: usize) -> Snapshot {
        // TODO: Maybe store the last coverage instead of recalculating it here
        let before_coverage = self.sim.state.diagnostics.coverage();

        // Step internal simulator
        self.sim.step();

        // Get states
        let state = self
            .sim
            .robots_mut()
            .iter_mut()
            .map(|r| {
                let r = r.any().downcast_ref::<RlRobot>().unwrap();
                r.state()
            })
            .collect();

        // Get reward
        let after_coverage = self.sim.state.diagnostics.coverage();
        let reward = after_coverage - before_coverage;

        let done = self.step >= self.max_steps;

        Snapshot {
            state,
            reward,
            done,
        }
    }
}

struct Snapshot {
    pub state: Vec<RlState>,
    pub reward: f32,
    pub done: bool,
}
