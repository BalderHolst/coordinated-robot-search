use botbrain::behaviors::rl::{
    state::{RlAction, RlState},
    RlRobot,
};
use simple_sim::sim::Simulator;

pub struct Enviornment {
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

        self.sim.robots_mut().iter_mut().zip(actions).for_each(|(r, a)| {
            let r = r.any_mut().downcast_mut::<RlRobot>().unwrap();
            r.model.set_action(a);
        });

        // TODO: Maybe store the last coverage instead of recalculating it here
        let before_coverage = self.sim.state.diagnostics.coverage();

        // Step internal simulator
        self.sim.step();

        // Get reward
        // TODO: Maybe refine reward
        let after_coverage = self.sim.state.diagnostics.coverage();
        let reward = after_coverage - before_coverage;

        let state = self.states();

        // TODO: Add terminal state at 100% coverage
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
