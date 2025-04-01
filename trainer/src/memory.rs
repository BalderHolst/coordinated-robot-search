use std::ops::Range;

use rand;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};

use crate::{SwarmAction, SwarmState};

type MemoryBatch = (
    Vec<SwarmState>,
    Vec<SwarmState>,
    Vec<SwarmAction>,
    Vec<f32>,
    Vec<bool>,
);

pub struct Memory<const CAP: usize> {
    states: ConstGenericRingBuffer<SwarmState, CAP>,
    next_states: ConstGenericRingBuffer<SwarmState, CAP>,
    actions: ConstGenericRingBuffer<SwarmAction, CAP>,
    rewards: ConstGenericRingBuffer<f32, CAP>,
    dones: ConstGenericRingBuffer<bool, CAP>,
}

impl<const CAP: usize> Memory<CAP> {
    pub fn new() -> Self {
        Self {
            states: ConstGenericRingBuffer::new(),
            next_states: ConstGenericRingBuffer::new(),
            actions: ConstGenericRingBuffer::new(),
            rewards: ConstGenericRingBuffer::new(),
            dones: ConstGenericRingBuffer::new(),
        }
    }

    pub fn push(
        &mut self,
        state: SwarmState,
        next_state: SwarmState,
        action: SwarmAction,
        reward: f32,
        done: bool,
    ) {
        self.states.push(state);
        self.next_states.push(next_state);
        self.actions.push(action);
        self.rewards.push(reward);
        self.dones.push(done);
    }

    pub fn len(&self) -> usize {
        self.states.len()
    }

    pub fn clear(&mut self) {
        self.states.clear();
        self.next_states.clear();
        self.actions.clear();
        self.rewards.clear();
        self.dones.clear();
    }

    pub fn batch(&self, range: Range<usize>) -> MemoryBatch {
        range.map(|i| {
            let state = self.states.get(i).unwrap().clone();
            let next_state = self.next_states.get(i).unwrap().clone();
            let action = self.actions.get(i).unwrap().clone();
            let reward = self.rewards.get(i).unwrap().clone();
            let done = self.dones.get(i).unwrap().clone();
            (state, next_state, action, reward, done)
        }).collect()
    }

    pub fn random_batch(&self, size: usize) -> MemoryBatch {
        let start = rand::random_range(0..self.len() - size);
        let end = start + size;
        self.batch(start..end)
    }
}
