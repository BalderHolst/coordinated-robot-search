use std::ops::Range;

use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use rand;

use crate::{SwarmAction, SwarmState};

type MemoryTuple = (SwarmState, SwarmState, SwarmAction, f32, bool);

pub struct Memory<const CAP: usize> {
    memories: ConstGenericRingBuffer<MemoryTuple, CAP>,
}

impl<const CAP: usize> Memory<CAP> {
    pub fn new() -> Self {
        Self {
            memories: ConstGenericRingBuffer::new(),
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
        self.memories
            .push((state, next_state, action, reward, done));
    }

    pub fn len(&self) -> usize {
        self.memories.len()
    }

    pub fn clear(&mut self) {
        self.memories.clear();
    }

    pub fn batch(&self, range: Range<usize>) -> Vec<&MemoryTuple> {
        range
            .into_iter()
            .map(|i| self.memories.get(i).unwrap())
            .collect::<Vec<_>>()
    }

    pub fn random_batch(&self, size: usize) -> Vec<&MemoryTuple> {
        let start = rand::random_range(0..self.memories.len()-size);
        let end = start + size;
        self.batch(start..end)
    }
}
