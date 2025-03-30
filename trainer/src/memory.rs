use std::ops::Range;

use botbrain::behaviors::rl::state::{RlAction, RlState};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};

type MemoryTuple = (RlState, RlState, RlAction, f32, bool);

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
        state: RlState,
        next_state: RlState,
        action: RlAction,
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
            .filter_map(|i| self.memories.get(i))
            .collect::<Vec<_>>()
    }
}
