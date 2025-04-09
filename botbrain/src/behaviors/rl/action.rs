use burn::prelude::*;
use rand::Rng;

use crate::Control;

#[derive(Debug, Clone, Copy)]
pub struct RlAction(usize);

impl<B: Backend> From<Tensor<B, 2>> for RlAction {
    fn from(tensor: Tensor<B, 2>) -> Self {
        (tensor.argmax(1).to_data().as_slice::<i32>().unwrap()[0] as usize).into()
    }
}

impl<B: Backend> From<Tensor<B, 1>> for RlAction {
    fn from(tensor: Tensor<B, 1>) -> Self {
        let i = tensor.argmax(0).to_data();
        let i = i.as_slice::<i32>().unwrap();
        let i = i[0] as usize;
        i.into()
    }
}

impl From<usize> for RlAction {
    fn from(i: usize) -> Self {
        assert!(i < RlAction::SIZE, "Invalid action: {}", i);
        Self(i)
    }
}

impl From<RlAction> for usize {
    fn from(action: RlAction) -> Self {
        action.0
    }
}

impl RlAction {
    /// The number of discrete actions
    pub const SIZE: usize = Self::STEERS.len() * Self::SPEEDS.len();

    const STEERS: [f32; 5] = [-1.0, -0.5, 0.0, 0.5, 1.0];
    const SPEEDS: [f32; 3] = [0.1, 0.5, 1.0];

    pub fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    pub fn control(&self) -> Control {
        let Self(i) = self;
        let speed = Self::SPEEDS[i % Self::SPEEDS.len()];
        let steer = Self::STEERS[i / Self::SPEEDS.len()];
        Control { speed, steer }
    }
}

impl Default for RlAction {
    fn default() -> Self {
        (Self::SIZE / 2 + 1).into()
    }
}
