use burn::prelude::*;
use rand::Rng;

use crate::Control;

use super::Action;

#[derive(Debug, Clone, Copy)]
pub struct SquareAction(usize);

impl SquareAction {
    const STEERS: [f32; 5] = [-1.0, -0.5, 0.0, 0.5, 1.0];
    const SPEEDS: [f32; 3] = [0.1, 0.5, 1.0];

    pub fn control(&self) -> Control {
        let Self(i) = self;
        let speed = Self::SPEEDS[i % Self::SPEEDS.len()];
        let steer = Self::STEERS[i / Self::SPEEDS.len()];
        Control { speed, steer }
    }
}

impl Action for SquareAction {
    const SIZE: usize = Self::STEERS.len() * Self::SPEEDS.len();

    fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    fn from_tensor<B: Backend>(tensor: Tensor<B, 2>) -> Self {
        (tensor.argmax(1).to_data().as_slice::<i32>().unwrap()[0] as usize).into()
    }

    fn control(&self) -> Control {
        todo!()
    }
}

impl From<usize> for SquareAction {
    fn from(i: usize) -> Self {
        assert!(i < SquareAction::SIZE, "Invalid action: {}", i);
        Self(i)
    }
}

impl Default for SquareAction {
    fn default() -> Self {
        (Self::SIZE / 2 + 1).into()
    }
}
