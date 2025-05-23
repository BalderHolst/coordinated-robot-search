//! Implementation of [`TinyNet`].

use burn::{
    prelude::*,
    tensor::{activation, backend::AutodiffBackend},
};
use nn::{Linear, LinearConfig};

use crate::behaviors::rl::{action::Action, state::State};

use super::{soft_update_linear, AutodiffNetwork, Network};

/// A tiny neural network with one hidden layer.
#[derive(Debug, Module)]
pub struct TinyNet<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
    linear3: Linear<B>,
}

impl<B: Backend> TinyNet<B> {
    const HIDDEN_SIZE_1: usize = 5;
    const HIDDEN_SIZE_2: usize = 5;
}

impl<B: AutodiffBackend, S: State, A: Action> AutodiffNetwork<B, S, A> for TinyNet<B> {}

impl<B: Backend, S: State, A: Action> Network<B, S, A> for TinyNet<B> {
    fn init(device: &B::Device) -> Self {
        Self {
            linear1: LinearConfig::new(S::SIZE, Self::HIDDEN_SIZE_1).init(device),
            linear2: LinearConfig::new(Self::HIDDEN_SIZE_1, Self::HIDDEN_SIZE_2).init(device),
            linear3: LinearConfig::new(Self::HIDDEN_SIZE_2, A::SIZE).init(device),
        }
    }

    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = activation::sigmoid(self.linear1.forward(input));
        let x = activation::sigmoid(self.linear2.forward(x));
        activation::sigmoid(self.linear3.forward(x))
    }

    fn soft_update(this: Self, that: &Self, tau: f64) -> Self {
        Self {
            linear1: soft_update_linear(this.linear1, &that.linear1, tau),
            linear2: soft_update_linear(this.linear2, &that.linear2, tau),
            linear3: soft_update_linear(this.linear3, &that.linear3, tau),
        }
    }
}
