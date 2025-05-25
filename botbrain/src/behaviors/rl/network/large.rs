//! Implementation of [`LargeNetwork`].

use burn::{
    nn::{Linear, LinearConfig},
    prelude::*,
    tensor::{activation, backend::AutodiffBackend},
};

use crate::behaviors::rl::{action::Action, state::State};

use super::{soft_update_linear, AutodiffNetwork, Network};

/// A small neural network with one hidden layer.
#[derive(Debug, Module)]
pub struct LargeNetwork<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
    linear3: Linear<B>,
    linear4: Linear<B>,
}

impl<B: Backend> LargeNetwork<B> {
    const HIDDEN_SIZE_1: usize = 96;
    const HIDDEN_SIZE_2: usize = 64;
    const HIDDEN_SIZE_3: usize = 32;
}

impl<B: AutodiffBackend, S: State, A: Action> AutodiffNetwork<B, S, A> for LargeNetwork<B> {}

impl<B: Backend, S: State, A: Action> Network<B, S, A> for LargeNetwork<B> {
    fn init(device: &B::Device) -> Self {
        Self {
            linear1: LinearConfig::new(S::SIZE, Self::HIDDEN_SIZE_1).init(device),
            linear2: LinearConfig::new(Self::HIDDEN_SIZE_1, Self::HIDDEN_SIZE_2).init(device),
            linear3: LinearConfig::new(Self::HIDDEN_SIZE_2, Self::HIDDEN_SIZE_3).init(device),
            linear4: LinearConfig::new(Self::HIDDEN_SIZE_3, A::SIZE).init(device),
        }
    }

    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = activation::sigmoid(self.linear1.forward(input));
        let x = activation::sigmoid(self.linear2.forward(x));
        let x = activation::sigmoid(self.linear3.forward(x));
        activation::sigmoid(self.linear4.forward(x))
    }

    fn soft_update(this: Self, that: &Self, tau: f64) -> Self {
        Self {
            linear1: soft_update_linear(this.linear1, &that.linear1, tau),
            linear2: soft_update_linear(this.linear2, &that.linear2, tau),
            linear3: soft_update_linear(this.linear3, &that.linear3, tau),
            linear4: soft_update_linear(this.linear4, &that.linear4, tau),
        }
    }
}
