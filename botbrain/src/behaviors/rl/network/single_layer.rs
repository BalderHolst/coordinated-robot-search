//! Implementation of [`SingleLayer`].

use burn::{
    nn::{Linear, LinearConfig},
    prelude::*,
    tensor::{activation, backend::AutodiffBackend},
};

use crate::behaviors::rl::{action::Action, state::State};

use super::{soft_update_linear, AutodiffNetwork, Network};

/// A small neural network with one hidden layer.
#[derive(Debug, Module)]
pub struct SingleLayerNetwork<B: Backend, const SIZE: usize> {
    linear1: Linear<B>,
    linear2: Linear<B>,
}

impl<B: AutodiffBackend, S: State, A: Action, const SIZE: usize> AutodiffNetwork<B, S, A>
    for SingleLayerNetwork<B, SIZE>
{
}

impl<B: Backend, S: State, A: Action, const SIZE: usize> Network<B, S, A>
    for SingleLayerNetwork<B, SIZE>
{
    fn init(device: &B::Device) -> Self {
        Self {
            linear1: LinearConfig::new(S::SIZE, SIZE).init(device),
            linear2: LinearConfig::new(SIZE, A::SIZE).init(device),
        }
    }

    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let mut x = input;
        x = activation::sigmoid(self.linear1.forward(x));
        x = activation::sigmoid(self.linear2.forward(x));
        return x;
    }

    fn soft_update(this: Self, that: &Self, tau: f64) -> Self {
        Self {
            linear1: soft_update_linear(this.linear1, &that.linear1, tau),
            linear2: soft_update_linear(this.linear2, &that.linear2, tau),
        }
    }
}
