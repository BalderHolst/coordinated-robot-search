use burn::{
    module::{Param, ParamId},
    nn::{Linear, LinearConfig},
    prelude::*,
    tensor::activation,
};

use crate::behaviors::rl::{action::RlAction, state::State};

use super::Network;

#[derive(Debug, Module)]
pub struct SmallNetwork<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
    linear3: Linear<B>,
}

impl<B: Backend> SmallNetwork<B> {
    const HIDDEN_SIZE_1: usize = 20;
    const HIDDEN_SIZE_2: usize = 10;
}

fn soft_update_tensor<const N: usize, B: Backend>(
    this: &Param<Tensor<B, N>>,
    that: &Param<Tensor<B, N>>,
    tau: f64,
) -> Param<Tensor<B, N>> {
    let that_weight = that.val();
    let this_weight = this.val();
    let new_weight = this_weight * (1.0 - tau) + that_weight * tau;

    Param::initialized(ParamId::new(), new_weight)
}

pub fn soft_update_linear<B: Backend>(this: Linear<B>, that: &Linear<B>, tau: f64) -> Linear<B> {
    let weight = soft_update_tensor(&this.weight, &that.weight, tau);
    let bias = match (&this.bias, &that.bias) {
        (Some(this_bias), Some(that_bias)) => Some(soft_update_tensor(this_bias, that_bias, tau)),
        _ => None,
    };

    Linear::<B> { weight, bias }
}

impl<B: Backend, S: State> Network<B, S> for SmallNetwork<B> {
    fn init(device: &B::Device) -> Self {
        Self {
            linear1: LinearConfig::new(S::SIZE, Self::HIDDEN_SIZE_1).init(device),
            linear2: LinearConfig::new(Self::HIDDEN_SIZE_1, Self::HIDDEN_SIZE_2).init(device),
            linear3: LinearConfig::new(Self::HIDDEN_SIZE_2, RlAction::SIZE).init(device),
        }
    }

    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = activation::relu(self.linear1.forward(input));
        let x = activation::relu(self.linear2.forward(x));
        activation::relu(self.linear3.forward(x))
    }

    fn soft_update(this: Self, that: &Self, tau: f64) -> Self {
        Self {
            linear1: soft_update_linear(this.linear1, &that.linear1, tau),
            linear2: soft_update_linear(this.linear2, &that.linear2, tau),
            linear3: soft_update_linear(this.linear3, &that.linear3, tau),
        }
    }

    fn react_with_exploration(&self, input: &S, epsilon: f64) -> RlAction {
        if rand::random::<f64>() > epsilon {
            let input_tensor = input.to_tensor();
            let output_tensor =
                <Self as Network<B, S>>::forward(self, input_tensor.clone().unsqueeze());
            output_tensor.clone().into()
        } else {
            RlAction::random()
        }
    }
}
