mod small;

use burn::{prelude::Backend, tensor::Tensor};

use super::RlRobot;

pub trait State: Clone + Send + From<RlRobot<Self>> {
    const SIZE: usize;
    fn to_tensor<B: Backend>(&self) -> Tensor<B, 1>;
}

// State representations
pub use small::SmallState;
