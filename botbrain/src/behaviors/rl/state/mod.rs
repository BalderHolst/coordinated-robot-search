mod small;

use burn::{prelude::Backend, tensor::Tensor};

use super::{action::Action, RlRobot};

pub trait State: Clone + Send + 'static {
    const SIZE: usize;
    fn from_robot<A: Action>(robot: &RlRobot<Self, A>) -> Self;
    fn to_tensor<B: Backend>(&self) -> Tensor<B, 1>;
}

// State representations
pub use small::SmallState;
