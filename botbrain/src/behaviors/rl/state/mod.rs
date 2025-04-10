mod single_ray;
mod small;

use burn::{prelude::Backend, tensor::Tensor};

use super::{action::Action, model::Network, RlRobot};

pub trait State: Clone + Send + 'static {
    const SIZE: usize;
    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self;
    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1>;
}

// State representations
pub use single_ray::RayState;
pub use small::SmallState;
