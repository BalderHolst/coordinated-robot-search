//! Defines the action space for the RL agent.

mod minimal;
mod slider;
mod square;

use burn::prelude::*;
use rand::Rng;

use crate::Control;

/// The action space for the RL agent
///
/// To create a new action space, implement the `SIZE` constant and the `control` method.
pub trait Action: Clone + Default + Send + From<usize> + Into<usize> + 'static {
    /// The size of the action space
    const SIZE: usize;

    /// Get a random action
    fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    /// Get an action from an output tensor
    fn from_tensor<B: Backend>(tensor: Tensor<B, 2>) -> Self {
        let i = tensor.argmax(1).to_data().as_slice::<i32>().unwrap()[0];
        (i as usize).into()
    }

    /// Convert the action to a control signal
    fn control(&self) -> Control;
}

pub use minimal::MinimalAction;
pub use slider::SliderAction;
pub use square::SquareAction;
