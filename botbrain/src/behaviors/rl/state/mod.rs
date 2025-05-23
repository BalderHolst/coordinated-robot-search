//! State spaces for the RL agent

mod polar;
mod single_ray;
mod small;
mod tiny_polar;

use burn::prelude::*;

use super::{action::Action, network::Network, RlRobot};

/// The state space for the RL agent
pub trait State: Clone + Send + 'static {
    /// The number of dimensions in the state space
    const SIZE: usize;

    /// Construct an RL state from a robot
    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self;

    /// Convert the state to a tensor
    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1>;
}

// State representations
pub use polar::PolarState;
pub use single_ray::RayState;
pub use small::SmallState;
pub use tiny_polar::TinyPolarState;

mod utils {
    pub struct ArrayWriter<const SIZE: usize> {
        data: [f32; SIZE],
        cursor: usize,
    }

    impl<const SIZE: usize> ArrayWriter<SIZE> {
        pub fn new() -> Self {
            Self {
                data: [Default::default(); SIZE],
                cursor: 0,
            }
        }

        pub fn write(&mut self, value: f32) {
            assert!(self.cursor < self.data.len(), "ArrayWriter: Out of bounds");
            self.data[self.cursor] = value;
            self.cursor += 1;
        }

        pub fn write_array(&mut self, array: &[f32]) {
            for &value in array {
                self.write(value);
            }
        }

        pub fn finish(self) -> [f32; SIZE] {
            assert_eq!(self.cursor, SIZE, "ArrayWriter: Not finished");
            self.data
        }
    }
}
