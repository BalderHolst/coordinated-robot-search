mod polar;
mod single_ray;
mod small;

use burn::prelude::*;

use super::{action::Action, network::Network, RlRobot};

pub trait State: Clone + Send + 'static {
    const SIZE: usize;
    fn from_robot<B: Backend, A: Action, N: Network<B, Self, A>>(
        robot: &RlRobot<B, Self, A, N>,
    ) -> Self;
    fn to_tensor<B: Backend>(&self, device: &B::Device) -> Tensor<B, 1>;
}

// State representations
pub use polar::PolarState;
pub use single_ray::RayState;
pub use small::SmallState;

pub mod utils {
    pub struct ArrayWriter<'a> {
        data: &'a mut [f32],
        cursor: usize,
    }

    impl<'a> ArrayWriter<'a> {
        pub fn new(data: &'a mut [f32]) -> Self {
            Self { data, cursor: 0 }
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
    }
}
