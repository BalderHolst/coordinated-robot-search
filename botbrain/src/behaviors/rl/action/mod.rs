pub mod minimal;
pub mod square;

use burn::prelude::*;
use rand::Rng;

use crate::Control;

pub trait Action: Clone + Default + Send + From<usize> + Into<usize> + 'static {
    const SIZE: usize;

    fn random() -> Self {
        rand::rng().random_range(0..Self::SIZE).into()
    }

    fn from_tensor<B: Backend>(tensor: Tensor<B, 2>) -> Self {
        let i = tensor.argmax(1).to_data().as_slice::<i32>().unwrap()[0];
        (i as usize).into()
    }

    fn control(&self) -> Control;
}

pub use minimal::MinimalAction;
pub use square::SquareAction;
