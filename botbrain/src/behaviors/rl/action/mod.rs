pub mod square;

use burn::prelude::*;

use crate::Control;

pub trait Action: Clone + Default + Send + From<usize> + 'static {
    const SIZE: usize;
    fn random() -> Self;
    fn from_tensor<B: Backend>(tensor: Tensor<B, 2>) -> Self;
    fn control(&self) -> Control;
}

pub use square::SquareAction;
