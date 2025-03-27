use burn::{
    nn::{Linear, LinearConfig},
    prelude::*,
};

#[derive(Debug, Module)]
pub struct Model<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
}

impl<B: Backend> Model<B> {
    pub fn forward(&self, input: Tensor<B, 1>) -> Tensor<B, 1> {
        let x = self.linear1.forward(input);
        let x = self.linear2.forward(x);
        x
    }
}

#[derive(Debug, Config)]
pub struct ModelConfig {
    #[config(default = 100)]
    input_size: usize,
    #[config(default = 2)]
    output_size: usize,
    #[config(default = 20)]
    hidden_size: usize,
}

impl ModelConfig {
    pub fn init<B: Backend>(&self, device: &B::Device) -> Model<B> {
        Model {
            linear1: LinearConfig::new(self.input_size, self.hidden_size).init(device),
            linear2: LinearConfig::new(self.hidden_size, self.output_size).init(device),
        }
    }
}
