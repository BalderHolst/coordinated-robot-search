use std::{cell::RefCell, rc::Rc};

use burn::{
    nn::{Linear, LinearConfig},
    prelude::*,
};

pub type ModelRef<B> = Rc<RefCell<Model<B>>>;

#[derive(Clone)]
pub enum BotModel<B: Backend> {
    Model(Model<B>),
    Ref(ModelRef<B>),
}

impl<B: Backend> BotModel<B> {
    pub fn new_model() -> Self {
        let config = ModelConfig::new();
        BotModel::Model(config.init(&Default::default()))
    }

    pub fn new_ref(model: ModelRef<B>) -> Self {
        BotModel::Ref(model)
    }

    pub fn forward(&self, input: Tensor<B, 1>) -> Tensor<B, 1> {
        match self {
            BotModel::Model(model) => model.forward(input),
            BotModel::Ref(model) => model.borrow().forward(input),
        }
    }
}

#[derive(Debug, Module)]
pub struct Model<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
}

impl<B: Backend> Model<B> {
    pub fn forward(&self, input: Tensor<B, 1>) -> Tensor<B, 1> {
        let x = self.linear1.forward(input);
        self.linear2.forward(x)
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
