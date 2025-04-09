use std::{cell::RefCell, rc::Rc};

use burn::{
    module::{Param, ParamId},
    nn::{Linear, LinearConfig},
    prelude::*,
    record,
    tensor::activation,
};

use super::action::RlAction;
use super::state::RlState;

pub type ModelRef<B> = Rc<RefCell<Model<B>>>;

#[derive(Clone)]
pub enum BotModel<B: Backend> {
    Model(Model<B>),
    Controlled(RlAction),
}

impl<B: Backend> BotModel<B> {
    pub fn new_model() -> Self {
        let config = ModelConfig::new();
        let mut model = config.init(&Default::default());

        if let Ok(path) = std::env::var("MODEL_PATH") {
            if !path.is_empty() {
                model = model
                    .load_file(
                        &path,
                        &record::DefaultRecorder::default(),
                        &Default::default(),
                    )
                    .unwrap_or_else(|_| {
                        panic!("Failed to load model from path: {}", path);
                    });
                println!("Model loaded from path: {}", path);
            }
        } else {
            println!("[WARNING] MODEL_PATH not set, using random model");
        }

        BotModel::Model(model)
    }

    pub fn new_controlled(action: RlAction) -> Self {
        BotModel::Controlled(action)
    }

    pub fn action(&self, input: Tensor<B, 1>) -> RlAction {
        match self {
            BotModel::Model(model) => model.forward(input.unsqueeze()).into(),
            BotModel::Controlled(action) => *action,
        }
    }

    pub fn is_controlled(&self) -> bool {
        matches!(self, BotModel::Controlled(_))
    }

    pub fn set_action(&mut self, action: RlAction) {
        *self = BotModel::Controlled(action);
    }
}

#[derive(Debug, Module)]
pub struct Model<B: Backend> {
    linear1: Linear<B>,
    linear2: Linear<B>,
    linear3: Linear<B>,
}

fn soft_update_tensor<const N: usize, B: Backend>(
    this: &Param<Tensor<B, N>>,
    that: &Param<Tensor<B, N>>,
    tau: f64,
) -> Param<Tensor<B, N>> {
    let that_weight = that.val();
    let this_weight = this.val();
    let new_weight = this_weight * (1.0 - tau) + that_weight * tau;

    Param::initialized(ParamId::new(), new_weight)
}

pub fn soft_update_linear<B: Backend>(this: Linear<B>, that: &Linear<B>, tau: f64) -> Linear<B> {
    let weight = soft_update_tensor(&this.weight, &that.weight, tau);
    let bias = match (&this.bias, &that.bias) {
        (Some(this_bias), Some(that_bias)) => Some(soft_update_tensor(this_bias, that_bias, tau)),
        _ => None,
    };

    Linear::<B> { weight, bias }
}

impl<B: Backend> Model<B> {
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = activation::relu(self.linear1.forward(input));
        let x = activation::relu(self.linear2.forward(x));
        activation::relu(self.linear3.forward(x))
    }

    pub fn consume(self) -> (Linear<B>, Linear<B>, Linear<B>) {
        (self.linear1, self.linear2, self.linear3)
    }

    pub fn soft_update(this: Self, that: &Self, tau: f64) -> Self {
        Self {
            linear1: soft_update_linear(this.linear1, &that.linear1, tau),
            linear2: soft_update_linear(this.linear2, &that.linear2, tau),
            linear3: soft_update_linear(this.linear3, &that.linear3, tau),
        }
    }

    pub fn react_with_exploration(&self, input: &RlState, epsilon: f64) -> RlAction {
        if rand::random::<f64>() > epsilon {
            let input_tensor = input.to_tensor::<B>();
            let output_tensor = self.forward(input_tensor.clone().unsqueeze());
            output_tensor.clone().into()
        } else {
            RlAction::random()
        }
    }
}

#[derive(Debug, Config)]
pub struct ModelConfig {
    #[config(default = 20)]
    pub hidden_size1: usize,
    #[config(default = 10)]
    pub hidden_size2: usize,
}

impl ModelConfig {
    pub fn init<B: Backend>(&self, device: &B::Device) -> Model<B> {
        Model {
            linear1: LinearConfig::new(RlState::SIZE, self.hidden_size1).init(device),
            linear2: LinearConfig::new(self.hidden_size1, self.hidden_size2).init(device),
            linear3: LinearConfig::new(self.hidden_size2, RlAction::SIZE).init(device),
        }
    }
}
