use std::marker::PhantomData;

use burn::{
    module::{Param, ParamId},
    nn::{Linear, LinearConfig},
    prelude::*,
    record,
    tensor::activation,
};

use super::state::RlState;
use super::{action::RlAction, state::State};

#[derive(Clone)]
pub struct BotModel<B: Backend, S: State> {
    kind: BotModelKind<B>,
    _state: PhantomData<S>,
}

#[derive(Clone)]
enum BotModelKind<B: Backend> {
    Model(Model<B>),
    Controlled(RlAction),
}

impl<B: Backend, S: State> BotModel<B, S> {
    fn new(kind: BotModelKind<B>) -> Self {
        Self {
            kind,
            _state: PhantomData,
        }
    }

    pub fn new_model(device: &B::Device) -> Self {
        let mut model = Model::new(device);

        if let Ok(path) = std::env::var("MODEL_PATH") {
            if !path.is_empty() {
                model = model
                    .load_file(&path, &record::DefaultRecorder::default(), device)
                    .unwrap_or_else(|_| {
                        panic!("Failed to load model from path: {}", path);
                    });
                println!("Model loaded from path: {}", path);
            }
        } else {
            println!("[WARNING] MODEL_PATH not set, using random model");
        }

        Self::new(BotModelKind::Model(model))
    }

    pub fn new_controlled(action: RlAction) -> Self {
        Self::new(BotModelKind::Controlled(action))
    }

    pub fn action(&self, input: Tensor<B, 1>) -> RlAction {
        match &self.kind {
            BotModelKind::Model(model) => model.forward(input.unsqueeze()).into(),
            BotModelKind::Controlled(action) => *action,
        }
    }

    pub fn is_controlled(&self) -> bool {
        matches!(self.kind, BotModelKind::Controlled(_))
    }

    pub fn set_action(&mut self, action: RlAction) {
        self.kind = BotModelKind::Controlled(action);
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
    const HIDDEN_SIZE_1: usize = 20;
    const HIDDEN_SIZE_2: usize = 10;

    pub fn new(device: &B::Device) -> Model<B> {
        Model {
            linear1: LinearConfig::new(RlState::SIZE, Self::HIDDEN_SIZE_1).init(device),
            linear2: LinearConfig::new(Self::HIDDEN_SIZE_1, Self::HIDDEN_SIZE_2).init(device),
            linear3: LinearConfig::new(Self::HIDDEN_SIZE_2, RlAction::SIZE).init(device),
        }
    }

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
            let input_tensor = input.to_tensor();
            let output_tensor = self.forward(input_tensor.clone().unsqueeze());
            output_tensor.clone().into()
        } else {
            RlAction::random()
        }
    }
}
