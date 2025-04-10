pub mod small;

use std::marker::PhantomData;

use burn::{prelude::*, record};

use super::{action::RlAction, state::State};

#[derive(Clone)]
pub enum BotModel<B: Backend, S: State, N: Network<B, S>> {
    Model(Model<B, S, N>),
    Controlled(RlAction),
}

impl<B: Backend, S: State, N: Network<B, S>> BotModel<B, S, N> {
    pub fn new_model(device: &B::Device) -> Self {
        let mut model: Model<B, S, N> = Model::init(device);

        if let Ok(path) = std::env::var("MODEL_PATH") {
            if !path.is_empty() {
                let net = model
                    .net
                    .load_file(&path, &record::DefaultRecorder::default(), device)
                    .unwrap_or_else(|_| {
                        panic!("Failed to load model from path: {}", path);
                    });
                model = Model::new(net);
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
        match &self {
            BotModel::Model(model) => model.net.forward(input.unsqueeze()).into(),
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

pub struct ModelConfig<B: Backend, S: State> {
    _backend: PhantomData<B>,
    _state: PhantomData<S>,
}

#[derive(Clone)]
pub struct Model<B: Backend, S: State, N: Network<B, S>> {
    net: N,
    _backend: PhantomData<B>,
    _state: PhantomData<S>,
}

impl<B: Backend, S: State, N: Network<B, S>> Model<B, S, N> {
    fn new(net: N) -> Self {
        Self {
            net,
            _state: PhantomData,
            _backend: PhantomData,
        }
    }

    fn init(device: &B::Device) -> Self {
        Self::new(N::init(device))
    }
}

pub trait Network<B: Backend, S: State>: Module<B> + Clone {
    fn init(device: &B::Device) -> Self;
    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2>;
    fn soft_update(this: Self, that: &Self, tau: f64) -> Self;
    fn react_with_exploration(&self, input: &S, epsilon: f64) -> RlAction;
}
