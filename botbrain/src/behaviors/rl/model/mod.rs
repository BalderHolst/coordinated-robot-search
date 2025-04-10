pub mod small;

use std::marker::PhantomData;

use burn::{prelude::*, record};

use super::{action::Action, state::State};

#[derive(Clone)]
pub enum BotModel<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    Model(Model<B, S, A, N>),
    Controlled(A),
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> BotModel<B, S, A, N> {
    pub fn new_model(device: &B::Device) -> Self {
        let mut model: Model<B, S, A, N> = Model::init(device);

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

    pub fn new_controlled(action: A) -> Self {
        BotModel::Controlled(action)
    }

    pub fn action(&self, input: Tensor<B, 1>) -> A {
        match &self {
            BotModel::Model(model) => A::from_tensor(model.net.forward(input.unsqueeze())),
            BotModel::Controlled(action) => action.clone(),
        }
    }

    pub fn is_controlled(&self) -> bool {
        matches!(self, BotModel::Controlled(_))
    }

    pub fn set_action(&mut self, action: A) {
        *self = BotModel::Controlled(action);
    }
}

pub struct ModelConfig<B: Backend, S: State> {
    _backend: PhantomData<B>,
    _state: PhantomData<S>,
}

#[derive(Clone)]
pub struct Model<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    net: N,
    _backend: PhantomData<B>,
    _state: PhantomData<S>,
    _action: PhantomData<A>,
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> Model<B, S, A, N> {
    fn new(net: N) -> Self {
        Self {
            net,
            _backend: PhantomData,
            _state: PhantomData,
            _action: PhantomData,
        }
    }

    fn init(device: &B::Device) -> Self {
        Self::new(N::init(device))
    }
}

pub trait Network<B: Backend, S: State, A: Action>: Module<B> + Clone {
    fn init(device: &B::Device) -> Self;
    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2>;
    fn soft_update(this: Self, that: &Self, tau: f64) -> Self;
    fn react_with_exploration(&self, input: &S, epsilon: f64) -> A;
}
