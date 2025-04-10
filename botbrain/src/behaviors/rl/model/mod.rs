pub mod small;
pub mod tiny;

use std::marker::PhantomData;

use burn::{
    module::{AutodiffModule, Param, ParamId},
    prelude::*,
    record,
    tensor::backend::AutodiffBackend,
};
use nn::Linear;

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

    pub fn network(&self) -> &N {
        &self.net
    }

    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        self.net.forward(input)
    }
}

pub trait AutodiffNetwork<B: AutodiffBackend, S: State, A: Action>:
    Network<B, S, A> + AutodiffModule<B>
{
}

pub trait Network<B: Backend, S: State, A: Action>: Module<B> + Clone + 'static {
    fn init(device: &B::Device) -> Self;
    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2>;
    fn soft_update(this: Self, that: &Self, tau: f64) -> Self;

    fn react_with_exploration(&self, input: &S, epsilon: f64) -> A {
        if rand::random::<f64>() > epsilon {
            let input_tensor = input.to_tensor(&Default::default());
            let output_tensor =
                <Self as Network<B, S, A>>::forward(self, input_tensor.clone().unsqueeze());
            A::from_tensor(output_tensor)
        } else {
            A::random()
        }
    }
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
