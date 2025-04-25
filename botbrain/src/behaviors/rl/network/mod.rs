pub mod small;
pub mod tiny;

use std::marker::PhantomData;

use burn::{
    module::{AutodiffModule, Param, ParamId},
    prelude::*,
    record::{BinBytesRecorder, FullPrecisionSettings, Recorder},
    tensor::backend::AutodiffBackend,
};
use nn::Linear;

use super::{action::Action, state::State};

pub type RecorderSettings = FullPrecisionSettings;

#[derive(Clone)]
pub enum BotModel<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    Model(Model<B, S, A, N>),
    Controlled(A),
}

pub trait TrainedNetwork<B: Backend, S: State, A: Action>: Network<B, S, A> {
    fn bytes() -> &'static [u8];

    fn load_trained(self, device: &B::Device) -> Self {
        let bytes = Self::bytes().to_vec();
        let recorder = BinBytesRecorder::<RecorderSettings>::default();
        let record = recorder
            .load(bytes.to_vec(), device)
            .expect("Failed to load trained model record");
        self.load_record(record)
    }
}

impl<B: Backend, S: State, A: Action, N: TrainedNetwork<B, S, A>> BotModel<B, S, A, N> {
    pub fn new_trained_model(device: &B::Device) -> Self {
        let mut model: Model<B, S, A, N> = Model::init(device);
        model.net = model.net.load_trained(device);
        BotModel::Model(model)
    }
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> BotModel<B, S, A, N> {
    pub fn new_model(model: Model<B, S, A, N>) -> Self {
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
    pub fn new(net: N) -> Self {
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
