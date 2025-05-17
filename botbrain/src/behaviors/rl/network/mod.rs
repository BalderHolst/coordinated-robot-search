//! Various neural network architectures for the RL agent

pub mod medium;
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

/// Settings for saving and loading the models
pub type RecorderSettings = FullPrecisionSettings;

/// A bot model that can be either a neural network or a controlled action
#[derive(Clone)]
pub enum BotModel<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    /// A neural network model. This is used when running the trained agent.
    Model(Model<B, S, A, N>),

    /// A controlled action. This is used when the agent is being trained as
    /// the trainer program manages the model and chooses what action to take.
    Controlled(A),
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> BotModel<B, S, A, N> {
    /// Creates a new model with the given network
    pub fn new_model(model: Model<B, S, A, N>) -> Self {
        BotModel::Model(model)
    }

    /// Creates a new controlled action model
    pub fn new_controlled(action: A) -> Self {
        BotModel::Controlled(action)
    }

    /// Use the model to infer an action
    pub fn action(&self, input: Tensor<B, 1>) -> A {
        match &self {
            BotModel::Model(model) => A::from_tensor(model.net.forward(input.unsqueeze())),
            BotModel::Controlled(action) => action.clone(),
        }
    }

    /// Check if the model is controlled by an external source
    pub fn is_controlled(&self) -> bool {
        matches!(self, BotModel::Controlled(_))
    }

    /// Set the action of the model. This will change the model to a controlled.
    pub fn set_action(&mut self, action: A) {
        *self = BotModel::Controlled(action);
    }
}

/// A model used for the RL agent
#[derive(Clone)]
pub struct Model<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    net: N,
    _backend: PhantomData<B>,
    _state: PhantomData<S>,
    _action: PhantomData<A>,
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> Model<B, S, A, N> {
    /// Creates a new model with the given network
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

    /// Get the network used by the model
    pub fn network(&self) -> &N {
        &self.net
    }

    /// Run the model forward pass
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        self.net.forward(input)
    }
}

/// A trait for a trained network. To implement this trait implement the
/// [`Self::bytes`] method to return the bytes of the trained model using the
/// [`include_bytes!`] macro.
pub trait TrainedNetwork<B: Backend, S: State, A: Action>: Network<B, S, A> {
    /// Returns the bytes of the trained model. See [`include_bytes!`].
    fn bytes() -> &'static [u8];

    /// Loads the trained model from the bytes returned by [`Self::bytes`].
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
    /// Initializes a new model with the trained network.
    pub fn new_trained_model(device: &B::Device) -> Self {
        let mut model: Model<B, S, A, N> = Model::init(device);
        model.net = model.net.load_trained(device);
        BotModel::Model(model)
    }
}

/// A trait defining a neural network.
pub trait Network<B: Backend, S: State, A: Action>: Module<B> + Clone + 'static {
    /// Initializes the network with the given device
    fn init(device: &B::Device) -> Self;

    /// Runs the network forward pass
    fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2>;

    /// Soft updates the network weights using the given tau value
    fn soft_update(this: Self, that: &Self, tau: f64) -> Self;

    /// React to the input state with `epsilon`-greedy exploration.
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

/// A [`Network`] using an [`AutodiffBackend`]. This is used for training the model.
pub trait AutodiffNetwork<B: AutodiffBackend, S: State, A: Action>:
    Network<B, S, A> + AutodiffModule<B>
{
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

/// Soft updates the weights of a linear layer using the given `tau` value
pub fn soft_update_linear<B: Backend>(this: Linear<B>, that: &Linear<B>, tau: f64) -> Linear<B> {
    let weight = soft_update_tensor(&this.weight, &that.weight, tau);
    let bias = match (&this.bias, &that.bias) {
        (Some(this_bias), Some(that_bias)) => Some(soft_update_tensor(this_bias, that_bias, tau)),
        _ => None,
    };

    Linear::<B> { weight, bias }
}
