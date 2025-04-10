use botbrain::behaviors::rl::action::Action;
use botbrain::behaviors::rl::state::State;
use burn::module::AutodiffModule;
use burn::optim::{GradientsParams, Optimizer};
use burn::tensor::backend::{AutodiffBackend, Backend};
use burn::tensor::{Int, Tensor};
use burn::LearningRate;

pub(crate) fn to_state_tensor<B: Backend, S: State>(state: S, device: &B::Device) -> Tensor<B, 1> {
    state.to_tensor(device)
}

pub(crate) fn ref_to_state_tensor<B: Backend, S: State>(
    state: &S,
    device: &B::Device,
) -> Tensor<B, 1> {
    to_state_tensor(state.clone(), device)
}

pub(crate) fn to_action_tensor<B: Backend, A: Action>(action: A) -> Tensor<B, 1, Int> {
    Tensor::<B, 1, Int>::from_ints([action.into()], &Default::default())
}

pub(crate) fn ref_to_action_tensor<B: Backend, A: Action>(action: &A) -> Tensor<B, 1, Int> {
    to_action_tensor(action.clone())
}

pub(crate) fn to_reward_tensor<B: Backend>(reward: f32) -> Tensor<B, 1> {
    Tensor::from_floats([reward], &Default::default())
}

pub(crate) fn ref_to_reward_tensor<B: Backend>(reward: &f32) -> Tensor<B, 1> {
    to_reward_tensor(*reward)
}
pub(crate) fn to_not_done_tensor<B: Backend>(done: bool) -> Tensor<B, 1> {
    Tensor::from_floats([if done { 0.0 } else { 1.0 }], &Default::default())
}

pub(crate) fn ref_to_not_done_tensor<B: Backend>(done: &bool) -> Tensor<B, 1> {
    to_not_done_tensor(*done)
}

pub(crate) fn update_parameters<B: AutodiffBackend, M: AutodiffModule<B>>(
    loss: Tensor<B, 1>,
    module: M,
    optimizer: &mut impl Optimizer<M, B>,
    learning_rate: LearningRate,
) -> M {
    let gradients = loss.backward();
    let gradient_params = GradientsParams::from_grads(gradients, &module);
    optimizer.step(learning_rate, module, gradient_params)
}
