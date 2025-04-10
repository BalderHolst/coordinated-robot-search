use crate::{params, Control};

use super::Action;

#[derive(Clone, Default)]
pub enum MinimalAction {
    #[default]
    Straight,
    Left,
    Right,
}

impl From<usize> for MinimalAction {
    fn from(i: usize) -> Self {
        match i {
            0 => MinimalAction::Straight,
            1 => MinimalAction::Left,
            2 => MinimalAction::Right,
            _ => panic!("Invalid action: {}", i),
        }
    }
}

impl From<MinimalAction> for usize {
    fn from(action: MinimalAction) -> Self {
        match action {
            MinimalAction::Straight => 0,
            MinimalAction::Left => 1,
            MinimalAction::Right => 2,
        }
    }
}

impl Action for MinimalAction {
    const SIZE: usize = 3;
    fn control(&self) -> crate::Control {
        match self {
            MinimalAction::Straight => Control {
                speed: params::SPEED_RANGE.end,
                steer: 0.0,
            },
            MinimalAction::Left => Control {
                speed: 0.0,
                steer: params::STEER_RANGE.start,
            },
            MinimalAction::Right => Control {
                speed: 0.0,
                steer: params::STEER_RANGE.end,
            },
        }
    }
}
