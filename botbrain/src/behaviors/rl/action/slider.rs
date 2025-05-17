use crate::{
    params::{MAX_STEER, SPEED_RANGE},
    Control,
};

use super::Action;

/// Action space containing a set amount of speeds and steers.
///
/// The action space is a square grid of size `SPEEDS` x `STEERS`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SliderAction<const SIZE: usize>(usize);

impl<const SIZE: usize> Action for SliderAction<SIZE> {
    const SIZE: usize = SIZE;

    fn control(&self) -> Control {
        let Self(i) = self;

        let t = (*i as f32 / (SIZE - 1) as f32) * 2.0 - 1.0;

        let speed =
            SPEED_RANGE.start + (SPEED_RANGE.end - SPEED_RANGE.start) * (1.0 - t.abs().sqrt());
        let steer = MAX_STEER * t;

        Control { speed, steer }
    }
}

impl<const SIZE: usize> From<usize> for SliderAction<SIZE> {
    fn from(i: usize) -> Self {
        assert!(i < Self::SIZE, "Invalid action: {}", i);
        Self(i)
    }
}

impl<const SIZE: usize> From<SliderAction<SIZE>> for usize {
    fn from(action: SliderAction<SIZE>) -> Self {
        action.0
    }
}
