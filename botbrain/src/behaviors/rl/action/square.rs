use crate::{params, Control};

use super::Action;

/// Action space containing a set amount of speeds and steers.
///
/// The action space is a square grid of size `SPEEDS` x `STEERS`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SquareAction<const SPEEDS: usize, const STEERS: usize>(usize);

const MIN_SPEED_T: f32 = 0.1;

impl<const SPEEDS: usize, const STEERS: usize> Action for SquareAction<SPEEDS, STEERS> {
    const SIZE: usize = STEERS * SPEEDS;

    fn control(&self) -> Control {
        let Self(i) = self;

        assert!(*i < Self::SIZE, "Invalid action: {}", i);

        // Interpolation variables in range [0, 1]
        let speed_t = (i % SPEEDS) as f32 / (SPEEDS - 1) as f32;
        let steer_t = (i / SPEEDS) as f32 / (STEERS - 1) as f32;

        let speed_t = speed_t.max(MIN_SPEED_T);

        assert!((0.0..=1.0).contains(&speed_t),);
        assert!((0.0..=1.0).contains(&steer_t),);

        let speed = params::SPEED_RANGE.start
            + (params::SPEED_RANGE.end - params::SPEED_RANGE.start) * speed_t;
        let steer = params::STEER_RANGE.start
            + (params::STEER_RANGE.end - params::STEER_RANGE.start) * steer_t;

        Control { speed, steer }
    }
}

impl<const SPEEDS: usize, const STEERS: usize> From<usize> for SquareAction<SPEEDS, STEERS> {
    fn from(i: usize) -> Self {
        assert!(i < Self::SIZE, "Invalid action: {}", i);
        Self(i)
    }
}

impl<const SPEEDS: usize, const STEERS: usize> From<SquareAction<SPEEDS, STEERS>> for usize {
    fn from(action: SquareAction<SPEEDS, STEERS>) -> Self {
        action.0
    }
}
