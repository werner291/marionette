use crate::path::{ParametrizedPath, SteeringFunction};
use crate::state::{Distance, LinearInterpolate};
use std::marker::PhantomData;
use std::ops::{Div, Not};

pub trait StateValidator<State> {
    fn validate_state(&self, st: &State) -> bool;
}

impl<State, F> StateValidator<State> for F
where
    F: Fn(&State) -> bool,
{
    fn validate_state(&self, st: &State) -> bool {
        self(st)
    }
}

pub trait MotionValidator<State> {
    /// Validate whether moving from st1 to st2 is possible.
    /// On inclusivity: st1 is assumed to be a valid state, st2 will be checked.
    fn validate_motion(&self, st1: &State, st2: &State) -> bool;
}

/// Probabilistically validate the motion by sampling by linear interpolation between the two states. Validation is done in sequence, from st1 to st2.

pub struct UniformLinearInterpolatedSamplingValidator<State, Validate>
where
    State: LinearInterpolate<f64>,
    Validate: StateValidator<State>,
{
    pub validate_state: Validate,
    pub sample_interval: f64,
    _phantom: PhantomData<State>,
}

impl<State, Validate> UniformLinearInterpolatedSamplingValidator<State, Validate>
where
    State: LinearInterpolate<f64>,
    Validate: StateValidator<State>,
{
    pub fn new(validator: Validate, sample_interval: f64) -> Self {
        Self {
            validate_state: validator,
            sample_interval,
            _phantom: PhantomData,
        }
    }
}

impl<State, Validate> MotionValidator<State>
    for UniformLinearInterpolatedSamplingValidator<State, Validate>
where
    State: LinearInterpolate<f64> + Distance<State, DistanceValue = f64>,
    Validate: StateValidator<State>,
{
    fn validate_motion(&self, st1: &State, st2: &State) -> bool {
        let samples = st1.distance(st2).div(self.sample_interval).round() as usize + 1;

        (0..samples)
            .any(|i| {
                let t = i as f64 / (samples - 1) as f64;

                !self
                    .validate_state
                    .validate_state(&st1.linear_interpolate(st2, t))
            })
            .not()
    }
}

impl<State, F> MotionValidator<State> for F
where
    F: Fn(&State, &State) -> bool,
{
    fn validate_motion(&self, st1: &State, st2: &State) -> bool {
        self(st1, st2)
    }
}

pub struct SteeredSamplingValidator<State, Validate, Steer>
where
    State: LinearInterpolate<f64>,
    Validate: StateValidator<State>,
    Steer: SteeringFunction<State>,
{
    pub validate_state: Validate,
    pub sample_interval: f64,
    pub steering_function: Steer,
    _phantom: PhantomData<State>,
}

impl<State, Validate, Steer> SteeredSamplingValidator<State, Validate, Steer>
where
    State: LinearInterpolate<f64>,
    Validate: StateValidator<State>,
    Steer: SteeringFunction<State>,
{
    pub fn new(validate_state: Validate, steering_function: Steer, sample_interval: f64) -> Self {
        Self {
            validate_state,
            steering_function,
            sample_interval,
            _phantom: PhantomData,
        }
    }
}

impl<State, Validate, Steer> MotionValidator<State>
    for SteeredSamplingValidator<State, Validate, Steer>
where
    State: LinearInterpolate<f64> + Distance<State, DistanceValue = f64>,
    Validate: StateValidator<State>,
    Steer: SteeringFunction<State>,
{
    fn validate_motion(&self, st1: &State, st2: &State) -> bool {
        let samples = st1.distance(st2).div(self.sample_interval).round() as usize + 1;
        let steer_path = self.steering_function.steer(st1, st2, 0.0);
        debug_assert_eq!(0.0, *steer_path.defined_range().start());
        let steer_range = steer_path.defined_range();

        (0..samples).all(|i| {
            let t = i as f64 / (samples - 1) as f64;

            self.validate_state.validate_state(
                &steer_path
                    .sample(t * steer_range.end())
                    .expect("Steering function received out-of-range parameter."),
            )
        })
    }
}
