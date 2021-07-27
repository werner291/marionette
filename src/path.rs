use crate::state::LinearInterpolate;
use itertools::Itertools;
use nonempty::NonEmpty;
use std::cmp::Ordering;
use std::convert::TryFrom;
use std::marker::PhantomData;
use std::ops::RangeInclusive;
use std::prelude::v1::Vec;
use std::result::Result;

#[derive(Debug)]
pub struct DiscretePointsPath<State> {
    pub states: Vec<State>,
}

#[derive(Debug)]
pub struct NonEmptyPointsPath<State> {
    pub states: NonEmpty<State>,
}

pub struct IsEmptyError;

impl<State> TryFrom<DiscretePointsPath<State>> for NonEmptyPointsPath<State> {
    type Error = IsEmptyError;

    fn try_from(value: DiscretePointsPath<State>) -> Result<Self, Self::Error> {
        NonEmpty::from_vec(value.states)
            .map(|states| NonEmptyPointsPath { states })
            .ok_or(IsEmptyError)
    }
}

pub struct TimedState<State>(pub State, pub f64);

impl<State: LinearInterpolate<f64>> LinearInterpolate<f64> for TimedState<State> {
    fn linear_interpolate(&self, other: &Self, t: f64) -> Self {
        TimedState(
            self.0.linear_interpolate(&other.0, t),
            self.1.linear_interpolate(&other.1, t),
        )
    }
}

pub fn parametrize_by_distance<State: Copy>(
    path: NonEmptyPointsPath<State>,
    distance: impl Fn(&State, &State) -> f64,
    start_point: f64,
) -> NonEmptyPointsPath<TimedState<State>> {
    NonEmptyPointsPath {
        states: NonEmpty {
            head: TimedState(path.states[0], start_point),
            tail: path
                .states
                .iter()
                .tuple_windows()
                .scan(start_point, |t, (st1, st2)| {
                    *t += distance(st1, st2);

                    Some(TimedState(*st2, *t))
                })
                .collect(),
        },
    }
}

#[derive(Debug, Copy, Clone)]
pub struct OutOfRangeError;

pub trait ParametrizedPath<State> {
    fn defined_range(&self) -> RangeInclusive<f64>;

    fn sample(&self, t: f64) -> Result<State, OutOfRangeError>;
}

pub struct LerpPathDiscretePath<'a, State>(pub &'a NonEmptyPointsPath<TimedState<State>>);

impl<'a, State: LinearInterpolate<f64> + Copy> ParametrizedPath<State>
    for LerpPathDiscretePath<'a, State>
{
    fn defined_range(&self) -> RangeInclusive<f64> {
        let TimedState(_, t0) = self.0.states.first();
        let TimedState(_, t1) = self.0.states.last();
        *t0..=*t1
    }

    fn sample(&self, t: f64) -> Result<State, OutOfRangeError> {
        if !self.defined_range().contains(&t) {
            // FIXME range bounds
            // t is before the first sample point, or beyond the last sample point
            Err(OutOfRangeError)
        } else {
            // Find the index of the first point that has a time greater than the sample point.
            let first_after = self.0.states.tail.partition_point(move |TimedState(_,state_t)| *state_t <= t)+1 /*+1 since there's the tail*/;

            if first_after == self.0.states.len() {
                let TimedState(s0, _t0) = &self.0.states[first_after - 1];
                Ok(*s0)
            } else {
                let TimedState(s0, t0) = &self.0.states[first_after - 1];
                let TimedState(s1, t1) = &self.0.states[first_after];

                Ok(s0.linear_interpolate(s1, (t - t0) / (t1 - t0)))
            }
        }
    }
}

pub trait SteeringFunction<State> {
    type PathType: ParametrizedPath<State>;
    fn steer(&self, from: &State, to: &State, start_time: f64) -> Self::PathType;
}

impl<State, F, P> SteeringFunction<State> for F
where
    F: Fn(&State, &State, f64) -> P,
    P: ParametrizedPath<State>,
{
    type PathType = P;

    fn steer(&self, from: &State, to: &State, start_time: f64) -> Self::PathType {
        self(from, to, start_time)
    }
}

pub struct CompoundPath<State, Subpath: ParametrizedPath<State>> {
    pub segments: NonEmpty<Subpath>,
    _phantom: PhantomData<State>,
}

#[derive(Debug)]
pub struct NotAtLeastOnePairOfStatesError;

pub fn build_compound_path_from_steering_function<Steer, State: Sized>(
    path: NonEmptyPointsPath<State>,
    steer: Steer,
    start_time: f64,
) -> Result<CompoundPath<State, Steer::PathType>, NotAtLeastOnePairOfStatesError>
where
    Steer: SteeringFunction<State>,
{
    NonEmpty::from_vec(
        path.states
            .iter()
            .tuple_windows()
            .scan(start_time, |t, (st1, st2)| {
                let segment = steer.steer(st1, st2, *t);
                assert_eq!(t, segment.defined_range().start());
                *t = *segment.defined_range().end();
                Some(segment)
            })
            .collect(),
    )
    .map(|segments| CompoundPath {
        segments,
        _phantom: PhantomData,
    })
    .ok_or(NotAtLeastOnePairOfStatesError)
}

impl<State, Subpath: ParametrizedPath<State>> ParametrizedPath<State>
    for CompoundPath<State, Subpath>
{
    fn defined_range(&self) -> RangeInclusive<f64> {
        *self.segments.first().defined_range().start()..=*self.segments.last().defined_range().end()
    }

    fn sample(&self, t: f64) -> Result<State, OutOfRangeError> {
        self.segments
            .binary_search_by(|seg| {
                if t < *seg.defined_range().start() {
                    Ordering::Greater
                } else if *seg.defined_range().end() < t {
                    Ordering::Less
                } else {
                    Ordering::Equal
                }
            })
            .map_err(|_| OutOfRangeError)
            .and_then(|seg| {
                assert!(self.segments[seg].defined_range().contains(&t));
                self.segments[seg].sample(t)
            })
    }
}

#[cfg(test)]
pub(crate) mod test_utilities {
    use crate::path::ParametrizedPath;
    use crate::state::Distance;

    pub(crate) fn check_path_smooth<State, Path>(path: &Path)
    where
        State: Distance<State, DistanceValue = f64>,
        Path: ParametrizedPath<State>,
    {
        let steps = std::cmp::max(path.defined_range().end().round() as usize * 100, 1000);

        for i in 0..steps {
            let t1 = path.defined_range().end() * (i as f64 / steps as f64);
            let t2 = path.defined_range().end() * ((i + 1) as f64 / steps as f64);

            let st1 = path.sample(t1).unwrap();
            let st2 = path.sample(t2).unwrap();

            assert!(crate::state::Distance::distance(&st1, &st2) < 0.1);
        }
    }

    pub(crate) fn check_path_end_matches<State, Path>(end: State, path: &Path)
    where
        State: Distance<State, DistanceValue = f64>,
        Path: ParametrizedPath<State>,
    {
        let computed_end = path
            .sample(*path.defined_range().end())
            .expect("Should not give out-of-range.");

        assert!(end.distance(&computed_end) < 1.0e-5);
    }

    pub(crate) fn check_path_start_matches<State, Path>(begin: State, path: &Path)
    where
        State: Distance<State, DistanceValue = f64>,
        Path: ParametrizedPath<State>,
    {
        let computed_start = path
            .sample(*path.defined_range().start())
            .expect("Should not give out-of-range.");

        assert!(begin.distance(&computed_start) < 1.0e-10);
    }
}

#[cfg(test)]
mod tests {
    use crate::path::{LerpPathDiscretePath, NonEmptyPointsPath, ParametrizedPath, TimedState};
    use core::iter::once;
    use nonempty::NonEmpty;

    use rand::distributions::Uniform;
    use rand::{thread_rng, Rng};

    #[test]
    fn test_identity() {
        let distr = Uniform::new_inclusive(-100.0, 100.0);

        let mut states = once(-100.0)
            .chain(thread_rng().sample_iter(distr).take(1000))
            .chain(once(100.0))
            .map(|t| TimedState(t, t))
            .collect::<Vec<_>>();

        states.sort_by(|a, b| a.0.partial_cmp(&b.1).unwrap());

        let path = NonEmptyPointsPath {
            states: NonEmpty::from_vec(states).unwrap(),
        };

        assert_eq!(-100.0..=100.0, LerpPathDiscretePath(&path).defined_range());

        for _ in 0..100 {
            let t = thread_rng().sample(distr);

            let sample = LerpPathDiscretePath(&path)
                .sample(t)
                .expect("All samples should be in range.");

            assert!((t - sample).abs() < 1.0e-10);
        }
    }
}
