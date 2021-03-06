use crate::goal::{ExactGoal, PredicateGoal};
use crate::motion_validation::MotionValidator;
use crate::path::NonEmptyPointsPath;
use acap::vp::VpTree;
use acap::{NearestNeighbors, Proximity};
use nonempty::NonEmpty;

use std::option::Option;
use std::prelude::v1::*;
use std::rc::Rc;

// A small struct containing a State, and an RC to its parent.
struct StateWithParent<State> {
    state: State,
    parent: Option<Rc<StateWithParent<State>>>,
}

impl<State: Copy> StateWithParent<State> {
    fn retrace(self: Rc<Self>) -> impl Iterator<Item = State> {
        itertools::unfold(Some(self), |optst| {
            if let Some(st) = optst.take() {
                let to_return = st.state;
                *optst = st.parent.clone();

                Some(to_return)
            } else {
                None
            }
        })
    }
}

// A wrapper such that we can implement Proximity on it.
#[derive(Clone)]
struct StateProximity<State>(Rc<StateWithParent<State>>);

impl<State: Proximity> Proximity for StateProximity<State> {
    type Distance = State::Distance;

    fn distance(&self, other: &Self) -> Self::Distance {
        self.0.state.distance(&other.0.state)
    }
}

pub fn rrt_connect<State>(
    start_state: State,
    is_valid_transition: impl MotionValidator<State>,
    goal: ExactGoal<State>,
    mut sample_state: impl FnMut() -> State,
) -> NonEmptyPointsPath<State>
where
    State: Proximity + Copy,
{
    // Nearest-neighbour lookup tree
    let mut lookup_nearest_fromstart: VpTree<StateProximity<State>> = VpTree::new();

    lookup_nearest_fromstart.push(StateProximity(Rc::new(StateWithParent {
        state: start_state,
        parent: None,
    })));

    // Nearest-neighbour lookup tree
    let mut lookup_nearest_fromgoal: VpTree<StateProximity<State>> = VpTree::new();

    lookup_nearest_fromgoal.push(StateProximity(Rc::new(StateWithParent {
        state: goal.goal,
        parent: None,
    })));

    loop {
        // Draw a new state from the sampler.
        let sample = sample_state();

        // Look up the nearest existing sample
        let from_start_closest = lookup_nearest_fromstart
            .nearest(&StateProximity(Rc::new(StateWithParent {
                parent: None, // This field is kinda ugly, can it be removed somehow?
                state: sample,
            })))
            .expect("Tree should be non-empty.")
            .item
            .0
            .clone();

        let is_valid_fromstart =
            is_valid_transition.validate_motion(&from_start_closest.state, &sample);

        // Look up the nearest existing sample
        let from_goal_closest = lookup_nearest_fromgoal
            .nearest(&StateProximity(Rc::new(StateWithParent {
                parent: None, // This field is kinda ugly, can it be removed somehow?
                state: sample,
            })))
            .expect("Tree should be non-empty.")
            .item
            .0
            .clone();

        let is_valid_fromgoal =
            is_valid_transition.validate_motion(&sample, &from_goal_closest.state);

        if is_valid_fromstart && is_valid_fromgoal {
            // Backtrack through the tree and find the path
            let mut result = vec![];

            result.extend(from_start_closest.retrace());

            result.reverse();

            result.push(sample);

            result.extend(from_goal_closest.retrace());

            return NonEmptyPointsPath {
                states: NonEmpty::from_vec(result).expect("RRTConnect never makes empty paths."),
            };
        } else {
            if is_valid_fromstart {
                lookup_nearest_fromstart.push(StateProximity(Rc::new(StateWithParent {
                    state: sample,
                    parent: Some(from_start_closest),
                })));
            }
            if is_valid_fromgoal {
                lookup_nearest_fromgoal.push(StateProximity(Rc::new(StateWithParent {
                    state: sample,
                    parent: Some(from_goal_closest),
                })));
            }
        }
    }
}

pub fn rrt<State>(
    start_state: State,
    is_valid_transition: impl MotionValidator<State>,
    goal: impl PredicateGoal<State>,
    mut sample_state: impl FnMut() -> State,
) -> NonEmptyPointsPath<State>
where
    State: Proximity + Copy,
{
    // Keep track of node inserted last
    let mut last_inserted = Rc::new(StateWithParent {
        state: start_state,
        parent: None,
    });

    // Nearest-neighbour lookup tree
    let mut lookup_nearest: VpTree<StateProximity<State>> = VpTree::new();

    // Wasn't the goal: put it into the tree to serve as an intermediate state.
    lookup_nearest.push(StateProximity(last_inserted.clone()));

    // Keep going until the last sample was the goal.
    while !goal.is_goal(&last_inserted.state) {
        // Draw a new state from the sampler.
        let sample = sample_state();

        // Look up the nearest existing sample
        let parent = lookup_nearest
            .nearest(&StateProximity(Rc::new(StateWithParent {
                parent: None, // This field is kinda ugly, can it be removed somehow?
                state: sample,
            })))
            .expect("Tree should be non-empty.")
            .item
            .0
            .clone();

        if is_valid_transition.validate_motion(&parent.state, &sample) {
            // Update last_inserted
            last_inserted = Rc::new(StateWithParent {
                parent: Some(parent),
                state: sample,
            });

            // Wasn't the goal: put it into the tree to serve as an intermediate state.
            lookup_nearest.push(StateProximity(last_inserted.clone()));
        }
    }

    let mut result: Vec<State> = last_inserted.retrace().collect();
    result.reverse();

    return NonEmptyPointsPath {
        states: NonEmpty::from_vec(result).expect("RRT never produces empty paths."),
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion_validation::UniformLinearInterpolatedSamplingValidator;
    use crate::path::{parametrize_by_distance, LerpPathDiscretePath, ParametrizedPath};
    use crate::point_state::PointState;
    use nalgebra::Point2;
    use rand::{thread_rng, Rng};

    #[test]
    fn rrt_1d() {
        #[derive(Clone, Copy, PartialEq, Debug)]
        struct SimpleState(f64);
        use acap::Proximity;

        impl Proximity for SimpleState {
            type Distance = f64;

            fn distance(&self, other: &Self) -> Self::Distance {
                (self.0 - other.0).abs()
            }
        }

        let start = SimpleState(0.0);
        let end = SimpleState(42.0);

        let mut rng = thread_rng();

        let is_valid_transition = move |_from: &SimpleState, _to: &SimpleState| true;

        let goal = ExactGoal { goal: end };

        let result = rrt(start, &is_valid_transition, goal, || {
            if rng.gen_bool(0.05) {
                end
            } else {
                SimpleState(rng.gen_range(-100.0..100.0))
            }
        });

        assert_eq!(&start, result.states.first());
        assert_eq!(&end, result.states.last());

        let result = rrt_connect(start, &is_valid_transition, goal, || {
            SimpleState(rng.gen_range(-100.0..100.0))
        });

        assert_eq!(&start, result.states.first());
        assert_eq!(&end, result.states.last());
    }

    #[test]
    fn rrt_connect_arounddisk() {
        for _ in 0..50 {
            let begin = PointState(Point2::new(-10.0, 0.0));
            let end = PointState(Point2::new(10.0, 0.0));

            fn validate_state(state: &PointState<2>) -> bool {
                nalgebra::distance(&state.0, &Point2::new(0.0, 0.0)) > 5.0
            }

            let transition_validator =
                UniformLinearInterpolatedSamplingValidator::new(validate_state, 0.1);

            let mut rng = thread_rng();

            let path = rrt_connect(begin, transition_validator, ExactGoal { goal: end }, || {
                PointState(Point2::new(
                    rng.gen_range(-10.0..=10.0),
                    rng.gen_range(-10.0..=10.0),
                ))
            });

            let path = parametrize_by_distance(path, crate::state::Distance::distance, 0.0);

            let linear_path = LerpPathDiscretePath(&path);

            assert_eq!(0.0, *linear_path.defined_range().start());

            crate::path::test_utilities::check_path_start_matches(begin, &linear_path);
            crate::path::test_utilities::check_path_end_matches(end, &linear_path);
            crate::path::test_utilities::check_path_smooth(&linear_path);
        }
    }
}
