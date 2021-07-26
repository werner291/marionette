use crate::path::{OutOfRangeError, ParametrizedPath};
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2};
use std::f64::consts::PI;
use std::ops::RangeInclusive;
use ordered_float::OrderedFloat;
use crate::state::rigid_2d::RigidBodyState2;
use proptest::arbitrary::Arbitrary;
use std::iter::IntoIterator;
use num_traits::Pow;
use std::option::Option;

#[derive(Debug)]
enum StraightlineArcPathSegment {
    StraightLine {
        from_point: Isometry2<f64>,
        distance: f64,
    },
    CircularArc {
        from_point: Isometry2<f64>,
        radius: f64,
        angle: f64,
    },
}

impl StraightlineArcPathSegment {
    fn length(&self) -> f64 {
        match self {
            Self::StraightLine { distance, .. } => *distance,
            Self::CircularArc { radius, angle, .. } => radius.abs() * angle.abs(),
        }
    }

    fn sample_at(&self, t: f64) -> Isometry2<f64> {
        match self {
            Self::StraightLine {
                distance: _,
                from_point,
            } => from_point * Translation2::from(Vector2::new(0.0, t)),
            Self::CircularArc {
                angle,
                radius,
                from_point,
            } => {
                let mut transform = from_point.clone();
                transform.append_rotation_wrt_point_mut(
                    &Rotation2::new(-t / radius).into(),
                    &(&transform * Point2::new(*radius, 0.0)),
                );
                transform
            }
        }
    }
}

#[derive(Debug)]
pub struct DubinsMotion {
    start_time: f64,
    start_point: Isometry2<f64>,
    segments: [StraightlineArcPathSegment; 3],
}

impl ParametrizedPath<RigidBodyState2> for DubinsMotion {
    fn defined_range(&self) -> RangeInclusive<f64> {
        self.start_time
            ..=self
            .segments
            .iter()
            .map(StraightlineArcPathSegment::length)
            .sum::<f64>()
            + self.start_time
    }

    fn sample(&self, mut t: f64) -> Result<RigidBodyState2, OutOfRangeError> {
        t -= self.start_time;

        if t < 0.0 {
            return Err(OutOfRangeError);
        }

        for seg_i in 0..self.segments.len() {
            let seg = &self.segments[seg_i];
            if seg_i + 1 == self.segments.len() || t <= seg.length() {
                if t > seg.length() + 0.01 {
                    print!("{}", t);
                }

                return Ok(RigidBodyState2(seg.sample_at(t)));
            } else {
                t -= seg.length();
            }
        }

        return Err(OutOfRangeError);
    }
}

#[derive(Debug, Clone, Copy)]
pub enum LeftRight {
    Left,
    Right,
}

pub fn outer_tangent_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    turn_direction: LeftRight,
) -> DubinsMotion {
    let radius = match turn_direction {
        LeftRight::Left => -1.0,
        LeftRight::Right => 1.0
    };

    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    let right_center = &from.0 * Point2::new(radius, 0.0);
    let right_center2 = &to.0 * Point2::new(radius, 0.0);

    // RSR
    let right_right_delta = right_center2 - right_center;

    let right_right_turnangle_1 =
        wrap_negative(radius.signum() * Rotation2::rotation_between(&right_right_delta, &from_heading).angle());

    assert!(right_right_turnangle_1 >= 0.0);

    let right_right_turnangle_2 =
        wrap_negative(radius.signum() * Rotation2::rotation_between(&to_heading, &right_right_delta).angle());

    assert!(right_right_turnangle_2 >= 0.0);

    let mut transform_along_path = from.0.clone();

    let path = DubinsMotion {
        start_time,
        start_point: from.0.clone(),
        segments: [
            {
                let arc = StraightlineArcPathSegment::CircularArc {
                    from_point: { transform_along_path.clone() },
                    radius,
                    angle: right_right_turnangle_1,
                };

                transform_along_path = arc.sample_at(arc.length());

                arc
            },
            {
                let line = StraightlineArcPathSegment::StraightLine {
                    from_point: transform_along_path.clone(),
                    distance: right_right_delta.norm(),
                };

                transform_along_path = line.sample_at(line.length());

                line
            },
            StraightlineArcPathSegment::CircularArc {
                from_point: transform_along_path,
                radius,
                angle: right_right_turnangle_2,
            },
        ],
    };

    path
}

pub fn inner_tangent_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    first_turn_direction: LeftRight,
) -> Option<DubinsMotion> {
    let radius = match first_turn_direction {
        LeftRight::Left => -1.0,
        LeftRight::Right => 1.0
    };

    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    let right_center = &from.0 * Point2::new(radius, 0.0);
    let right_center2 = &to.0 * Point2::new(-radius, 0.0);

    // RSR
    let right_right_delta = &right_center2 - &right_center;

    if right_right_delta.norm() < radius * 2.0 {
        None
    } else {
        let tangent_from_midline_angle = (radius / (right_right_delta.norm() / 2.0)).acos();
        let tangent_point_1 = right_center + Rotation2::new(tangent_from_midline_angle) * right_right_delta.normalize() * radius;
        let tangent_point_2 = right_center2 + Rotation2::new(tangent_from_midline_angle) * (-right_right_delta.normalize()) * radius;
        let tangent_delta = tangent_point_2 - tangent_point_1;

        let right_right_turnangle_1 =
            wrap_negative(radius.signum() * Rotation2::rotation_between(&tangent_delta, &from_heading).angle());

        assert!(right_right_turnangle_1 >= 0.0);

        let right_right_turnangle_2 =
            wrap_negative((-radius).signum() * Rotation2::rotation_between(&to_heading, &tangent_delta).angle());

        assert!(right_right_turnangle_2 >= 0.0);

        let mut transform_along_path = from.0.clone();

        let path = DubinsMotion {
            start_time,
            start_point: from.0.clone(),
            segments: [
                {
                    let arc = StraightlineArcPathSegment::CircularArc {
                        from_point: { transform_along_path.clone() },
                        radius,
                        angle: right_right_turnangle_1,
                    };

                    transform_along_path = arc.sample_at(arc.length());

                    arc
                },
                {
                    let line = StraightlineArcPathSegment::StraightLine {
                        from_point: transform_along_path.clone(),
                        distance: tangent_delta.norm(),
                    };

                    transform_along_path = line.sample_at(line.length());

                    line
                },
                StraightlineArcPathSegment::CircularArc {
                    from_point: transform_along_path,
                    radius: -radius,
                    angle: right_right_turnangle_2,
                },
            ],
        };

        Some(path)
    }
}

pub fn compute_dubins_motion(from: &RigidBodyState2,
                             to: &RigidBodyState2,
                             start_time: f64) -> DubinsMotion {
    use core::iter::once;

    once(outer_tangent_motion(from, to, start_time, LeftRight::Right))
        .chain(once(outer_tangent_motion(from, to, start_time, LeftRight::Left)))
        .min_by_key(|motion| OrderedFloat(*motion.defined_range().end())).unwrap()
}

fn wrap_negative(a: f64) -> f64 {
    if a < 0.0 {
        2.0 * PI + a
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dubins::{DubinsMotion};
    use crate::path::ParametrizedPath;
    use crate::state::rigid_2d::RigidBodyState2;
    use crate::state::Distance;
    use nalgebra::{Isometry2, Vector2, UnitComplex};

    use rand::{Rng};
    use proptest::prelude::*;
    use std::f64::consts::PI;

    fn arbitrary_left_right() -> impl Strategy<Value=LeftRight> {
        prop_oneof![
            Just(LeftRight::Left),
            Just(LeftRight::Right)
        ]
    }

    fn arbitrary_dubins_state() -> impl Strategy<Value=RigidBodyState2> {
        (
            -1000.0..=1000.0,
            -1000.0..=1000.0,
            -PI..=PI
        ).prop_map(|(x, y, angle)| {
            RigidBodyState2(Isometry2::new(
                Vector2::new(x, y),
                angle,
            ))
        })
    }

    proptest! {
        #[test]
        fn test_inner_tangent(start_point in arbitrary_dubins_state(),
                              end_point in arbitrary_dubins_state(),
                                steering_direction in arbitrary_left_right()) {
            if let Some(motion) = inner_tangent_motion(&start_point, &end_point, 0.0, steering_direction) {
                check_dubins_motion(start_point, end_point, motion);
            }
        }

                #[test]
        fn test_outer_tangent(start_point in arbitrary_dubins_state(),
                                    end_point in arbitrary_dubins_state(),
                                    steering_direction in arbitrary_left_right()) {
            check_dubins_motion(start_point, end_point,
                outer_tangent_motion(&start_point, &end_point, 0.0, steering_direction));
        }
    }

    fn check_dubins_motion(start_point: RigidBodyState2, end_point: RigidBodyState2, motion: DubinsMotion) {
        assert_eq!(0.0, *motion.defined_range().start());
        let computed_start = motion
            .sample(*motion.defined_range().start())
            .expect("Should not give out-of-range.");

        assert!(start_point.distance(&computed_start) < 1.0e-10);
        let computed_end = motion
            .sample(*motion.defined_range().end())
            .expect("Should not give out-of-range.");

        assert!(end_point.distance(&computed_end) < 1.0e-5);

        let steps = std::cmp::max(motion.defined_range().end().round() as usize * 100, 1000);

        for i in 0..steps {
            let t1 = motion.defined_range().end() * (i as f64 / steps as f64);
            let t2 = motion.defined_range().end() * ((i + 1) as f64 / steps as f64);

            let st1 = motion.sample(t1).unwrap();
            let st2 = motion.sample(t2).unwrap();

            let delta: Vector2<f64> =
                (st2.0.translation.vector - st1.0.translation.vector).into();
            let forward = st1.0.rotation * Vector2::new(0.0, 1.0);

            assert!(st1.distance(&st2) < 0.1);
            assert!(delta.dot(&forward) > 0.0 || delta.norm() < 1.0e-5);
        }
    }
}
