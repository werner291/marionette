//! A "dubins car" is a model of a vehicle, posessing a location and rotation in the plane,
//! that can only travel either straight forward, or turn left or right with a given maximum
//! turn radius.
//!
//! This module relates to planning "dubins motions": paths consisting of circular arcs and
//! straight lines between two possible states of the car. This effectively provides a steering
//! function for higher-level, obstacle-avoiding planners to use.

use crate::path::{OutOfRangeError, ParametrizedPath};
use crate::state::rigid_2d::RigidBodyState2;
use crate::state::LinearInterpolate;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2};
use num_traits::Pow;
use ordered_float::OrderedFloat;
use std::clone::Clone;
use std::convert::{From, Into};
use std::f64::consts::PI;
use std::iter::IntoIterator;
use std::ops::RangeInclusive;
use std::option::Option;
use std::option::Option::{None, Some};
use std::result::Result;
use std::result::Result::{Err, Ok};

/// Either a straight line, or a circular arc, originating at some point in space with a given direction.
/// For the circular arc, note that the radius encodes the turn direction.
#[derive(Debug)]
enum StraightlineArcPathSegment {
    StraightLine {
        /// This path segment starts with the car at the given position, traveling in the positive y-direction.
        from_point: Isometry2<f64>,
        /// The car travels this distance to complete the motion.
        distance: f64,
    },
    CircularArc {
        /// This path segment starts with the car at the given position, traveling in the positive y-direction.
        from_point: Isometry2<f64>,
        /// Signed radius of the turn. Positive turns right, negative turns left.
        signed_radius: f64,
        /// The angle with which to turn the car to complete the motion, in radians. Always non-negative.
        angle: f64,
    },
}

impl StraightlineArcPathSegment {
    /// The length of the path
    fn length(&self) -> f64 {
        match self {
            Self::StraightLine { distance, .. } => *distance,
            Self::CircularArc {
                signed_radius: radius,
                angle,
                ..
            } => radius.abs() * angle.abs(),
        }
    }

    /// Considering the path as a function that takes a point in time, and returns a position and rotation for the car, compute the position of the car at that point in time.
    ///
    /// t should be a value between 0 and whatever length() returns. Returned values outside that range will be continuations of the motion, but are largely undefined.
    fn sample_at(&self, t: f64) -> Isometry2<f64> {
        match self {
            Self::StraightLine {
                distance: _, // length() already takes this into account
                from_point,
            } => from_point * Translation2::from(Vector2::new(0.0, t)),
            Self::CircularArc {
                angle, // length() already takes this into account
                signed_radius: radius,
                from_point,
            } => {
                let mut transform = from_point.clone();
                // Rotate around a point (radius,0) or (-radius,0) w.r.t the start transform.
                transform.append_rotation_wrt_point_mut(
                    &Rotation2::new(-t / radius).into(),
                    &(&transform * Point2::new(*radius, 0.0)),
                );
                transform
            }
        }
    }
}

/// Represents a three-segment dubins motion.
///
/// TODO: Perhaps this should simply be a CompoundPath?
#[derive(Debug)]
pub struct DubinsMotion {
    /// An absolute time point at which this motion starts, to comply with the ParametrizedPath trait.
    start_time: f64,
    /// Exacty three segments, which is enough to represent any shortest dubins path. Segments may be zero-length.
    segments: [StraightlineArcPathSegment; 3],
}

impl ParametrizedPath<RigidBodyState2> for DubinsMotion {
    fn defined_range(&self) -> RangeInclusive<f64> {
        // Simply add up the three segment lengths, and adjust with the start time.
        self.start_time
            ..=self
                .segments
                .iter()
                .map(StraightlineArcPathSegment::length)
                .sum::<f64>()
                + self.start_time
    }

    fn sample(&self, mut t: f64) -> Result<RigidBodyState2, OutOfRangeError> {
        // Remove start time, to get time relative to start
        t -= self.start_time;

        if t < 0.0 {
            return Err(OutOfRangeError);
        }

        // Linear search is fine, since we have O(1) segments.
        for seg_i in 0..self.segments.len() {
            let seg = &self.segments[seg_i];
            if seg_i + 1 == self.segments.len() || t <= seg.length() {
                // We use the current segment if it is either the one that `t` fits into, or it's the last one.
                // The "or it's the last one" is to deal with rounding errors that put `t` just beyond the end of the segment.
                // Note that, while `t` is not supposed to be beyond the defined range of the motion, it won't break anything if it is.
                return Ok(RigidBodyState2(seg.sample_at(t)));
            } else {
                // Keep removing the previous segment length to get the sample relative to the start of the next segment.
                t -= seg.length();
            }
        }

        // Unreachable, but putting this in here just in case.
        return Err(OutOfRangeError);
    }
}

/// Used to distinguish between turning eiter left or right on the first turn of the Dubins motion.
#[derive(Debug, Clone, Copy)]
pub enum LeftRight {
    Left,
    Right,
}

/// Compute a Dubins motion along the outer common tangents of the two circles.
/// This motion is always defined (sometimes degenerate), but it can be overly long.
pub fn outer_tangent_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    turn_direction: LeftRight,
    radius: f64,
) -> DubinsMotion {
    // Compute the signed radius based on whether we turn left or right.
    let signed_radius = match turn_direction {
        LeftRight::Left => -radius,
        LeftRight::Right => radius,
    };

    // Compute the heading vector at the start and end position.
    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    // Compute the center points of the circular arcs that the motion starts and ends with;
    // these are points (radius,0) relative to the start/end transforms.
    let circle_center_1 = &from.0 * Point2::new(signed_radius, 0.0);
    let circle_center_2 = &to.0 * Point2::new(signed_radius, 0.0);

    // Compute the vector of the straight-line motion, which is the same as the vector between the centers.
    let straight_section_delta = circle_center_2 - circle_center_1;

    // Compute the angle of the first arc before the car starts on the straight piece,
    // wrapped such that the angle is always positive (a Dubins car cannot go backward)
    let angle_1 = wrap_positive(
        signed_radius.signum()
            * Rotation2::rotation_between(&straight_section_delta, &from_heading).angle(),
    );

    // Same, but for the second arc.
    let angle_2 = wrap_positive(
        signed_radius.signum()
            * Rotation2::rotation_between(&to_heading, &straight_section_delta).angle(),
    );

    // Keep track of the transform as it is swept through the path, piece by piece.
    let mut transform_along_path = from.0.clone();

    // Assemble the resulting three-section Dubins motion.
    arc_straight_arc_motion(
        start_time,
        transform_along_path,
        signed_radius,
        angle_1,
        straight_section_delta.norm(),
        signed_radius,
        angle_2,
    )
}

/// Utility function that builds a DubinsMotion consisting of a circular arc, a straight line, then a circular arc.
fn arc_straight_arc_motion(
    start_time: f64,
    start_transform: Isometry2<f64>,
    signed_radius: f64,
    angle_1: f64,
    straight_distance: f64,
    signed_radius2: f64,
    angle_2: f64,
) -> DubinsMotion {
    let mut transform_along_path = start_transform;

    DubinsMotion {
        start_time,
        segments: [
            {
                let arc = StraightlineArcPathSegment::CircularArc {
                    from_point: { transform_along_path.clone() },
                    signed_radius,
                    angle: angle_1,
                };

                transform_along_path = arc.sample_at(arc.length());

                arc
            },
            {
                let line = StraightlineArcPathSegment::StraightLine {
                    from_point: transform_along_path.clone(),
                    distance: straight_distance,
                };

                transform_along_path = line.sample_at(line.length());

                line
            },
            StraightlineArcPathSegment::CircularArc {
                from_point: transform_along_path,
                signed_radius: signed_radius2,
                angle: angle_2,
            },
        ],
    }
}

/// Compute a Dubins motion along one of the inner tangents between the two circles.
/// This motion is not defined when the circles overlap, in which case None is returned.
pub fn inner_tangent_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    first_turn_direction: LeftRight,
    radius: f64,
) -> Option<DubinsMotion> {
    // Compute the signed radius based on whether we turn left or right.
    let signed_radius = match first_turn_direction {
        LeftRight::Left => -radius,
        LeftRight::Right => radius,
    };

    // Compute the heading vector at the start and end position.
    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    // Compute the center points of the circular arcs.
    let circle_center_1 = &from.0 * Point2::new(signed_radius, 0.0);
    let circle_center_2 = &to.0 * Point2::new(-signed_radius, 0.0);

    // Compute the vector between the two circe centers.
    let circle_centers_delta = &circle_center_2 - &circle_center_1;

    // If the circles intersect, the inner common tangents are non-existant.
    if circle_centers_delta.norm() < radius * 2.0 {
        None
    } else {
        // Compute the distance between the midway point between the two circle centers, and
        // the tangent point at which the Dubins transitions from a circular motion to a straight motion.
        let tangent_from_midline_angle =
            (signed_radius / (circle_centers_delta.norm() / 2.0)).acos();
        // Compute the start/end points of the straight line.
        let tangent_point_1 = circle_center_1
            + Rotation2::new(tangent_from_midline_angle)
                * circle_centers_delta.normalize()
                * signed_radius;
        let tangent_point_2 = circle_center_2
            + Rotation2::new(tangent_from_midline_angle)
                * (-circle_centers_delta.normalize())
                * signed_radius;
        // Compute the movement vector of the straight line.
        let tangent_delta = tangent_point_2 - tangent_point_1;

        // Compute the arc angles of the circular arc motions.
        let arc_angle_1 = wrap_positive(
            signed_radius.signum()
                * Rotation2::rotation_between(&tangent_delta, &from_heading).angle(),
        );

        assert!(arc_angle_1 >= 0.0);

        // Compute the arc angles of the circular arc motions.
        let arc_angle_2 = wrap_positive(
            (-signed_radius).signum()
                * Rotation2::rotation_between(&to_heading, &tangent_delta).angle(),
        );

        assert!(arc_angle_2 >= 0.0);

        Some(arc_straight_arc_motion(
            start_time,
            from.0.clone(),
            signed_radius,
            arc_angle_1,
            tangent_delta.norm(),
            -signed_radius,
            arc_angle_2,
        ))
    }
}

/// Compute a Dubins motion along three circular arcs, where the middle arc has a curvature opposite
/// to the first and last arc.
///
/// This motion is undefined if the two circles are more than twice the defined radius apart.
pub fn three_circular_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    first_turn_direction: LeftRight,
    radius: f64,
) -> Option<DubinsMotion> {
    let signed_radius = match first_turn_direction {
        LeftRight::Left => -radius,
        LeftRight::Right => radius,
    };

    // Compute the heading vector at the start and end position.
    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    // Compute the center points of the circular arcs that the motion starts and ends with;
    // these are points (radius,0) relative to the start/end transforms.
    let circle_center_1 = &from.0 * Point2::new(signed_radius, 0.0);
    let circle_center_2 = &to.0 * Point2::new(signed_radius, 0.0);

    // Compute the vector between the two circe centers.
    let circle_center_delta = &circle_center_2 - &circle_center_1;

    // The motion does not exist if the circles are more than four times the radius apart.
    if circle_center_delta.norm() > 4.0 * signed_radius.abs() {
        None
    } else {
        // Midpoint between the circle centers.
        let midpoint = circle_center_1.linear_interpolate(&circle_center_2, 0.5);

        // Distance between middle circle center and midpoint, using Pythagorean theorem.
        let thirdpoint_from_midpoint = {
            let a: f64 = (2.0 * signed_radius.abs()).pow(2);
            let b: f64 = (circle_center_delta.norm() / 2.0).pow(2);
            (a - b).sqrt()
        };

        // Obtain the actual center point through a vector perpendicular to the delta.
        let third_circle_center = midpoint
            + Rotation2::new(-PI / 2.0)
                * circle_center_delta.normalize()
                * thirdpoint_from_midpoint;

        // Obtain heading vectors of the Dubins car at the points where the circles meet,
        // which are always perpendicular to the lines between the circle centers.
        let tangent_vector_1 = Rotation2::new(-signed_radius.signum() * PI / 2.0)
            * (third_circle_center - circle_center_1);
        let tangent_vector_2 = Rotation2::new(-signed_radius.signum() * PI / 2.0)
            * (third_circle_center - circle_center_2);

        // Obtain the sweep angles of all three arcs.
        let angle_1 = wrap_positive(
            signed_radius.signum()
                * Rotation2::rotation_between(&tangent_vector_1, &from_heading).angle(),
        );
        let angle_2 = wrap_positive(
            signed_radius.signum()
                * Rotation2::rotation_between(&to_heading, &tangent_vector_2).angle(),
        );
        let angle_middle = wrap_positive(
            signed_radius.signum()
                * Rotation2::rotation_between(&tangent_vector_1, &tangent_vector_2).angle(),
        );

        let mut transform_along_path = from.0.clone();

        // Build the resulting DubinsMotion.
        let path = DubinsMotion {
            start_time,
            segments: [
                {
                    let arc = StraightlineArcPathSegment::CircularArc {
                        from_point: { transform_along_path.clone() },
                        signed_radius,
                        angle: angle_1,
                    };

                    transform_along_path = arc.sample_at(arc.length());

                    arc
                },
                {
                    let arc = StraightlineArcPathSegment::CircularArc {
                        from_point: { transform_along_path.clone() },
                        signed_radius: -signed_radius,
                        angle: angle_middle,
                    };

                    transform_along_path = arc.sample_at(arc.length());

                    arc
                },
                StraightlineArcPathSegment::CircularArc {
                    from_point: transform_along_path,
                    signed_radius,
                    angle: angle_2,
                },
            ],
        };

        Some(path)
    }
}

/// Build the minimal dubins path between two [`RigidBodyState2`]s.
pub fn shortest_dubins_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
    radius: f64,
) -> DubinsMotion {
    use core::iter::once;

    // The minimal Dubins motion is always one of six possible cases.
    once(outer_tangent_motion(
        from,
        to,
        start_time,
        LeftRight::Right,
        radius,
    ))
    .chain(once(outer_tangent_motion(
        from,
        to,
        start_time,
        LeftRight::Left,
        radius,
    )))
    .chain(inner_tangent_motion(from, to, start_time, LeftRight::Left, radius).into_iter())
    .chain(inner_tangent_motion(from, to, start_time, LeftRight::Right, radius).into_iter())
    .chain(three_circular_motion(from, to, start_time, LeftRight::Left, radius).into_iter())
    .chain(three_circular_motion(from, to, start_time, LeftRight::Right, radius).into_iter())
    .min_by_key(|motion| OrderedFloat(*motion.defined_range().end()))
    .unwrap()
}

/// Wrap an angle value such that it is positive.
fn wrap_positive(a: f64) -> f64 {
    if a < 0.0 {
        2.0 * PI + a
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dubins::DubinsMotion;
    use crate::path::ParametrizedPath;
    use crate::state::rigid_2d::RigidBodyState2;
    use crate::state::Distance;
    use nalgebra::{Isometry2, UnitComplex, Vector2};

    use crate::dubins::LeftRight::Left;
    use proptest::prelude::*;
    use rand::Rng;
    use std::f64::consts::PI;

    fn arbitrary_left_right() -> impl Strategy<Value = LeftRight> {
        prop_oneof![Just(LeftRight::Left), Just(LeftRight::Right)]
    }

    fn arbitrary_dubins_state(square_radius: f64) -> impl Strategy<Value = RigidBodyState2> {
        (
            -square_radius..=square_radius,
            -square_radius..=square_radius,
            -PI..=PI,
        )
            .prop_map(|(x, y, angle)| RigidBodyState2(Isometry2::new(Vector2::new(x, y), angle)))
    }

    proptest! {
        #[test]
        fn test_inner_tangent(
            start_point in arbitrary_dubins_state(100.0),
                              end_point in arbitrary_dubins_state(100.0),
                                steering_direction in arbitrary_left_right(),
                                radius in 0.5 ..= 2.0) {
            if let Some(motion) = inner_tangent_motion(&start_point, &end_point, 0.0, steering_direction, radius) {
                check_dubins_motion(start_point, end_point, motion);
            }
        }

        #[test]
        fn test_outer_tangent(start_point in arbitrary_dubins_state(100.0),
                                    end_point in arbitrary_dubins_state(100.0),
                                    steering_direction in arbitrary_left_right(),
        radius in 0.5 ..= 2.0) {
            check_dubins_motion(start_point, end_point,
                outer_tangent_motion(&start_point, &end_point, 0.0, steering_direction, radius));
        }

        #[test]
        fn test_threecircle_motion(start_point in arbitrary_dubins_state(2.5),
                                    end_point in arbitrary_dubins_state(2.5),
                                    steering_direction in arbitrary_left_right(),
        radius in 0.5 ..= 2.0) {
            if let Some(motion) = three_circular_motion(&start_point, &end_point, 0.0, steering_direction, radius) {
                check_dubins_motion(start_point, end_point, motion);
            }
        }

                #[test]
        fn test_full_dubins(start_point in arbitrary_dubins_state(100.0),
                                    end_point in arbitrary_dubins_state(100.0),
                                    steering_direction in arbitrary_left_right(),
        radius in 0.5 ..= 2.0) {
            let motion = shortest_dubins_motion(&start_point, &end_point, 0.0, radius);
                check_dubins_motion(start_point, end_point, motion);

        }
    }

    /// Checks to make sure that the provided [`DubinsMotion`] fulfills the following criteria:
    /// - It starts at the start point (according to defined_range())
    /// - It ends at the end point (according to defined_range())
    /// - It is continuous (contains no large discontinuities)
    /// - The car never moves backward.
    fn check_dubins_motion(
        start_point: RigidBodyState2,
        end_point: RigidBodyState2,
        motion: DubinsMotion,
    ) {
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

            let delta: Vector2<f64> = (st2.0.translation.vector - st1.0.translation.vector).into();
            let forward = st1.0.rotation * Vector2::new(0.0, 1.0);

            // Note that "distance" includes angular distance.
            assert!(st1.distance(&st2) < 0.1);
            assert!(delta.dot(&forward) > 0.0 || delta.norm() < 1.0e-5);
        }
    }
}
