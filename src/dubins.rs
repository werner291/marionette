use crate::path::{OutOfRangeError, ParametrizedPath};
use crate::state::rigid_2d::rigid_body_state_2d::RigidBodyState2;
use crate::state::Lerp;
use nalgebra::{Isometry2, Point2, Rotation2, Translation2, Vector2};
use std::f64::consts::PI;
use std::ops::RangeInclusive;

struct DubinsCarConfiguration {
    max_turn_radius: f64,
}

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
                distance,
                from_point,
            } => from_point * Translation2::from(Vector2::new(0.0, t)),
            Self::CircularArc {
                angle,
                radius,
                from_point,
            } => {
                let mut transform = from_point.clone();
                transform.append_rotation_wrt_point_mut(
                    &Rotation2::new(t / radius * angle.signum()).into(),
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

pub fn compute_dubins_motion(
    from: &RigidBodyState2,
    to: &RigidBodyState2,
    start_time: f64,
) -> DubinsMotion {
    let radius = 1.0;
    let from_heading = &from.0.rotation * Vector2::new(0.0, 1.0);
    let to_heading = &to.0.rotation * Vector2::new(0.0, 1.0);

    let right_center = &from.0 * Point2::new(radius, 0.0);
    let right_center2 = &to.0 * Point2::new(radius, 0.0);

    // RSR
    let right_right_delta = right_center2 - right_center;

    let right_right_turnangle_1 = {
        let a = Rotation2::rotation_between(&from_heading, &right_right_delta).angle();
        if a > 0.0 {
            a - 2.0 * PI
        } else {
            a
        }
    };

    let right_right_turnangle_2 = {
        let a = Rotation2::rotation_between(&right_right_delta, &to_heading).angle();
        if a > 0.0 {
            a - 2.0 * PI
        } else {
            a
        }
    };

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

#[cfg(test)]
mod tests {
    use crate::dubins::{compute_dubins_motion, DubinsMotion};
    use crate::path::ParametrizedPath;
    use crate::state::rigid_2d::rigid_body_state_2d::RigidBodyState2;
    use crate::state::Distance;
    use nalgebra::{Isometry2, Translation2, Vector2};
    use rand::prelude::ThreadRng;
    use rand::{thread_rng, Rng};
    use std::f64::consts::PI;

    fn gen_isometry(mut rng: &mut ThreadRng) -> Isometry2<f64> {
        Isometry2::new(
            Vector2::new(rng.gen_range(-100.0..100.0), rng.gen_range(-100.0..100.0)),
            rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI),
        )
    }

    #[test]
    fn dubins_validpath() {
        for _ in 0..100 {
            let mut rng = thread_rng();
            let start_point = RigidBodyState2(gen_isometry(&mut rng));
            let end_point = RigidBodyState2(gen_isometry(&mut rng));

            let motion = compute_dubins_motion(&start_point, &end_point, 0.0);

            for i in 0..=1000 {
                let t1 = motion.defined_range().end() * (i as f64 / 1000.0);
                let st1 = motion.sample(t1).unwrap();

                println!(
                    "{}, {}",
                    st1.0.translation.vector.x, st1.0.translation.vector.y
                );
            }

            assert_eq!(0.0, *motion.defined_range().start());
            let computed_start = motion
                .sample(*motion.defined_range().start())
                .expect("Should not give out-of-range.");

            assert!(start_point.distance(&computed_start) < 1.0e-10);
            let computed_end = motion
                .sample(*motion.defined_range().end())
                .expect("Should not give out-of-range.");

            assert!(dbg!(end_point).distance(&dbg!(computed_end)) < 1.0e-5);

            test_continuous_and_forward(motion);
        }
    }

    fn test_continuous_and_forward(motion: DubinsMotion) {
        for i in 0..10000 {
            let t1 = motion.defined_range().end() * (i as f64 / 10000.0);
            let t2 = motion.defined_range().end() * ((i + 1) as f64 / 10000.0);

            let st1 = motion.sample(t1).unwrap();
            let st2 = motion.sample(t2).unwrap();

            let delta: Vector2<f64> =
                (st2.0.translation.vector - st1.0.translation.vector).into();
            let forward = st1.0.rotation * Vector2::new(0.0, 1.0);

            assert!(st1.distance(&st2) < 0.1);
            assert!(delta.dot(&forward) < 0.0);
        }
    }
}
