mod dubins;
pub mod goal;
pub mod motion_validation;
pub mod path;
pub mod point_state;
pub mod prm;
pub mod rrt;
pub mod state;
pub mod steering;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::goal::ExactGoal;
    use crate::motion_validation::{
        MotionValidator, StateValidator, UniformLinearInterpolatedSamplingValidator,
    };
    use crate::path::{
        build_compound_path_from_steering_function, parametrize_by_distance, CompoundPath,
        LerpPathDiscretePath, OutOfRangeError, ParametrizedPath,
    };
    use crate::rrt::rrt_connect;
    use crate::state::rigid_2d::rigid_body_state_2d::RigidBodyState2;
    use crate::state::{Distance, Lerp};
    use kiss3d::light::Light;
    use kiss3d::nalgebra::UnitQuaternion;
    use kiss3d::window::{State, Window};
    use nalgebra::{Isometry2, Isometry3, Point2, Rotation2, Vector2, Vector3};
    use ncollide2d::shape::Cuboid;
    use rand::{thread_rng, Rng};
    use std::ops::{Not, RangeInclusive};

    #[test]
    fn dubins_parking() {
        let start = RigidBodyState2(Isometry2::new(Vector2::new(5.0, 0.0), 0.0));
        let end = RigidBodyState2(Isometry2::new(Vector2::new(0.0, 5.0), 0.0));

        let cuboid = Cuboid::new(Vector2::new(1.0, 2.0));

        let parked_cars_at = [
            Isometry2::new(Vector2::new(5.0, 5.0), 0.0),
            Isometry2::new(Vector2::new(2.5, 5.0), 0.0),
            Isometry2::new(Vector2::new(-2.5, 5.0), 0.0),
            Isometry2::new(Vector2::new(-5.0, 5.0), 0.0),
        ];

        let validate_state = |state: &RigidBodyState2| {
            parked_cars_at
                .iter()
                .any(|car_pos| {
                    ncollide2d::query::proximity(&state.0, &cuboid, &car_pos, &cuboid, 0.0)
                        == ncollide2d::query::Proximity::Intersecting
                })
                .not()
        };

        let mut rng = thread_rng();

        let path = rrt_connect(
            start,
            UniformLinearInterpolatedSamplingValidator::new(validate_state, 0.1),
            ExactGoal { goal: end },
            || {
                RigidBodyState2(Isometry2::new(
                    Vector2::new(rng.gen_range(-10.0..10.0), rng.gen_range(-10.0..10.0)),
                    rng.gen_range(0.0..2.0 * std::f64::consts::PI),
                ))
            },
        );

        dbg!(&path);

        let path = build_compound_path_from_steering_function(
            path,
            crate::dubins::compute_dubins_motion,
            0.0,
        )
        .expect("RRT-connect should have at least two states: start and end.");

        fn lift_isometry(iso2: &nalgebra::Isometry2<f64>) -> nalgebra::Isometry3<f64> {
            Isometry3::new(
                Vector3::new(iso2.translation.vector.x, 0.0, iso2.translation.vector.y),
                Vector3::new(0.0, -iso2.rotation.angle(), 0.0),
            )
        }

        let mut window = Window::new("Kiss3d: cube");

        let mut agent_car_cube = window.add_cube(
            cuboid.half_extents.x as f32 * 2.0f32,
            cuboid.half_extents.x as f32 * 2.0f32,
            cuboid.half_extents.y as f32 * 2.0f32,
        );

        for pos in &parked_cars_at {
            let mut car_cube = window.add_cube(
                cuboid.half_extents.x as f32 * 2.0f32,
                cuboid.half_extents.x as f32 * 2.0f32,
                cuboid.half_extents.y as f32 * 2.0f32,
            );
            car_cube.set_local_transformation(lift_isometry(pos).cast());
        }

        let range = path.defined_range();
        let mut t = *range.start();

        while window.render() {
            let state_interpolated = path.sample(t).expect("t should be kept in range");

            agent_car_cube.set_local_transformation(lift_isometry(&state_interpolated.0).cast());

            t += 0.05;

            if t > *range.end() {
                t = *range.start();
            }
        }
    }

    #[test]
    fn test_integration() {
        use acap::Proximity;
        use petgraph::prelude::EdgeRef;
        use rand::{thread_rng, Rng};

        use crate::goal::ExactGoal;
        use crate::path::{
            parametrize_by_distance, LerpPathDiscretePath, ParametrizedPath, TimedState,
        };
        use crate::point_state::PointState;
        use crate::rrt::rrt_connect;
        use itertools::Itertools;
        use nalgebra::{distance, Isometry3, Point3};
        use ncollide3d::math::Isometry;
        use ncollide3d::query::algorithms::gjk::GJKResult::Intersection;
        use ncollide3d::shape::{Ball, Cuboid, Shape};
        use std::f64::consts::PI;
        use std::ops::{Div, Not};

        let cuboid = Cuboid::new(Vector3::new(2.0f64, 1.0, 3.0));

        let start_state = PointState(Point3::new(-10.0, 0.0, 0.0));

        let obstacle_xform =
            Isometry3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0));

        let end_state = PointState(Point3::new(10.0, 0.0, 0.0));

        let mut rng = thread_rng();

        const SAMPLE_DISTANCE: f64 = 0.1;

        let validate_state = |state: &PointState| {
            ncollide3d::query::proximity(
                &Isometry3::from(state.0),
                &cuboid,
                &obstacle_xform,
                &cuboid,
                0.0,
            ) == ncollide3d::query::Proximity::Intersecting
        };

        let valid = UniformLinearInterpolatedSamplingValidator::new(validate_state, 0.1);

        fn test(v: impl MotionValidator<PointState>) {}

        test(valid);

        //
        // let path = rrt_connect(
        //     start_state,
        //     valid,
        //     ExactGoal { goal: end_state },
        //     || {
        //         PointState(
        //             Point3::new(
        //                 rng.gen_range(-100.0 .. 100.0),
        //                 rng.gen_range(-100.0 .. 100.0),
        //                 rng.gen_range(-100.0 .. 100.0)
        //             )
        //         )
        //     }
        // );
        //
        // let path = parametrize_by_distance(path, |st1, st2| distance(&st1.0,&st2.0), 0.0 );
        //
        // let mut window = Window::new("Kiss3d: cube");
        // let mut c      = window.add_cube(1.0, 1.0, 1.0);
        //
        // c.set_color(1.0, 0.0, 0.0);
        //
        // window.set_light(Light::StickToCamera);
        //
        // let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
        //
        // let range = LerpPathDiscretePath(&path).defined_range();
        // let mut t = *range.start();
        //
        // while window.render() {
        //
        //     let state_interpolated = LerpPathDiscretePath(&path).sample(t).expect("t should be kept in range");
        //     t += 0.5;
        //
        //     if t > *range.end() {
        //         t = *range.start();
        //     }
        //
        //     c.set_local_translation(nalgebra::Translation3::new(state_interpolated.0.x as f32, state_interpolated.0.y as f32, state_interpolated.0.z as f32));
        //
        //     for (TimedState(a,_),TimedState(b,_)) in path.states.iter().tuple_windows() {
        //         window.draw_line(&a.0.cast(),&b.0.cast(),&Point3::new(1.0,0.0,1.0));
        //     }
        // }
    }
}
