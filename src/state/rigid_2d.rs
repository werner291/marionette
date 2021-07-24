pub mod rigid_body_state_2d {

    use crate::state::{Distance, Lerp};

    /// A state that represents a 2D isometry: position and rotation.
    #[derive(Copy, Clone, Debug)]
    pub struct RigidBodyState2(pub nalgebra::Isometry2<f64>);

    impl Distance<RigidBodyState2> for RigidBodyState2 {
        type DistanceValue = f64;

        fn distance(&self, other: &RigidBodyState2) -> Self::DistanceValue {
            (self.0.translation.vector - other.0.translation.vector).norm()
                + (self.0.rotation.angle_to(&other.0.rotation).abs())
        }
    }

    impl acap::Proximity for RigidBodyState2 {
        type Distance = f64;

        fn distance(&self, other: &Self) -> Self::Distance {
            crate::state::Distance::distance(self, other)
        }
    }

    impl Lerp<f64> for RigidBodyState2 {
        fn lerp(&self, other: &Self, t: f64) -> Self {
            RigidBodyState2(
                self.0.lerp_slerp(&other.0, t)
            )
        }
    }
}
