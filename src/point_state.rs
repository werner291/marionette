use crate::state::{Distance, LinearInterpolate};
use acap::{Metric, Proximity};
use nalgebra::Point3;

#[derive(Copy, Clone)]
pub struct PointState(pub Point3<f64>);

impl Distance<PointState> for PointState {
    type DistanceValue = f64;

    fn distance(&self, other: &PointState) -> Self::DistanceValue {
        (self.0 - other.0).norm()
    }
}

impl Proximity for PointState {
    type Distance = f64;

    fn distance(&self, other: &Self) -> Self::Distance {
        (self.0 - other.0).norm_squared()
    }
}

impl Metric for PointState {}

impl LinearInterpolate<f64> for PointState {
    fn linear_interpolate(&self, other: &Self, t: f64) -> Self {
        PointState(Point3::new(
            self.0.x.linear_interpolate(&other.0.x, t),
            self.0.y.linear_interpolate(&other.0.y, t),
            self.0.z.linear_interpolate(&other.0.z, t),
        ))
    }
}