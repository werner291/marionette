use crate::state::{Distance, LinearInterpolate};
use acap::{Metric, Proximity};
use nalgebra::{Point3, Point, SimdValue};

#[derive(Copy, Clone)]
pub struct PointState<const D: usize>(pub Point<f64, D>);

impl<const D: usize> Distance<PointState<D>> for PointState<D> {
    type DistanceValue = f64;

    fn distance(&self, other: &PointState<D>) -> Self::DistanceValue {
        (self.0 - other.0).norm()
    }
}

impl<const D: usize> Proximity for PointState<D> {
    type Distance = f64;

    fn distance(&self, other: &Self) -> Self::Distance {
        (&self.0 - &other.0).norm_squared()
    }
}

impl<const D: usize> Metric for PointState<D> {}

impl<const D: usize> LinearInterpolate<f64> for PointState<D> {
    fn linear_interpolate(&self, other: &Self, t: f64) -> Self {
        PointState(Point::from(
            self.0.zip_map_lanes(other.0.clone(), |a,b| a.linear_interpolate(&b, t))
        ))
    }
}