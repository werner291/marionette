pub mod rigid_2d;
use nalgebra::{Point};



pub trait Distance<B> {
    type DistanceValue;

    fn distance(&self, other: &B) -> Self::DistanceValue;
}

pub trait LinearInterpolate<Parameter> {
    fn linear_interpolate(&self, other: &Self, t: Parameter) -> Self;
}

impl LinearInterpolate<f64> for f64 {
    fn linear_interpolate(&self, other: &Self, t: f64) -> Self {
        self * (1.0 - t) + other * t
    }
}

impl<const D: usize> LinearInterpolate<f64> for Point<f64, D> {
    fn linear_interpolate(&self, other: &Self, t: f64) -> Self {
        Point {
            coords: self.coords.zip_map(&other.coords, |a, b| a.linear_interpolate(&b, t)),
        }
    }
}
