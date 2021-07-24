pub mod rigid_2d;

use std::ops::{Mul, Add, Sub};
use num_traits::One;
use nalgebra::{Point3, Point, SimdValue};

pub trait Distance<B> {
    type DistanceValue;

    fn distance(&self, other: &B) -> Self::DistanceValue;
}

pub trait Lerp<Parameter> {
    fn lerp(&self, other: &Self, t: Parameter) -> Self;
}

impl Lerp<f64> for f64 {
    fn lerp(&self, other: &Self, t: f64) -> Self {
        self * (1.0-t) + other * t
    }
}

impl<const D: usize> Lerp<f64> for Point<f64, D> {
    fn lerp(&self, other: &Self, t: f64) -> Self {
        Point {
            coords: self.coords.zip_map(&other.coords, |a,b| a.lerp(&b,t))
        }
    }
}
