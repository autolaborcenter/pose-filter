use nalgebra::{Isometry2, Point2, Vector2};
use std::time::Duration;

// mod derecorder;
mod interpolation_filter;
mod particle_filter;

pub use interpolation_filter::{InterpolationFilter, PoseType};
pub use particle_filter::{ParticleFilter, ParticleFilterParameters};

struct Stamped<T>(Duration, T);

type Particle = Stamped<Isometry2<f32>>;

pub trait PoseFilter<Key> {
    fn update(&mut self, key: Key, time: Duration, pose: Isometry2<f32>) -> Isometry2<f32>;
}

#[inline]
const fn isometry(x: f32, y: f32, cos: f32, sin: f32) -> Isometry2<f32> {
    use nalgebra::{Complex, Translation, Unit};
    Isometry2 {
        translation: Translation {
            vector: vector(x, y),
        },
        rotation: Unit::new_unchecked(Complex { re: cos, im: sin }),
    }
}

#[inline]
const fn vector(x: f32, y: f32) -> Vector2<f32> {
    use nalgebra::{ArrayStorage, OVector};
    OVector::from_array_storage(ArrayStorage([[x, y]]))
}

#[inline]
const fn point(x: f32, y: f32) -> Point2<f32> {
    Point2 {
        coords: vector(x, y),
    }
}
