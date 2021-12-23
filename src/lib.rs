use nalgebra::{Isometry2, Point2, Vector2};
use rand::{thread_rng, Rng};
use std::cell::Cell;

// mod derecorder;
mod interpolation_filter;
mod particle_filter;

pub use interpolation_filter::{InterpolationFilter, PoseType};
pub use particle_filter::{Particle, ParticleFilter, ParticleFilterParameters};

pub fn gaussian() -> f32 {
    thread_local! {
        static MEMORY: Cell<Option<f32>> = Cell::new(None);
    }
    match MEMORY.with(|cell| cell.take()) {
        Some(x) => x,
        None => {
            let mut rng = thread_rng();
            loop {
                let x: f32 = 2.0 * rng.gen_range(-1.0..1.0) - 1.0;
                let y: f32 = 2.0 * rng.gen_range(-1.0..1.0) - 1.0;
                let r = x.hypot(y);
                if 0.0 < r && r < 1.0 {
                    let m = (-2.0 * r.ln() / r).sqrt();
                    let x = x * m;
                    let y = y * m;
                    MEMORY.with(|cell| cell.set(Some(x)));
                    return y;
                }
            }
        }
    }
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

#[test]
fn test() {
    for _ in 0..10 {
        println!("{}", gaussian());
    }
}
