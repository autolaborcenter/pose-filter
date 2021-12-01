use chassis::{Isometry2, StatusPredictor};
use std::time::Instant;

mod derecorder;
mod interpolation_filter;

pub use derecorder::Derecorder;
pub use interpolation_filter::{InterpolationFilter, PoseType};

struct Stamped<T>(Instant, T);

type Particle = Stamped<Isometry2<f32>>;

pub trait PoseFilter<Key> {
    fn update(&mut self, key: Key, time: Instant, pose: Isometry2<f32>) -> Isometry2<f32>;
}

pub struct XXXFilter<P: StatusPredictor> {
    pub recorder: Derecorder<P>,
    pub trans: Isometry2<f32>,
}
