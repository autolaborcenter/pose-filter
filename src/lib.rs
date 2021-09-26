use nalgebra::{Isometry2, Vector2};
use std::time::Instant;

pub trait PoseFilter<Key> {
    fn update(&mut self, key: Key, time: Instant, pose: Isometry2<f32>) -> Isometry2<f32>;
}

pub struct InterpolationAndPredictionFilter {
    transform: Isometry2<f32>, // 经过插值调整的变换关系，用于相对变换单独到来时
    relative_buffer: (Instant, Isometry2<f32>), // 相对定位缓存，用于插值
    absolute_buffer: (Instant, Isometry2<f32>), // 绝对定位缓存，用于插值
}

pub enum PoseType {
    Absolute,
    Relative,
}

impl InterpolationAndPredictionFilter {
    pub fn new() -> Self {
        let now = Instant::now();
        let zero = Isometry2::new(Vector2::new(0.0, 0.0), 0.0);
        Self {
            transform: zero,
            relative_buffer: (now, zero),
            absolute_buffer: (now, zero),
        }
    }
}

impl PoseFilter<PoseType> for InterpolationAndPredictionFilter {
    fn update(&mut self, key: PoseType, time: Instant, pose: Isometry2<f32>) -> Isometry2<f32> {
        match key {
            PoseType::Absolute => {
                self.absolute_buffer = (time, pose);
                pose
            }
            PoseType::Relative => {
                fn interpolate(p0: &Isometry2<f32>, p1: &Isometry2<f32>, k: f32) -> Isometry2<f32> {
                    let d = p0.inverse() * p1;
                    let v = d.translation.vector * k;
                    let a = d.rotation.angle() * k;
                    p0 * Isometry2::new(v, a)
                }

                let (t0, p0) = self.relative_buffer;
                let (t1, p1) = self.absolute_buffer;
                let (t2, p2) = (time, pose);
                if t0 <= t1 && t1 <= t2 {
                    let k = (t1 - t0).as_secs_f32() / (t2 - t0).as_secs_f32();
                    self.transform = interpolate(&p0, &p2, k).inverse() * p1;
                }
                self.relative_buffer = (t2, p2);
                self.transform * pose
            }
        }
    }
}
