use chassis::{isometry, Isometry2};
use std::time::Duration;

/// 基于插值的滤波器，适用于任何物体
pub struct InterpolationFilter {
    transform: Isometry2<f32>, // 经过插值调整的变换关系，用于相对变换单独到来时
    relative_buffer: (Duration, Isometry2<f32>), // 相对定位缓存，用于插值
    absolute_buffer: (Duration, Isometry2<f32>), // 绝对定位缓存，用于插值
}

pub enum PoseType {
    Absolute,
    Relative,
}

impl InterpolationFilter {
    pub const fn new() -> Self {
        const ZERO: Isometry2<f32> = isometry(0.0, 0.0, 1.0, 0.0);
        Self {
            transform: ZERO,
            relative_buffer: (Duration::ZERO, ZERO),
            absolute_buffer: (Duration::ZERO, ZERO),
        }
    }
}

impl InterpolationFilter {
    pub fn update(
        &mut self,
        key: PoseType,
        time: Duration,
        pose: Isometry2<f32>,
    ) -> Isometry2<f32> {
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
                let (ta, pa) = self.absolute_buffer;
                let (tr, pr) = (time, pose);
                if t0 <= ta {
                    if ta <= tr {
                        let k = (ta - t0).as_secs_f32() / (tr - t0).as_secs_f32();
                        self.transform = pa * interpolate(&p0, &pr, k).inverse();
                    }
                    self.relative_buffer = (tr, pr);
                }
                self.transform * pose
            }
        }
    }
}
