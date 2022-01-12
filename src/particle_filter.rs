use crate::{gaussian, isometry, vector, Isometry2, Point2};
use chassis::ChassisModel;
use nalgebra::{Complex, Normed};
use std::{
    cmp::Ordering::*,
    collections::VecDeque,
    f32::consts::{FRAC_PI_4, PI},
    time::Duration,
};

/// 粒子滤波器
pub struct ParticleFilter<M: ChassisModel, F> {
    pub parameters: ParticleFilterParameters<M>,

    absolute: Option<(Duration, Point2<f32>)>,
    incremental: VecDeque<(Duration, <M as ChassisModel>::Measure)>,

    particles: Vec<Particle<M>>,
    f: F,
}

/// 粒子
#[derive(Clone)]
pub struct Particle<M> {
    pub pose: Isometry2<f32>, // 位姿
    pub model: M,             // 用于将控制/测量转化为位姿增量的模型
    pub weight: f32,          // 经异构传感器验证得到的权重
}

/// 滤波器参数
pub struct ParticleFilterParameters<M> {
    pub incremental_timeout: Duration, // 增量队列最大长度
    pub memory_rate: f32,              // 权重的滤波器系数
    pub default_model: M,              // 用于初始化的底盘模型
    pub count: usize,                  // 固定的粒子数量
    pub measure_weight: f32,           // 基本测量权重
    pub beacon_on_robot: Point2<f32>,  // 定位信标在机器人坐标系上的位置
    pub max_inconsistency: f32,        // 定位位置变化与估计位置变化允许的最大差距
}

impl<M: ChassisModel, F: FnMut(&M, f32) -> M> ParticleFilter<M, F> {
    #[inline]
    pub fn new(parameters: ParticleFilterParameters<M>, f: F) -> Self {
        Self {
            parameters,

            absolute: None,
            incremental: VecDeque::new(),

            particles: Vec::new(),
            f,
        }
    }
}

impl<M, F> ParticleFilter<M, F>
where
    M: Clone + ChassisModel,
    M::Measure: Copy + std::ops::Mul<f32, Output = M::Measure>,
    F: FnMut(&M, f32) -> M,
{
    /// 保存一个绝对位置
    #[inline]
    pub fn measure(&mut self, time: Duration, p: Point2<f32>) {
        self.absolute = Some((time, p));
        self.calculate();
    }

    /// 保存一个位姿增量
    #[inline]
    pub fn update(&mut self, time: Duration, i: <M as ChassisModel>::Measure) {
        self.incremental.push_front((time, i));
        self.calculate();
        // 超时的更新记录直接应用于粒子并移除
        if let Some(deadline) = time.checked_sub(self.parameters.incremental_timeout) {
            while let Some((t, m)) = self.incremental.back() {
                if t <= &deadline {
                    Self::move_particles(&mut self.particles, m);
                    self.incremental.pop_back();
                } else {
                    break;
                }
            }
        }
    }

    /// 查看粒子
    #[inline]
    pub fn particles<'a>(&'a self) -> &'a [Particle<M>] {
        self.particles.as_slice()
    }

    /// 查看粒子
    #[inline]
    pub fn fold_models<T>(&self, t0: T, f: impl Fn(T, &M, f32) -> T) -> (T, f32) {
        self.particles.iter().fold((t0, 0.0), |(t, w), p| {
            (f(t, &p.model, p.weight), w + p.weight)
        })
    }

    /// 估计当前位姿
    pub fn get(&self) -> Option<Isometry2<f32>> {
        if self.particles.is_empty() {
            return None;
        }
        let mut particles = self
            .particles
            .iter()
            .take_while(|p| p.weight > 0.0)
            .cloned()
            .collect::<Vec<_>>();
        for (_, m) in self.incremental.iter().rev() {
            Self::move_particles(&mut particles, &m);
        }
        let mut weight = 0.0;
        let mut p = vector(0.0, 0.0);
        let mut d = Complex { re: 0.0, im: 0.0 };
        for Particle {
            model: _,
            pose: Isometry2 {
                translation,
                rotation,
            },
            weight: w,
        } in particles
        {
            weight += w;
            p += translation.vector * w;
            d += rotation.complex() * w;
        }
        if weight.is_normal() {
            p /= weight;
            d = Some(d.norm())
                .filter(|n| n > &f32::EPSILON)
                .map_or(Complex { re: 1.0, im: 0.0 }, |n| d / n);
            Some(isometry(p[0], p[1], d.re, d.im))
        } else {
            None
        }
    }

    /// 使用一个增量移动所有粒子
    #[inline]
    fn move_particles(particles: &mut [Particle<M>], m: &M::Measure) {
        for particle in particles.iter_mut() {
            particle.pose *= particle.model.measure(m).to_odometry().pose;
        }
    }

    /// 计算粒子权重
    fn calculate(&mut self) {
        let (t, measure) = match self.absolute.take() {
            Some(pair) => pair,
            None => return,
        };
        let mut save = None;
        while let Some((t1, m)) = self.incremental.pop_back() {
            match t1.cmp(&t) {
                Less => {
                    save = Some((t1, m));
                    Self::move_particles(&mut self.particles, &m);
                }
                Equal => {
                    save = Some((t1, m));
                    Self::move_particles(&mut self.particles, &m);
                    break;
                }
                Greater => {
                    save = if let Some((t0, _)) = save {
                        let k = (t - t0).as_secs_f32() / (t1 - t0).as_secs_f32();
                        Self::move_particles(&mut self.particles, &(m * k));
                        Some((t1, m * (1.0 - k)))
                    } else {
                        Some((t1, m))
                    };
                    break;
                }
            }
        }
        if let Some(pair) = save {
            self.incremental.push_back(pair);
        } else {
            if self.particles.is_empty() {
                self.particles = self.parameters.initialize(measure);
            }
            return;
        }

        let mut w = 0.0;
        let mut p = vector(0.0, 0.0);
        let mut d = Complex { re: 0.0, im: 0.0 };
        let mut j = 0;
        for i in 0..self.particles.len() {
            let particle = unsafe { self.particles.get_unchecked_mut(i) };
            let beacon = particle.pose * self.parameters.beacon_on_robot;
            let diff = f32::min(
                (beacon - measure).norm() / self.parameters.max_inconsistency,
                1.0,
            );
            particle.weight = self.parameters.update_weight(particle.weight, diff);
            if particle.weight > 0.1 {
                w += particle.weight;
                p += particle.pose.translation.vector * particle.weight;
                d += particle.pose.rotation.complex() * particle.weight;
                if i > j {
                    unsafe { *self.particles.get_unchecked_mut(j) = particle.clone() };
                }
                j += 1;
            }
        }
        if j == 0 {
            self.particles = self.parameters.initialize(measure);
        } else {
            self.particles.truncate(j);
            self.particles
                .sort_unstable_by(|a, b| b.weight.partial_cmp(&a.weight).unwrap());
            if j < self.parameters.count {
                let total = (self.parameters.count - j) as f32 / w;
                let sigma = 1.0 - j as f32 / self.parameters.count as f32;
                for i in 0..j {
                    let Particle {
                        model,
                        pose,
                        weight,
                    } = self.particles[i].clone();
                    let n = (weight * total).round() as isize;
                    if n <= 0 {
                        break;
                    }
                    for _ in 0..n {
                        let dx = gaussian() * sigma * 0.1;
                        let dy = gaussian() * sigma * 0.1;
                        let (sin, cos) = (gaussian() * sigma * FRAC_PI_4).sin_cos();
                        self.particles.push(Particle {
                            model: (self.f)(&model, weight),
                            pose: pose * isometry(dx, dy, cos, sin),
                            weight: self.parameters.update_weight(weight, 1.0),
                        });
                    }
                }
            }
        }
    }
}

impl<M: Clone> ParticleFilterParameters<M> {
    /// 用低通滤波更新权重
    #[inline]
    fn update_weight(&self, w: f32, diff: f32) -> f32 {
        w * self.memory_rate + (1.0 - diff) * (1.0 - self.memory_rate)
    }

    /// 初始化一组粒子
    fn initialize(&self, measure: Point2<f32>) -> Vec<Particle<M>> {
        let robot_on_beacon =
            isometry(-self.beacon_on_robot[0], -self.beacon_on_robot[1], 1.0, 0.0);
        let step = 2.0 * PI / self.count as f32;
        (0..self.count)
            .map(|i| {
                let (sin, cos) = (i as f32 * step).sin_cos();
                Particle {
                    pose: isometry(measure[0], measure[1], cos, sin) * robot_on_beacon,
                    model: self.default_model.clone(),
                    weight: 0.1 / self.memory_rate.powi(2),
                }
            })
            .collect()
    }
}
