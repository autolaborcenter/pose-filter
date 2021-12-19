﻿use crate::{isometry, vector, Isometry2, Point2};
use chassis::ChassisModel;
use nalgebra::Complex;
use rand::{thread_rng, Rng};
use std::{
    cmp::Ordering::*,
    collections::VecDeque,
    f32::consts::{FRAC_PI_8, PI},
    time::Duration,
};

#[derive(Clone)]
pub struct Particle<M> {
    pub model: M,
    pub pose: Isometry2<f32>,
    pub weight: f32,
}

pub struct ParticleFilter<M: ChassisModel> {
    pub parameters: ParticleFilterParameters<M>,

    absolute_buffer: Option<(Duration, Point2<f32>)>,
    reletive_queue: VecDeque<(Duration, <M as ChassisModel>::Measure)>,

    particles: Vec<Particle<M>>,
}

pub struct ParticleFilterParameters<M> {
    pub memory_rate: f32,             // 权重的滤波器系数
    pub default_model: M,             // 用于初始化的底盘模型
    pub count: usize,                 // 固定的粒子数量
    pub measure_weight: f32,          // 基本测量权重
    pub beacon_on_robot: Point2<f32>, // 定位信标在机器人坐标系上的位置
    pub max_inconsistency: f32,       // 定位位置变化与估计位置变化允许的最大差距
}

impl<M: ChassisModel> ParticleFilter<M> {
    #[inline]
    pub fn new(parameters: ParticleFilterParameters<M>) -> Self {
        Self {
            parameters,

            absolute_buffer: None,
            reletive_queue: VecDeque::new(),

            particles: Vec::new(),
        }
    }
}

impl<M> ParticleFilter<M>
where
    M: Clone + ChassisModel,
    M::Measure: Copy + std::ops::Mul<f32, Output = M::Measure>,
{
    #[inline]
    pub fn measure(&mut self, time: Duration, p: Point2<f32>) {
        self.absolute_buffer = Some((time, p));
        self.calculate();
    }

    #[inline]
    pub fn update(&mut self, time: Duration, measure: <M as ChassisModel>::Measure) {
        self.reletive_queue.push_front((time, measure));
        self.calculate();
    }

    #[inline]
    pub fn particles<'a>(&'a self) -> &'a [Particle<M>] {
        self.particles.as_slice()
    }

    pub fn get(&self) -> Isometry2<f32> {
        let mut particles = self
            .particles
            .iter()
            .take_while(|p| p.weight > 0.0)
            .cloned()
            .collect::<Vec<_>>();
        for (_, m) in self.reletive_queue.iter().rev() {
            for particle in particles.iter_mut() {
                particle.pose *= particle.model.measure(&m).to_odometry().pose;
            }
        }
        let mut weight = 0.0;
        let mut p = vector(0.0, 0.0);
        let mut d = Complex { re: 0.0, im: 0.0 };
        for particle in particles {
            weight += particle.weight;
            p += particle.pose.translation.vector * particle.weight;
            d += particle.pose.rotation.complex() * particle.weight;
        }
        p /= weight;
        d /= weight;
        isometry(p[0], p[1], d.re, d.im)
    }

    fn calculate(&mut self) {
        let (t, measure) = match self.absolute_buffer.take() {
            Some(pair) => pair,
            None => return,
        };
        let mut save = None;
        while let Some((t1, m)) = self.reletive_queue.pop_back() {
            match t1.cmp(&t) {
                Less => {
                    save = Some((t1, m));
                    for particle in self.particles.iter_mut() {
                        particle.pose *= particle.model.measure(&m).to_odometry().pose;
                    }
                }
                Equal => {
                    save = Some((t1, m));
                    for particle in self.particles.iter_mut() {
                        particle.pose *= particle.model.measure(&m).to_odometry().pose;
                    }
                    break;
                }
                Greater => {
                    save = if let Some((t0, _)) = save {
                        let k = (t - t0).as_secs_f32() / (t1 - t0).as_secs_f32();
                        for particle in self.particles.iter_mut() {
                            particle.pose *= (particle.model.measure(&m) * k).pose;
                        }
                        Some((t1, m * (1.0 - k)))
                    } else {
                        Some((t1, m))
                    };
                    break;
                }
            }
        }
        if let Some(pair) = save {
            self.reletive_queue.push_back(pair);

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
                // self.particles
                //     .sort_unstable_by(|a, b| b.weight.partial_cmp(&a.weight).unwrap());
                if j < self.parameters.count {
                    let total = (self.parameters.count - j) as f32 / w;
                    let mut rng = thread_rng();
                    for i in 0..j {
                        let particle = self.particles[i].clone();
                        let n = (particle.weight * total).round() as isize;
                        if n <= 0 {
                            break;
                        }
                        for _ in 0..n {
                            let dx = rng.gen_range(-0.03..0.03);
                            let dy = rng.gen_range(-0.03..0.03);
                            let (sin, cos) = rng.gen_range(-FRAC_PI_8..FRAC_PI_8).sin_cos();
                            self.particles.push(Particle {
                                model: particle.model.clone(),
                                pose: particle.pose * isometry(dx, dy, cos, sin),
                                weight: self.parameters.update_weight(particle.weight, 1.0),
                            });
                        }
                    }
                }
            }
        }
    }
}

impl<M: Clone> ParticleFilterParameters<M> {
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
                    model: self.default_model.clone(),
                    pose: isometry(measure[0], measure[1], cos, sin) * robot_on_beacon,
                    weight: 0.1 / self.memory_rate.powi(2),
                }
            })
            .collect()
    }
}
