use nalgebra::Complex;
use rand::{thread_rng, Rng};

use crate::{isometry, point, vector, Isometry2, Point2};
use std::{
    cmp::Ordering::*,
    collections::{HashSet, VecDeque},
    f32::consts::{FRAC_PI_6, PI},
    time::Duration,
};

pub struct ParticleFilter {
    parameters: ParticleFilterParameters,

    transformation: Isometry2<f32>,

    absolute_buffer: (Duration, Point2<f32>),
    reletive_queue: VecDeque<(Duration, Isometry2<f32>)>,

    last_pair: Option<(Point2<f32>, Isometry2<f32>)>,
    particles: Vec<(Isometry2<f32>, usize)>,
}

pub struct ParticleFilterParameters {
    pub count: usize,                 // 固定的粒子数量
    pub measure_weight: f32,          // 基本测量权重
    pub beacon_on_robot: Point2<f32>, // 定位信标在机器人坐标系上的位置
    pub max_inconsistency: f32,       // 定位位置变化与估计位置变化允许的最大差距
}

impl ParticleFilter {
    #[inline]
    pub fn new(parameters: ParticleFilterParameters) -> Self {
        Self {
            parameters,

            transformation: isometry(0.0, 0.0, 1.0, 0.0),

            absolute_buffer: (Duration::ZERO, point(0.0, 0.0)),
            reletive_queue: VecDeque::new(),

            last_pair: None,
            particles: Vec::new(),
        }
    }
}

impl ParticleFilter {
    #[inline]
    pub fn measure(&mut self, time: Duration, p: Point2<f32>) {
        self.absolute_buffer = (time, p);
        self.calculate0();
    }

    #[inline]
    pub fn update(&mut self, time: Duration, pose: Isometry2<f32>) {
        self.reletive_queue.push_front((time, pose));
        self.calculate0();
    }

    #[inline]
    pub fn particles<'a>(&'a self) -> &'a [(Isometry2<f32>, usize)] {
        self.particles.as_slice()
    }

    #[inline]
    pub fn transform(&self, pose: Isometry2<f32>) -> Isometry2<f32> {
        self.transformation * pose
    }

    fn calculate0(&mut self) {
        fn interpolate(p0: &Isometry2<f32>, p1: &Isometry2<f32>, k: f32) -> Isometry2<f32> {
            let d = p0.inverse() * p1;
            let v = d.translation.vector * k;
            let a = d.rotation.angle() * k;
            p0 * Isometry2::new(v, a)
        }

        let mut save = None;
        let t = self.absolute_buffer.0;
        while let Some((t1, p1)) = self.reletive_queue.pop_back() {
            let last = save.replace((t1, p1));
            match t1.cmp(&t) {
                Less => {}
                Equal => {
                    self.calculate1(p1);
                    break;
                }
                Greater => {
                    if let Some((t0, p0)) = last {
                        let k = (t - t0).as_secs_f32() / (t1 - t0).as_secs_f32();
                        self.calculate1(interpolate(&p0, &p1, k));
                    }
                    break;
                }
            }
        }
        if let Some(pair) = save {
            self.reletive_queue.push_back(pair);
        }
    }

    #[inline]
    fn initialize(&mut self) {
        let (measure, _) = self.last_pair.unwrap();
        self.particles = self.parameters.initialize(measure);
    }

    fn calculate1(&mut self, pose: Isometry2<f32>) {
        let measure = self.absolute_buffer.1;
        match self.last_pair.replace((measure, pose)) {
            Some((m, p)) => {
                // 推动粒子
                let delta = p.inv_mul(&pose);
                self.particles
                    .iter_mut()
                    .for_each(|(pose, _)| *pose *= delta);
                // 计算测量权重
                let measure_weight = self.parameters.measure_weight((measure - m).norm(), delta);
                if measure_weight > 0.0 {
                    let mut to_reset = HashSet::new();
                    let mut weight = 0.0;
                    let mut p = vector(0.0, 0.0);
                    let mut d = Complex { re: 0.0, im: 0.0 };
                    for (i, (pose, age)) in self.particles.iter_mut().enumerate() {
                        let beacon = *pose * self.parameters.beacon_on_robot;
                        let diff = (beacon - measure).norm() / self.parameters.max_inconsistency;
                        if diff < 1.0 {
                            *age = std::cmp::min(*age + 1, 20);
                            let w = *age as f32 * (1.0 - diff);
                            weight += w;
                            p += pose.translation.vector * w;
                            d += pose.rotation.complex() * w;
                        } else if *age == 1 {
                            to_reset.insert(i);
                        } else {
                            *age -= 1;
                        }
                    }
                    println!("Σ = {}", weight);
                    if weight < 1.0 {
                        self.particles = self.parameters.initialize(measure);
                    } else {
                        p /= weight;
                        d /= weight;
                        let angle = d.im.atan2(d.re);
                        self.transformation = Isometry2::new(p, angle) * pose.inverse();
                        let mut rng = thread_rng();
                        for i in to_reset {
                            self.particles[i].0 = Isometry2::new(
                                vector(
                                    p[0] + rng.gen_range(-0.05..0.05),
                                    p[1] + rng.gen_range(-0.05..0.05),
                                ),
                                angle + rng.gen_range(-FRAC_PI_6..FRAC_PI_6),
                            );
                        }
                    }
                }
            }
            None => self.initialize(),
        }
    }
}

impl ParticleFilterParameters {
    /// 计算测量权重
    #[inline]
    fn measure_weight(&self, lm: f32, delta: Isometry2<f32>) -> f32 {
        let lp = (delta * self.beacon_on_robot - self.beacon_on_robot).norm();
        self.measure_weight * (1.0 - (lm - lp).abs() / self.max_inconsistency)
    }

    /// 初始化一组粒子
    fn initialize(&self, measure: Point2<f32>) -> Vec<(Isometry2<f32>, usize)> {
        let robot_on_beacon =
            isometry(-self.beacon_on_robot[0], -self.beacon_on_robot[1], 1.0, 0.0);
        let step = 2.0 * PI / self.count as f32;
        (0..self.count)
            .map(|i| {
                let (sin, cos) = (i as f32 * step).sin_cos();
                (
                    isometry(measure[0], measure[1], cos, sin) * robot_on_beacon,
                    1,
                )
            })
            .collect()
    }
}
