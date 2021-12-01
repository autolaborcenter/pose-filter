use crate::Stamped;
use chassis::{Isometry2, Odometry, StatusPredictor, TrajectoryPredictor};
use std::{cmp::Ordering::*, collections::VecDeque, time::Instant};

/// 在时间轴上记录或预测位姿
///
/// 根据底盘模型估计任意时刻位姿
pub struct Derecorder<P: StatusPredictor> {
    memory: VecDeque<Stamped<Odometry>>,
    predictors: VecDeque<Stamped<TrajectoryPredictor<P>>>,
}

impl<P: StatusPredictor> Default for Derecorder<P> {
    #[inline]
    fn default() -> Self {
        Self {
            memory: [Stamped(Instant::now(), Odometry::ZERO)]
                .into_iter()
                .collect(),
            predictors: Default::default(),
        }
    }
}

impl<P: StatusPredictor> Derecorder<P> {
    /// 存入一个新的预测器
    pub fn update_predictor(&mut self, p: TrajectoryPredictor<P>) {
        let now = Instant::now();
        self.predictors.push_front(Stamped(now, p));
        // 如果之前预测的太多，移除一些
        if self.predictors.front().unwrap().0 > now {
            match self.memory.binary_search_by(|Stamped(t, _)| now.cmp(t)) {
                Ok(mut i) => {
                    while i > 0 {
                        i -= 1;
                        self.memory.pop_front();
                    }
                }
                Err(mut i) => {
                    while i > 1 {
                        i -= 1;
                        self.memory.pop_front();
                    }
                    let Stamped(t1, o1) = self.memory.pop_front().unwrap();
                    if let Some(Stamped(t0, o0)) = self.memory.front() {
                        let k = (now - *t0).as_secs_f32() / (t1 - *t0).as_secs_f32();
                        let o = interpolate(o0, &o1, k);
                        self.save(now, o);
                    }
                }
            };
        }
    }

    /// 从早到晚移除，直到仅剩一个缓存不晚于 `t` 时刻
    pub fn drop_until(&mut self, t: Instant) {
        if self.predictors.back().unwrap().0 < t {
            match self.memory.binary_search_by(|Stamped(t_, _)| t.cmp(t_)) {
                Ok(i) | Err(i) => self.memory.truncate(i + 1),
            };
        }
    }

    /// 获得 `t` 时刻的位姿
    pub fn interpolate(&mut self, t: Instant) -> Odometry {
        let Stamped(t0, o0) = *self.memory.back().unwrap(); // 最早的缓存
        let Stamped(t1, o1) = *self.memory.front().unwrap(); // 最新的缓存

        // 比最早的缓存更早
        if t < t0 {
            o0
        }
        // 查找缓存
        else if t <= t1 {
            // 二分查找
            //
            // 注意到 `self.memory` 按从新到老排序（<=> 从大到小），
            // 故需要反向使用 cmp
            match self.memory.binary_search_by(|Stamped(t0, _)| t.cmp(t0)) {
                Ok(i) => self.memory[i].1,
                Err(i) => {
                    let Stamped(t0, o0) = self.memory[i];
                    let Stamped(t1, o1) = self.memory[i - 1];
                    let k = (t - t0).as_secs_f32() / (t1 - t0).as_secs_f32();
                    interpolate(&o0, &o1, k)
                }
            }
        }
        // 需要预测
        else {
            self.predict(t, t1, o1)
        }
    }

    #[inline]
    fn save(&mut self, t: Instant, o: Odometry) {
        self.memory.push_front(Stamped(t, o));
    }

    /// 已知当前已计算的最新缓存为 `t1` 时刻的 `o1`，消费预测器计算 `t` 时刻位姿
    fn predict(&mut self, t: Instant, mut t1: Instant, mut o1: Odometry) -> Odometry {
        // 如果还有新的预测器
        while let Some(Stamped(t2, mut pre)) = self.predictors.pop_back() {
            // 全新预测器，说明机器人静止了一段时间
            if t2 > t1 {
                self.save(t2, o1);
                t1 = t2;
            }
            // 不是最后一个预测器，且当前预测器不能覆盖目标时刻
            if let Some(t2) = self
                .predictors
                .back()
                .filter(|Stamped(t_, _)| t_ <= &t)
                .map(|Stamped(t, _)| *t)
            {
                // 使用当前预测器，预测到下一个预测器生效
                while let Some(d) = pre.next() {
                    t1 += pre.period;
                    match t1.cmp(&t2) {
                        Less => {
                            o1 += d;
                            self.save(t1, o1);
                        }
                        Equal => {
                            o1 += d;
                            self.save(t1, o1);
                            break;
                        }
                        Greater => {
                            let k = (t1 - t2).as_secs_f32() / pre.period.as_secs_f32();
                            o1 = interpolate(&o1, &(o1 + d), 1.0 - k);
                            self.save(t2, o1);
                            break;
                        }
                    }
                }
            }
            // 最后一个预测器
            else {
                while let Some(d) = pre.next() {
                    t1 += pre.period;
                    let on = o1 + d;
                    self.save(t1, on);
                    if t1 > t {
                        let k = (t1 - t).as_secs_f32() / pre.period.as_secs_f32();
                        o1 = interpolate(&o1, &on, 1.0 - k);
                        // 预测器没用完，放回去
                        self.predictors.push_back(Stamped(t2, pre));
                        break;
                    } else {
                        o1 = on;
                    }
                }
                break;
            }
        }
        // 没有预测器了
        o1
    }
}

fn interpolate(o0: &Odometry, o1: &Odometry, k: f32) -> Odometry {
    let d = o0.pose.inv_mul(&o1.pose);
    let v = d.translation.vector * k;
    let a = d.rotation.angle() * k;
    Odometry {
        s: o0.s + (o1.s - o0.s) * k,
        a: o0.a + (o1.a - o0.a) * k,
        pose: o0.pose * Isometry2::new(v, a),
    }
}
