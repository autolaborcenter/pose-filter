use rand::{thread_rng, Rng};

#[derive(Clone)]
pub struct Gaussian {
    mu: f32,
    sigma: f32,
    memory: Option<f32>,
}

impl Gaussian {
    #[inline]
    pub const fn new(mu: f32, sigma: f32) -> Self {
        Self {
            mu,
            sigma,
            memory: None,
        }
    }

    pub fn next(&mut self) -> f32 {
        match self.memory.take() {
            Some(x) => x,
            None => {
                let mut rng = thread_rng();
                loop {
                    let x: f32 = 2.0 * rng.gen_range(-1.0..1.0) - 1.0;
                    let y: f32 = 2.0 * rng.gen_range(-1.0..1.0) - 1.0;
                    let r = x.hypot(y);
                    if 0.0 < r && r < 1.0 {
                        let m = (-2.0 * r.ln() / r).sqrt() * self.sigma;
                        let x = x * m + self.mu;
                        let y = y * m + self.mu;
                        self.memory = Some(x);
                        return y;
                    }
                }
            }
        }
    }
}

#[test]
fn test() {
    let mut gaussian = Gaussian::new(0.0, 1.0);
    for _ in 0..10 {
        println!("{}", gaussian.next());
    }
}
