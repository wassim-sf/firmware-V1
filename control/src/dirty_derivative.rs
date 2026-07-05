//! Band-limited ("dirty") numerical derivative — a direct port of
//! `se3quad .../dirty_derivative.rs` (originally `matlab/DirtyDerivative.m`).
//!
//! Causal filtered differentiator `P(s) = s / (tau·s + 1)`, discretised with the
//! Tustin (bilinear) transform:
//!
//! ```text
//! dot[k] = a1 · dot[k-1] + a2 · (x[k] − x[k-1])
//! a1 = (2·tau − Ts) / (2·tau + Ts)
//! a2 =        2      / (2·tau + Ts)
//! ```
//!
//! Warmed up for `order` samples before it produces output. The controller
//! differentiates only the *measured* velocity (a feedback signal with no closed
//! form); the desired-trajectory derivatives come analytically from the [`Target`].
//!
//! [`Target`]: crate::trajectory::Target

use crate::math::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct DirtyDerivative {
    a1: f32,
    a2: f32,
    order: u32,
    dot: Vec3,
    x_d1: Vec3,
    it: u32,
}

impl DirtyDerivative {
    /// `order` = which derivative this filter feeds (number of warm-up samples),
    /// `tau` = filter time constant, `ts` = sample time.
    pub fn new(order: u32, tau: f32, ts: f32) -> Self {
        Self {
            a1: (2.0 * tau - ts) / (2.0 * tau + ts),
            a2: 2.0 / (2.0 * tau + ts),
            order,
            dot: Vec3::ZERO,
            x_d1: Vec3::ZERO,
            it: 1,
        }
    }

    /// Feed the next sample, returning the current filtered derivative.
    pub fn calculate(&mut self, x: Vec3) -> Vec3 {
        if self.it > self.order {
            self.dot = self.a1 * self.dot + self.a2 * (x - self.x_d1);
        }
        self.it += 1;
        self.x_d1 = x;
        self.dot
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn tracks_sine_derivative() {
        // d/dt sin(2π f t) = 2π f cos(2π f t)
        let ts = 0.001;
        let tau = 0.01;
        let freq = 1.0;
        let mut d = DirtyDerivative::new(1, tau, ts);
        let mut max_err: f32 = 0.0;
        for k in 0..3000 {
            let t = k as f32 * ts;
            let x = libm::sinf(2.0 * PI * freq * t);
            let est = d.calculate(Vec3::new(x, 0.0, 0.0)).x;
            let truth = 2.0 * PI * freq * libm::cosf(2.0 * PI * freq * t);
            if t > 0.5 {
                max_err = max_err.max((est - truth).abs());
            }
        }
        // Residual is dominated by the filter's phase lag; true amplitude ≈ 6.28.
        assert!(max_err < 0.5, "max derivative error too large: {max_err}");
    }
}
