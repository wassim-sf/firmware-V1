//! A minimal rigid-body SE(3) plant for the closed-loop tests (test builds only).
//!
//! Z-up dynamics matching [`crate::frame`]: thrust acts along the body `+e3` axis,
//! gravity along `−e3`.
//!
//! ```text
//! ṗ = v
//! v̇ = −g·e3 + (f/m)·R·e3
//! Ṙ = R·hat(Ω)                     (integrated exactly with the exp map)
//! Ω̇ = J⁻¹·(M − Ω × J·Ω)
//! ```
//!
//! Attitude is advanced on SO(3) via the exponential map (stays a rotation with no
//! reprojection); the rest use semi-implicit Euler with fine substeps.

use crate::math::{so3_exp, Mat3, Vec3};
use crate::params::QuadParams;
use crate::QuadState;

pub struct Plant {
    pub state: QuadState,
}

impl Plant {
    pub fn new(state: QuadState) -> Self {
        Self { state }
    }

    /// Hold `(f, moment)` constant and integrate forward by `dt` in `substeps`.
    pub fn step(&mut self, f: f32, moment: Vec3, p: &QuadParams, dt: f32, substeps: u32) {
        let h = dt / substeps as f32;
        let e3 = Vec3::new(0.0, 0.0, 1.0);
        for _ in 0..substeps {
            let s = &self.state;
            // Translational (semi-implicit: use current R).
            let acc = -p.gravity * e3 + (f / p.mass) * (s.r * e3);
            let new_v = s.v + acc * h;
            let new_p = s.p + new_v * h;
            // Rotational.
            let jomega = p.j * s.omega;
            let omega_dot = p.j_inv * (moment - s.omega.cross(&jomega));
            let new_r = s.r * so3_exp(&(s.omega * h));
            let new_omega = s.omega + omega_dot * h;
            self.state = QuadState {
                p: new_p,
                v: new_v,
                r: new_r,
                omega: new_omega,
            };
        }
    }

    /// SO(3) configuration error `Ψ = ½·tr(I − Rcᵀ·R)` against a desired attitude.
    pub fn psi(&self, rc: &Mat3) -> f32 {
        0.5 * (Mat3::identity() - rc.transpose() * self.state.r).trace()
    }
}
