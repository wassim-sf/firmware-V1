//! Physical parameters and controller gains. Ports `se3quad .../params.rs`
//! (`matlab/param.m`) to `f32`; the defaults are the simulator's exact values so
//! the ported controller and closed-loop tests reproduce its behaviour.
//!
//! The firmware drives analog ESCs through a **normalised** [`crate::mixer`], so
//! the physical rotor-mixing matrix is not needed here — the plant used by the
//! tests takes thrust + body moment directly, and the real airframe's `mass`/`J`
//! only set the *shape* of the SO(3) feedforward (absolute scale is absorbed by
//! the mixer gains and the tunable `kr`/`komega`).

use crate::math::{Mat3, Vec3};

/// Airframe physical parameters and controller gains.
#[derive(Clone, Copy, Debug)]
pub struct QuadParams {
    /// Gravitational acceleration [m/s²].
    pub gravity: f32,
    /// Mass [kg].
    pub mass: f32,
    /// Inertia matrix `diag(Jxx, Jyy, Jzz)` [kg·m²].
    pub j: Mat3,
    /// Inverse inertia matrix (precomputed).
    pub j_inv: Mat3,
    /// Controller sample time [s] — set to the actual control-loop period.
    pub ts: f32,
    /// Dirty-derivative filter time constant [s].
    pub tau: f32,

    // --- control gains (Lee 2011) ---
    /// Position proportional gain.
    pub kx: f32,
    /// Velocity (position-derivative) gain.
    pub kv: f32,
    /// Attitude proportional gain.
    pub kr: f32,
    /// Angular-velocity gain.
    pub komega: f32,
    /// Position integral gain (0 = pure PD, as in the paper).
    pub ki: f32,
    /// Anti-windup bound on each component of the position-error integral [m·s].
    pub i_max: f32,
}

impl Default for QuadParams {
    fn default() -> Self {
        let mass = 4.34;
        let j = Mat3::from_diagonal(Vec3::new(0.0820, 0.0845, 0.1377));
        let j_inv = Mat3::from_diagonal(Vec3::new(1.0 / 0.0820, 1.0 / 0.0845, 1.0 / 0.1377));
        QuadParams {
            gravity: 9.81,
            mass,
            j,
            j_inv,
            ts: 0.01,
            tau: 0.05,
            kx: 4.0 * mass,
            kv: 5.6 * mass,
            kr: 8.81,
            komega: 2.54,
            ki: 0.0,
            i_max: 3.0,
        }
    }
}

impl QuadParams {
    /// Hover thrust `m·g` [N].
    pub fn hover_thrust(&self) -> f32 {
        self.mass * self.gravity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gains_match_reference() {
        let p = QuadParams::default();
        assert!((p.kx - 4.0 * 4.34).abs() < 1e-4);
        assert!((p.kv - 5.6 * 4.34).abs() < 1e-4);
        assert!((p.kr - 8.81).abs() < 1e-4);
        assert!((p.komega - 2.54).abs() < 1e-4);
    }

    #[test]
    fn j_inv_is_inverse_of_j() {
        let p = QuadParams::default();
        let prod = p.j * p.j_inv;
        for i in 0..3 {
            for k in 0..3 {
                let expect = if i == k { 1.0 } else { 0.0 };
                assert!((prod.get(i, k) - expect).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn hover_thrust_is_mg() {
        let p = QuadParams::default();
        assert!((p.hover_thrust() - 4.34 * 9.81).abs() < 1e-3);
    }
}
