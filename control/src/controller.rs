//! The geometric SE(3) tracking controller — a faithful `f32` port of
//! `se3quad .../controllers/{mod,geometric}.rs` (Lee et al.).
//!
//! The position/velocity/attitude outer loop builds a desired attitude `Rc` and a
//! thrust magnitude; the inner loop forms the coordinate-free SO(3) attitude error
//! `eR = ½·vee(Rcᵀ·R − Rᵀ·Rc)` and the moment law (feedback + gyroscopic + full
//! Appendix-F SO(3) feedforward). The moment is unit-agnostic; a normalised
//! [`crate::mixer`] turns `(thrust, moment)` into per-motor commands.
//!
//! # Z-up convention
//!
//! The reference is NED (gravity `+e3`, thrust along `−R·e3`). The firmware world
//! is **Z-up** (thrust along `+R·e3`, gravity `−g·e3`), so exactly five lines of the
//! outer loop flip sign relative to the reference — the gravity term in `a`, the
//! thrust projection `f`, and `b3c` with its first two time-derivatives. Every other
//! line (the `b1c`/`b2c` construction, `Rc` rates, the whole moment law, `eR`, `Ψ`)
//! is frame-agnostic and identical to the reference. Each flip is marked `Z-up:`.

use crate::dirty_derivative::DirtyDerivative;
use crate::math::{hat, vee, Mat3, Vec3};
use crate::params::QuadParams;
use crate::safety::Safety;
use crate::trajectory::{ControlMode, Target};
use crate::QuadState;

/// Coordinate-free SO(3) attitude error `eR = ½·vee(Rcᵀ·R − Rᵀ·Rc)`.
///
/// Global on SO(3); vanishes at `R == Rc` (and, as an unstable equilibrium, at a
/// 180° error where `sin(angle) → 0`).
pub fn attitude_error(r: &Mat3, rc: &Mat3) -> Vec3 {
    0.5 * vee(&(rc.transpose() * *r - r.transpose() * *rc))
}

/// Output of one controller evaluation.
#[derive(Clone, Copy, Debug)]
pub struct ControlOutput {
    /// Total thrust magnitude `f` [N].
    pub f: f32,
    /// Body moment `M` [N·m].
    pub moment: Vec3,
    /// Commanded attitude `Rc`.
    pub rc: Mat3,
    /// Commanded body angular velocity `Ωc`.
    pub omega_c: Vec3,
    /// Attitude error `eR` used by the moment law.
    pub er: Vec3,
    /// SO(3) configuration error `Ψ = ½·tr(I − Rcᵀ·R) ∈ [0, 2]`.
    pub psi: f32,
}

/// A stateful geometric tracking controller.
#[derive(Clone, Copy, Debug)]
pub struct GeometricController {
    dv1: DirtyDerivative,
    dv2: DirtyDerivative,
    /// Position-error integral accumulator (only active when `ki > 0`).
    ix: Vec3,
}

impl GeometricController {
    pub fn new(p: &QuadParams) -> Self {
        GeometricController {
            dv1: DirtyDerivative::new(1, p.tau, p.ts),
            dv2: DirtyDerivative::new(2, p.tau * 10.0, p.ts),
            ix: Vec3::ZERO,
        }
    }

    /// Reset the integrator and velocity-derivative filters (call on (re)arm).
    pub fn reset(&mut self, p: &QuadParams) {
        *self = GeometricController::new(p);
    }

    /// Integrate the position error with anti-windup clamping.
    fn integrate(&mut self, ex: &Vec3, p: &QuadParams) -> Vec3 {
        self.ix = self.ix + *ex * p.ts;
        self.ix = self.ix.map(|c| c.clamp(-p.i_max, p.i_max));
        self.ix
    }

    /// Evaluate the control law for the current `state` and `target`.
    ///
    /// `thrust_override` (used in [`ControlMode::Attitude`] / RC angle mode) sets the
    /// total thrust directly, e.g. from the throttle stick; `None` holds hover.
    pub fn control(
        &mut self,
        state: &QuadState,
        target: &Target,
        p: &QuadParams,
        cmode: ControlMode,
        safety: &Safety,
        thrust_override: Option<f32>,
    ) -> ControlOutput {
        let e3 = Vec3::new(0.0, 0.0, 1.0);
        let xd = target.xd;
        let b1d = target.b1d;
        let (x, v, r, omega) = (state.p, state.v, state.r, state.omega);

        // Desired-trajectory derivatives come analytically from the target.
        let (xd_1, xd_2, xd_3, xd_4) = (
            target.xd_dot,
            target.xd_ddot,
            target.xd_3dot,
            target.xd_4dot,
        );
        let (b1d_1, b1d_2) = (target.b1d_dot, target.b1d_ddot);
        // The measured velocity is differentiated numerically (feedback signal).
        let v_1 = self.dv1.calculate(v);
        let v_2 = self.dv2.calculate(v_1);

        // Velocity mode drops the position P gain (and its feedforward); the
        // integral only applies when there is a position reference.
        let kx = if cmode == ControlMode::Velocity { 0.0 } else { p.kx };
        let ki = if cmode == ControlMode::Position { p.ki } else { 0.0 };

        // Position / velocity / accel / jerk errors (eq. 17-18).
        let ex = x - xd;
        let ev = v - xd_1;
        let ea = v_1 - xd_2;
        let ej = v_2 - xd_3;

        // Integral term (anti-windup); snapshot to freeze on thrust saturation.
        let ix_prev = self.ix;
        let ix = if ki > 0.0 {
            self.integrate(&ex, p)
        } else {
            Vec3::ZERO
        };

        // Desired specific-force vector `a` (points along the desired thrust axis,
        // up at hover). Z-up: gravity term is `+ m·g·e3` (reference uses `−`).
        let a = -kx * ex - p.kv * ev - ki * ix + p.mass * p.gravity * e3 + p.mass * xd_2;
        let na = a.norm().max(1e-9);
        let na3 = na * na * na;
        let na5 = na3 * na * na;
        // Z-up: thrust is the projection onto the body-up axis `+R·e3`.
        let f_pos = a.dot(&(r * e3));

        // Desired body axes (eq. 23, 38). Safety caps the commanded tilt first.
        // Z-up: `b3c = +a/‖a‖` (reference uses `−a/‖a‖`).
        let b3c = safety.limit_tilt(a * (1.0 / na));
        let c = b3c.cross(&b1d);
        let nc = c.norm().max(1e-9);
        let nc3 = nc * nc * nc;
        let nc5 = nc3 * nc * nc;
        let b2c = c * (1.0 / nc);
        let b1c = -(b3c.cross(&c)) * (1.0 / nc);
        let rc = Mat3::from_columns(b1c, b2c, b3c);

        // First derivatives of the body axes (arXiv:1003.2005, Appendix F).
        let a_1 = -kx * ev - p.kv * ea - ki * ex + p.mass * xd_3;
        // Z-up: `d/dt(a/‖a‖)` with the `+a/‖a‖` sign.
        let b3c_1 = a_1 * (1.0 / na) - (a.dot(&a_1) / na3) * a;
        let c_1 = b3c_1.cross(&b1d) + b3c.cross(&b1d_1);
        let b2c_1 = c_1 * (1.0 / nc) - (c.dot(&c_1) / nc3) * c;
        let b1c_1 = b2c_1.cross(&b3c) + b2c.cross(&b3c_1);

        // Second derivatives.
        let a_2 = -kx * ea - p.kv * ej - ki * ev + p.mass * xd_4;
        // Z-up: second derivative of `+a/‖a‖`.
        let b3c_2 = a_2 * (1.0 / na) - (2.0 / na3) * a.dot(&a_1) * a_1
            - ((a_1.norm_squared() + a.dot(&a_2)) / na3) * a
            + (3.0 / na5) * a.dot(&a_1) * a.dot(&a_1) * a;
        let c_2 = b3c_2.cross(&b1d) + b3c.cross(&b1d_2) + 2.0 * b3c_1.cross(&b1d_1);
        let b2c_2 = c_2 * (1.0 / nc) - (2.0 / nc3) * c.dot(&c_1) * c_1
            - ((c_1.norm_squared() + c.dot(&c_2)) / nc3) * c
            + (3.0 / nc5) * c.dot(&c_1) * c.dot(&c_1) * c;
        let b1c_2 = b2c_2.cross(&b3c) + b2c.cross(&b3c_2) + 2.0 * b2c_1.cross(&b3c_1);

        let rc_1 = Mat3::from_columns(b1c_1, b2c_1, b3c_1);
        let rc_2 = Mat3::from_columns(b1c_2, b2c_2, b3c_2);

        // In Attitude mode the controller is handed `Rd`, `Ωd`, `Ω̇d` directly and
        // thrust comes from `thrust_override` (hover if none). Otherwise the
        // attitude command and its rates come from the thrust-direction construction.
        let (rc, omega_c, omega_c_1, f) = if cmode == ControlMode::Attitude {
            (
                target.rd,
                target.omega_d,
                target.omega_d_dot,
                thrust_override.unwrap_or_else(|| p.hover_thrust()),
            )
        } else {
            let omega_c = vee(&(rc.transpose() * rc_1));
            let omega_c_1 = vee(&(rc.transpose() * rc_2 - hat(&omega_c) * hat(&omega_c)));
            let f = thrust_override.unwrap_or(f_pos);
            (rc, omega_c, omega_c_1, f)
        };

        // Attitude + angular-velocity error (eq. 21).
        let er = attitude_error(&r, &rc);
        let e_omega = omega - r.transpose() * rc * omega_c;

        // Geometric moment law: feedback + gyroscopic + SO(3) feedforward.
        let moment = -p.kr * er - p.komega * e_omega + omega.cross(&(p.j * omega))
            - p.j * (hat(&omega) * r.transpose() * rc * omega_c - r.transpose() * rc * omega_c_1);

        let psi = 0.5 * (Mat3::identity() - rc.transpose() * r).trace();

        // Safety: clamp total thrust; inhibit torque when disarmed.
        let f_raw = f;
        let f = safety.clamp_thrust(f_raw);
        // Conditional anti-windup: freeze the integrator if thrust saturated.
        if ki > 0.0 && (f - f_raw).abs() > 1e-6 {
            self.ix = ix_prev;
        }
        let moment = if safety.inhibited() { Vec3::ZERO } else { moment };

        ControlOutput {
            f,
            moment,
            rc,
            omega_c,
            er,
            psi,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::so3_exp;
    use core::f32::consts::PI;

    #[test]
    fn attitude_error_vanishes_at_r_equals_rc() {
        let r = so3_exp(&Vec3::new(0.2, -0.3, 0.5));
        let er = attitude_error(&r, &r);
        assert!(er.norm() < 1e-5, "eR should vanish at R==Rc: {er:?}");
    }

    #[test]
    fn attitude_error_points_along_error_axis_small() {
        // Small pitch error → eR aligned with +y, modest magnitude.
        let r = so3_exp(&Vec3::new(0.0, 0.1, 0.0));
        let rc = Mat3::identity();
        let er = attitude_error(&r, &rc);
        assert!(er.x.abs() < 1e-4 && er.z.abs() < 1e-4);
        assert!(er.y > 0.05 && er.y < 0.15);
    }

    #[test]
    fn geometric_error_vanishes_at_180() {
        // At exactly 180° the geometric error vanishes (unstable equilibrium).
        let r = so3_exp(&Vec3::new(PI, 0.0, 0.0));
        let rc = Mat3::identity();
        let er = attitude_error(&r, &rc);
        assert!(er.norm() < 1e-4, "geometric error should vanish at 180°: {er:?}");
    }

    #[test]
    fn hover_gives_mg_thrust_and_zero_moment() {
        // Level, at rest, holding the origin: thrust ≈ m·g, moment ≈ 0.
        let p = QuadParams::default();
        let mut c = GeometricController::new(&p);
        let state = QuadState::at(Vec3::ZERO, Mat3::identity());
        let target = Target::hold(Vec3::ZERO, Vec3::unit_x());
        let out = c.control(&state, &target, &p, ControlMode::Position, &Safety::off(), None);
        assert!((out.f - p.hover_thrust()).abs() < 1e-2, "thrust not m·g: {}", out.f);
        assert!(out.moment.norm() < 1e-3, "moment not zero: {:?}", out.moment);
        assert!(out.psi.abs() < 1e-4, "Ψ not zero: {}", out.psi);
    }

    #[test]
    fn position_error_tilts_toward_target() {
        // Target 1 m to +x (North): the controller should command a tilt whose
        // body-up axis leans toward +x, i.e. b3c.x > 0.
        let p = QuadParams::default();
        let mut c = GeometricController::new(&p);
        let state = QuadState::at(Vec3::ZERO, Mat3::identity());
        let target = Target::hold(Vec3::new(1.0, 0.0, 0.0), Vec3::unit_x());
        let out = c.control(&state, &target, &p, ControlMode::Position, &Safety::off(), None);
        // Desired body-up (third column of Rc) leans toward +x.
        assert!(out.rc.col(2).x > 0.02, "did not tilt toward target: {:?}", out.rc.col(2));
    }

    #[test]
    fn attitude_mode_uses_thrust_override() {
        let p = QuadParams::default();
        let mut c = GeometricController::new(&p);
        let state = QuadState::at(Vec3::ZERO, Mat3::identity());
        let target = Target::hold_attitude(Mat3::identity());
        let out = c.control(&state, &target, &p, ControlMode::Attitude, &Safety::off(), Some(30.0));
        assert!((out.f - 30.0).abs() < 1e-4);
        assert!(out.moment.norm() < 1e-3);
    }
}
