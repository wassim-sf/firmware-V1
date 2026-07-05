//! Desired trajectories and the [`Target`] flat-output bundle the controller
//! tracks. Ports `se3quad .../trajectory.rs` to `f32`.
//!
//! Each trajectory produces a desired position `xd` and heading direction `b1d`
//! **with their analytic time-derivatives**, which makes the Appendix-F
//! feedforward exact. Constant/stepped targets leave the derivatives at zero.
//!
//! Heights follow the firmware's **Z-up** convention (up is `+z`), unlike the
//! reference's NED. For RC flight the [`Target`] is built by [`crate::rc`]; these
//! presets drive the autonomous modes and the closed-loop tests.

use crate::math::{so3_exp, Mat3, Vec3};

/// Which quantity the controller tracks.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum ControlMode {
    /// Track a desired position `xd` (and heading `b1d`). Full outer loop.
    #[default]
    Position,
    /// Track a desired velocity `ẋd` (and heading); no position feedback.
    Velocity,
    /// Track a desired attitude `Rd`, `Ωd` directly; thrust is supplied separately
    /// (hover in the sim; the RC throttle stick in flight).
    Attitude,
}

impl ControlMode {
    pub fn label(&self) -> &'static str {
        match self {
            ControlMode::Position => "Position",
            ControlMode::Velocity => "Velocity",
            ControlMode::Attitude => "Attitude",
        }
    }
}

/// Desired flat outputs (and derivatives) fed to the controller.
#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub xd: Vec3,
    pub xd_dot: Vec3,
    pub xd_ddot: Vec3,
    pub xd_3dot: Vec3,
    pub xd_4dot: Vec3,
    pub b1d: Vec3,
    pub b1d_dot: Vec3,
    pub b1d_ddot: Vec3,
    /// Directly commanded attitude (used in [`ControlMode::Attitude`]).
    pub rd: Mat3,
    pub omega_d: Vec3,
    pub omega_d_dot: Vec3,
}

impl Target {
    /// A held setpoint: position `xd`, heading `b1d`, all derivatives zero.
    pub fn hold(xd: Vec3, b1d: Vec3) -> Self {
        let z = Vec3::ZERO;
        Target {
            xd,
            xd_dot: z,
            xd_ddot: z,
            xd_3dot: z,
            xd_4dot: z,
            b1d,
            b1d_dot: z,
            b1d_ddot: z,
            rd: Mat3::identity(),
            omega_d: z,
            omega_d_dot: z,
        }
    }

    /// A held *attitude* setpoint `rd` (for [`ControlMode::Attitude`]).
    pub fn hold_attitude(rd: Mat3) -> Self {
        Target {
            rd,
            ..Target::hold(Vec3::ZERO, Vec3::unit_x())
        }
    }
}

impl Default for Target {
    fn default() -> Self {
        Target::hold(Vec3::ZERO, Vec3::unit_x())
    }
}

/// Heading direction from a yaw angle (radians).
pub fn yaw_to_b1d(yaw: f32) -> Vec3 {
    Vec3::new(libm::cosf(yaw), libm::sinf(yaw), 0.0)
}

/// A parametric desired trajectory.
#[derive(Clone, Copy, Debug)]
pub enum Trajectory {
    /// Hold a fixed position and heading.
    Hover { pos: Vec3, yaw: f32 },
    /// Jump from `from` to `to` at `t_step` seconds.
    Step {
        from: Vec3,
        to: Vec3,
        t_step: f32,
        yaw: f32,
    },
    /// Horizontal circle of `radius` at angular rate `omega`, heading tangent.
    Circle { radius: f32, omega: f32, height: f32 },
    /// Horizontal figure-eight (Lissajous 1:2).
    Figure8 { scale: f32, omega: f32, height: f32 },
    /// Climbing helix.
    Helix {
        radius: f32,
        omega: f32,
        climb_rate: f32,
    },
    /// Hold a fixed attitude `exp(hat(axis_angle))` (for [`ControlMode::Attitude`]).
    HoldAttitude { axis_angle: Vec3 },
    /// Spin in place about the vertical at `rate` rad/s — a moving attitude
    /// reference `Rd(t) = Rz(rate·t)`, `Ωd = (0, 0, rate)`.
    AttitudeSpin { rate: f32 },
}

impl Trajectory {
    /// Sample the trajectory at time `t`.
    pub fn sample(&self, t: f32) -> Target {
        match *self {
            Trajectory::Hover { pos, yaw } => Target::hold(pos, yaw_to_b1d(yaw)),
            Trajectory::Step {
                from,
                to,
                t_step,
                yaw,
            } => Target::hold(if t < t_step { from } else { to }, yaw_to_b1d(yaw)),
            Trajectory::Circle {
                radius,
                omega,
                height,
            } => circle(radius, omega, height, 0.0, omega, t),
            Trajectory::Figure8 {
                scale,
                omega,
                height,
            } => figure8(scale, omega, height, t),
            Trajectory::Helix {
                radius,
                omega,
                climb_rate,
            } => circle(radius, omega, 0.0, climb_rate, omega, t),
            Trajectory::HoldAttitude { axis_angle } => Target::hold_attitude(so3_exp(&axis_angle)),
            Trajectory::AttitudeSpin { rate } => {
                let rd = so3_exp(&Vec3::new(0.0, 0.0, rate * t));
                Target {
                    omega_d: Vec3::new(0.0, 0.0, rate),
                    ..Target::hold_attitude(rd)
                }
            }
        }
    }
}

/// Circle / helix with analytic derivatives. `z0 + vz·t` gives the vertical
/// profile (`vz = 0` ⇒ flat circle), heading tangent to the path.
fn circle(radius: f32, omega: f32, z0: f32, vz: f32, w: f32, t: f32) -> Target {
    let (s, c) = (libm::sinf(omega * t), libm::cosf(omega * t));
    let (r, w2, w3, w4) = (radius, w * w, w * w * w, w * w * w * w);
    Target {
        xd: Vec3::new(r * c, r * s, z0 + vz * t),
        xd_dot: Vec3::new(-r * w * s, r * w * c, vz),
        xd_ddot: Vec3::new(-r * w2 * c, -r * w2 * s, 0.0),
        xd_3dot: Vec3::new(r * w3 * s, -r * w3 * c, 0.0),
        xd_4dot: Vec3::new(r * w4 * c, r * w4 * s, 0.0),
        b1d: Vec3::new(-s, c, 0.0),
        b1d_dot: Vec3::new(-w * c, -w * s, 0.0),
        b1d_ddot: Vec3::new(w2 * s, -w2 * c, 0.0),
        rd: Mat3::identity(),
        omega_d: Vec3::ZERO,
        omega_d_dot: Vec3::ZERO,
    }
}

/// Figure-eight (x = A·sin(ωt), y = (A/2)·sin(2ωt)) with analytic derivatives.
fn figure8(scale: f32, omega: f32, height: f32, t: f32) -> Target {
    let a = scale;
    let (w, w2, w3, w4) = (omega, omega * omega, omega * omega * omega, omega * omega * omega * omega);
    let (s1, c1) = (libm::sinf(w * t), libm::cosf(w * t));
    let (s2, c2) = (libm::sinf(2.0 * w * t), libm::cosf(2.0 * w * t));
    Target {
        xd: Vec3::new(a * s1, 0.5 * a * s2, height),
        xd_dot: Vec3::new(a * w * c1, a * w * c2, 0.0),
        xd_ddot: Vec3::new(-a * w2 * s1, -2.0 * a * w2 * s2, 0.0),
        xd_3dot: Vec3::new(-a * w3 * c1, -4.0 * a * w3 * c2, 0.0),
        xd_4dot: Vec3::new(a * w4 * s1, 8.0 * a * w4 * s2, 0.0),
        b1d: Vec3::unit_x(),
        b1d_dot: Vec3::ZERO,
        b1d_ddot: Vec3::ZERO,
        rd: Mat3::identity(),
        omega_d: Vec3::ZERO,
        omega_d_dot: Vec3::ZERO,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hold_has_zero_derivatives() {
        let t = Target::hold(Vec3::new(1.0, 2.0, 3.0), Vec3::unit_x());
        assert_eq!(t.xd_dot, Vec3::ZERO);
        assert_eq!(t.xd_ddot, Vec3::ZERO);
        assert_eq!(t.omega_d, Vec3::ZERO);
    }

    #[test]
    fn yaw_to_b1d_is_unit_and_points_right_way() {
        let b = yaw_to_b1d(0.0);
        assert!((b - Vec3::unit_x()).norm() < 1e-6);
        let b90 = yaw_to_b1d(core::f32::consts::FRAC_PI_2);
        assert!((b90 - Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-5);
    }

    #[test]
    fn circle_analytic_derivatives_match_finite_difference() {
        let traj = Trajectory::Circle {
            radius: 2.0,
            omega: 0.7,
            height: 1.5,
        };
        let t = 0.83;
        let h = 1e-3;
        let a = traj.sample(t - h);
        let b = traj.sample(t + h);
        let mid = traj.sample(t);
        // xd_dot ≈ central difference of xd.
        let fd_v = (b.xd - a.xd) * (1.0 / (2.0 * h));
        assert!((fd_v - mid.xd_dot).norm() < 1e-2, "xd_dot mismatch");
        // xd_ddot ≈ central difference of xd_dot.
        let fd_a = (b.xd_dot - a.xd_dot) * (1.0 / (2.0 * h));
        assert!((fd_a - mid.xd_ddot).norm() < 1e-2, "xd_ddot mismatch");
        // b1d stays a unit vector.
        assert!((mid.b1d.norm() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn attitude_spin_has_matching_rate() {
        let traj = Trajectory::AttitudeSpin { rate: 0.5 };
        let tgt = traj.sample(1.0);
        assert!((tgt.omega_d.z - 0.5).abs() < 1e-6);
    }
}
