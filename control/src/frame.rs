//! Frame convention + state assembly.
//!
//! The reference simulator works in **NED** (gravity along `+e3`, Z down, body-Z
//! down at hover). The firmware's AHRS/EKF instead report **North / East / Up**
//! (Z up): the Mahony filter snaps level to `R = I` with the body Z axis reading
//! `+1 g` *up*, and the EKF publishes position/velocity as N/E/Up.
//!
//! Converting that into the simulator's NED frame would require an *improper*
//! (reflection) transform — N/E/Up is a left-handed labelling of physical space —
//! which is under-determined without knowing the exact hardware handedness. So
//! instead the controller runs **natively in the firmware's Z-up world**:
//!
//! * world up is `e3 = (0, 0, 1)`, gravity acts along `-e3`,
//! * thrust acts along the body `+e3` axis (`f · R·e3`),
//! * `R = quat_to_mat(q)` is body→world, `Ω` is the body-frame gyro.
//!
//! The coordinate-free SO(3) attitude error and the whole moment law are
//! frame-agnostic, so they are ported verbatim; only the outer-loop gravity/up
//! sign differs from the reference (see [`crate::controller`]). This module holds
//! the small, testable glue that turns raw AHRS/EKF outputs into a [`QuadState`].

use crate::math::{quat_to_mat, Vec3};
use crate::QuadState;

const DEG2RAD: f32 = core::f32::consts::PI / 180.0;

/// World up axis `e3` (Z up).
pub const E3_UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Assemble the controller [`QuadState`] from firmware estimates, all in the
/// Z-up world convention:
///
/// * `q` — AHRS attitude quaternion `[w, x, y, z]` (body→world),
/// * `rates_dps` — bias-corrected body angular rates [deg/s],
/// * `pos_neu` — position N/E/Up [m] (e.g. EKF `NavSolution::pos`),
/// * `vel_neu` — velocity N/E/Up [m/s] (e.g. EKF `NavSolution::vel`).
pub fn state_from_ahrs(
    q: [f32; 4],
    rates_dps: [f32; 3],
    pos_neu: [f32; 3],
    vel_neu: [f32; 3],
) -> QuadState {
    QuadState {
        p: Vec3::new(pos_neu[0], pos_neu[1], pos_neu[2]),
        v: Vec3::new(vel_neu[0], vel_neu[1], vel_neu[2]),
        r: quat_to_mat(q),
        omega: Vec3::new(
            rates_dps[0] * DEG2RAD,
            rates_dps[1] * DEG2RAD,
            rates_dps[2] * DEG2RAD,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Mat3;

    #[test]
    fn level_quaternion_is_identity_attitude() {
        let s = state_from_ahrs([1.0, 0.0, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]);
        for i in 0..3 {
            for j in 0..3 {
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!((s.r.get(i, j) - expect).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn rates_are_converted_to_radians() {
        let s = state_from_ahrs([1.0, 0.0, 0.0, 0.0], [180.0, 0.0, -90.0], [0.0; 3], [0.0; 3]);
        assert!((s.omega.x - core::f32::consts::PI).abs() < 1e-5);
        assert!((s.omega.z + core::f32::consts::FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn position_velocity_passed_through() {
        let s = state_from_ahrs(
            [1.0, 0.0, 0.0, 0.0],
            [0.0; 3],
            [1.0, -2.0, 3.0],
            [0.4, 0.5, -0.6],
        );
        assert_eq!(s.p, Vec3::new(1.0, -2.0, 3.0));
        assert_eq!(s.v, Vec3::new(0.4, 0.5, -0.6));
    }

    #[test]
    fn e3_is_world_up() {
        // Hover thrust direction: body +e3 mapped through a level attitude is up.
        let r = Mat3::identity();
        assert_eq!(r * E3_UP, E3_UP);
    }
}
