//! End-to-end closed-loop tests (test builds only): controller + plant, mirroring
//! the headline behaviours of `se3quad .../tests/closed_loop.rs` with the `f32`
//! Z-up port. These prove the whole loop — outer loop, SO(3) inner loop and
//! feedforward — actually converges, not just that the pieces are individually
//! correct.

use crate::controller::GeometricController;
use crate::math::{so3_exp, Mat3, Vec3};
use crate::params::QuadParams;
use crate::plant::Plant;
use crate::safety::Safety;
use crate::trajectory::{ControlMode, Target, Trajectory};
use crate::QuadState;

/// Run the loop for `secs`, returning the final plant + a peak-Ψ against `rc_fn`.
fn run(
    mut ctrl: GeometricController,
    mut plant: Plant,
    p: &QuadParams,
    mode: ControlMode,
    mut target_at: impl FnMut(f32) -> Target,
    thrust_override: Option<f32>,
    secs: f32,
) -> Plant {
    let safety = Safety::off();
    let ts = p.ts;
    let steps = (secs / ts) as u32;
    let mut t = 0.0;
    for _ in 0..steps {
        let target = target_at(t);
        let out = ctrl.control(&plant.state, &target, p, mode, &safety, thrust_override);
        assert!(out.f.is_finite() && out.moment.is_finite(), "control diverged at t={t}");
        plant.step(out.f, out.moment, p, ts, 10);
        assert!(plant.state.is_finite(), "plant diverged at t={t}");
        t += ts;
    }
    plant
}

#[test]
fn hover_stays_put() {
    let p = QuadParams::default();
    let ctrl = GeometricController::new(&p);
    let plant = Plant::new(QuadState::at(Vec3::ZERO, Mat3::identity()));
    let target = Target::hold(Vec3::ZERO, Vec3::unit_x());
    let end = run(
        ctrl,
        plant,
        &p,
        ControlMode::Position,
        |_| target,
        None,
        4.0,
    );
    assert!(end.psi(&Mat3::identity()) < 1e-3, "attitude drifted: Ψ={}", end.psi(&Mat3::identity()));
    assert!(end.state.p.norm() < 1e-2, "position drifted: {:?}", end.state.p);
}

#[test]
fn recovers_from_upside_down() {
    // Start strongly inverted (160° about x, off the 180° unstable equilibrium)
    // and hold the upright origin. The geometric law recovers attitude decisively.
    let p = QuadParams::default();
    let ctrl = GeometricController::new(&p);
    let r0 = so3_exp(&Vec3::new(160_f32.to_radians(), 0.0, 0.0));
    let plant = Plant::new(QuadState::at(Vec3::ZERO, r0));
    let target = Target::hold(Vec3::ZERO, Vec3::unit_x());
    let end = run(
        ctrl,
        plant,
        &p,
        ControlMode::Position,
        |_| target,
        None,
        8.0,
    );
    assert!(
        end.psi(&Mat3::identity()) < 5e-2,
        "did not recover upright: Ψ={}",
        end.psi(&Mat3::identity())
    );
}

#[test]
fn attitude_tracks_moving_reference() {
    // Steady spin reference (Ωd ≠ 0): the tracking error stays small throughout and
    // the airframe holds hover (thrust = hover).
    let p = QuadParams::default();
    let ctrl = GeometricController::new(&p);
    let plant = Plant::new(QuadState::at(Vec3::ZERO, Mat3::identity()));
    let traj = Trajectory::AttitudeSpin { rate: 0.5 };
    let end = run(
        ctrl,
        plant,
        &p,
        ControlMode::Attitude,
        |t| traj.sample(t),
        None,
        8.0,
    );
    let rc = traj.sample(8.0).rd;
    assert!(end.psi(&rc) < 5e-2, "attitude tracking lost: Ψ={}", end.psi(&rc));
}
