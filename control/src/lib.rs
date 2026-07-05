//! `scky-control` — the geometric SE(3) tracking flight-control law for scky-fc.
//!
//! This is a faithful `f32` port of the simulator controller in
//! `se3quad/rust/core` (Lee et al., "Geometric tracking control of a quadrotor
//! UAV on SE(3)"). It runs the position/velocity/attitude outer loop, builds a
//! desired attitude `Rc`, computes the coordinate-free SO(3) attitude error and
//! the full moment law (feedback + gyroscopic + SO(3) feedforward), and hands
//! back a thrust magnitude and body moment. A separate [`mixer`] turns that
//! wrench into normalised per-motor commands for the analog ESCs.
//!
//! # `no_std` in the firmware, `std` in tests
//!
//! The crate is `no_std` when built into the firmware, but the `test` cfg drops
//! that so the whole control law can be unit-tested on the host:
//!
//! ```text
//! cargo test --target x86_64-unknown-linux-gnu -p scky-control
//! ```
//!
//! Every module carries its own `#[cfg(test)]` tests, and `tests/closed_loop.rs`
//! exercises the whole loop against a rigid-body plant.
//!
//! # Frames and units
//!
//! The controller works in the **NED** convention of the reference (gravity along
//! `+e3`, Z down). The firmware's AHRS/EKF report **North/East/Up (Z up)**, so the
//! [`frame`] adapter converts state in and moment/attitude out. All angles are in
//! radians, distances in metres, and the physical [`params::QuadParams`] default to
//! the simulator's numbers so the ported tests reproduce its results exactly.

#![cfg_attr(not(test), no_std)]

pub mod controller;
pub mod dirty_derivative;
pub mod frame;
pub mod math;
pub mod mixer;
pub mod params;
pub mod rc;
pub mod safety;
pub mod trajectory;

// Test-only: a rigid-body plant + end-to-end closed-loop tests. Gated behind
// `cfg(test)` so neither is ever compiled into the firmware.
#[cfg(test)]
mod closed_loop;
#[cfg(test)]
mod plant;

pub use controller::{ControlOutput, GeometricController};
pub use math::{Mat3, Vec3};
pub use mixer::{MixerGains, Motors};
pub use params::QuadParams;
pub use rc::{RcInput, RcMap};
pub use safety::Safety;
pub use trajectory::{ControlMode, Target, Trajectory};

/// Full rigid-body state consumed by the controller, in the NED convention.
///
/// * `p` — position in the inertial frame [m],
/// * `v` — velocity in the inertial frame [m/s],
/// * `r` — body→inertial rotation `R ∈ SO(3)`,
/// * `omega` — angular velocity in the body frame [rad/s].
#[derive(Clone, Copy, Debug)]
pub struct QuadState {
    pub p: Vec3,
    pub v: Vec3,
    pub r: Mat3,
    pub omega: Vec3,
}

impl QuadState {
    /// State at rest at position `p` with orientation `r`.
    pub fn at(p: Vec3, r: Mat3) -> Self {
        Self {
            p,
            v: Vec3::ZERO,
            r,
            omega: Vec3::ZERO,
        }
    }

    /// Whether every component is finite (divergence guard for the firmware).
    pub fn is_finite(&self) -> bool {
        self.p.is_finite() && self.v.is_finite() && self.r.is_finite() && self.omega.is_finite()
    }
}
