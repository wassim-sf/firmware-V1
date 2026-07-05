//! RC stick mapping: raw channel microseconds → a desired attitude + throttle for
//! the manual (self-levelling "angle") flight mode.
//!
//! The channel assignment is **configurable** ([`RcMap`]); the default is standard
//! **AETR** — roll = ch0, pitch = ch1, throttle = ch2, yaw = ch3 — with the arm
//! switch on ch4 (CH5, 1-based). Roll/pitch sticks command a tilt-limited bank
//! angle, the yaw stick commands a yaw *rate* (integrated into a heading setpoint),
//! and the throttle stick sets the collective. Per-axis `inv_*` flags let the sign
//! of each axis be corrected from config after the props-off bench check, without a
//! code change.

use crate::math::{so3_exp, Vec3};
use crate::trajectory::Target;

/// RC channel map + stick calibration + manual-mode limits.
#[derive(Clone, Copy, Debug)]
pub struct RcMap {
    /// 0-based channel indices.
    pub ch_roll: usize,
    pub ch_pitch: usize,
    pub ch_throttle: usize,
    pub ch_yaw: usize,
    pub ch_arm: usize,
    /// Stick endpoints / centre in microseconds (988 / 1500 / 2012 typical).
    pub us_min: u16,
    pub us_mid: u16,
    pub us_max: u16,
    /// Arm when the arm channel is at/above this (µs).
    pub arm_us: u16,
    /// Manual-mode limits.
    pub max_tilt: f32,     // rad, full roll/pitch stick
    pub max_yaw_rate: f32, // rad/s, full yaw stick
    /// Deadband around centre for roll/pitch/yaw, as a fraction of half-range.
    pub deadband: f32,
    /// Per-axis sign inversion (set from config after the bench check).
    pub inv_roll: bool,
    pub inv_pitch: bool,
    pub inv_yaw: bool,
}

impl Default for RcMap {
    fn default() -> Self {
        RcMap {
            ch_roll: 0,
            ch_pitch: 1,
            ch_throttle: 2,
            ch_yaw: 3,
            ch_arm: 4,
            us_min: 988,
            us_mid: 1500,
            us_max: 2012,
            arm_us: 1700,
            max_tilt: 35_f32 * core::f32::consts::PI / 180.0,
            max_yaw_rate: 180_f32 * core::f32::consts::PI / 180.0,
            deadband: 0.03,
            inv_roll: false,
            inv_pitch: false,
            inv_yaw: false,
        }
    }
}

/// One mapped manual command.
#[derive(Clone, Copy, Debug)]
pub struct RcInput {
    /// Desired attitude + yaw-rate feedforward for `ControlMode::Attitude`.
    pub target: Target,
    /// Collective throttle fraction `[0, 1]`.
    pub throttle: f32,
    /// Whether the arm switch requests arming (gating happens in the firmware).
    pub arm_requested: bool,
}

impl RcMap {
    /// Symmetric stick value in `[-1, 1]` about centre, with deadband applied.
    fn axis(&self, us: u16) -> f32 {
        let mid = self.us_mid as f32;
        let half = ((self.us_max as f32 - self.us_min as f32) * 0.5).max(1.0);
        let mut n = (us as f32 - mid) / half;
        n = n.clamp(-1.0, 1.0);
        if n.abs() < self.deadband {
            0.0
        } else {
            // Rescale so the response is continuous just outside the deadband.
            let s = n.signum();
            s * ((n.abs() - self.deadband) / (1.0 - self.deadband))
        }
    }

    /// Throttle stick value in `[0, 1]`.
    fn throttle(&self, us: u16) -> f32 {
        let lo = self.us_min as f32;
        let span = (self.us_max as f32 - lo).max(1.0);
        ((us as f32 - lo) / span).clamp(0.0, 1.0)
    }

    /// Map raw channel microseconds to a manual command. `yaw_sp` is the persistent
    /// heading setpoint (radians), advanced by the yaw stick over `dt` seconds.
    pub fn map(&self, us: &[u16], dt: f32, yaw_sp: &mut f32) -> RcInput {
        let get = |i: usize| us.get(i).copied().unwrap_or(self.us_mid);

        let sroll = self.axis(get(self.ch_roll)) * if self.inv_roll { -1.0 } else { 1.0 };
        let spitch = self.axis(get(self.ch_pitch)) * if self.inv_pitch { -1.0 } else { 1.0 };
        let syaw = self.axis(get(self.ch_yaw)) * if self.inv_yaw { -1.0 } else { 1.0 };

        let roll = sroll * self.max_tilt;
        let pitch = spitch * self.max_tilt;
        let yaw_rate = syaw * self.max_yaw_rate;

        // Advance and wrap the heading setpoint.
        *yaw_sp += yaw_rate * dt;
        if *yaw_sp > core::f32::consts::PI {
            *yaw_sp -= 2.0 * core::f32::consts::PI;
        } else if *yaw_sp < -core::f32::consts::PI {
            *yaw_sp += 2.0 * core::f32::consts::PI;
        }

        // Desired attitude Rd = Rz(yaw) · Ry(pitch) · Rx(roll) (aerospace ZYX).
        let rd = so3_exp(&Vec3::new(0.0, 0.0, *yaw_sp))
            * so3_exp(&Vec3::new(0.0, pitch, 0.0))
            * so3_exp(&Vec3::new(roll, 0.0, 0.0));

        let target = Target {
            omega_d: Vec3::new(0.0, 0.0, yaw_rate),
            ..Target::hold_attitude(rd)
        };

        RcInput {
            target,
            throttle: self.throttle(get(self.ch_throttle)),
            arm_requested: get(self.ch_arm) >= self.arm_us,
        }
    }

    /// Whether the throttle stick is low enough to permit arming.
    pub fn throttle_low(&self, us: &[u16]) -> bool {
        let t = self.throttle(us.get(self.ch_throttle).copied().unwrap_or(self.us_min));
        t < 0.05
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn centered() -> [u16; 16] {
        let mut a = [1500u16; 16];
        a[2] = 988; // throttle low
        a[4] = 1000; // disarmed
        a
    }

    #[test]
    fn centre_sticks_give_level_attitude() {
        let map = RcMap::default();
        let mut yaw = 0.0;
        let cmd = map.map(&centered(), 0.01, &mut yaw);
        // Rd ≈ identity.
        for i in 0..3 {
            for j in 0..3 {
                let e = if i == j { 1.0 } else { 0.0 };
                assert!((cmd.target.rd.get(i, j) - e).abs() < 1e-5);
            }
        }
        assert!(cmd.throttle < 0.05);
        assert!(!cmd.arm_requested);
    }

    #[test]
    fn full_roll_reaches_max_tilt() {
        let map = RcMap::default();
        let mut a = centered();
        a[0] = map.us_max; // full roll
        let mut yaw = 0.0;
        let cmd = map.map(&a, 0.01, &mut yaw);
        // The body-up axis (3rd column of Rd) tilts by ~max_tilt from vertical.
        let up = cmd.target.rd.col(2);
        let tilt = libm::acosf(up.z.clamp(-1.0, 1.0));
        assert!((tilt - map.max_tilt).abs() < 1e-3, "tilt {tilt} != {}", map.max_tilt);
    }

    #[test]
    fn arm_switch_high_requests_arm() {
        let map = RcMap::default();
        let mut a = centered();
        a[4] = 1900;
        let mut yaw = 0.0;
        assert!(map.map(&a, 0.01, &mut yaw).arm_requested);
    }

    #[test]
    fn yaw_stick_advances_heading_setpoint() {
        let map = RcMap::default();
        let mut a = centered();
        a[3] = map.us_max; // full yaw
        let mut yaw = 0.0;
        map.map(&a, 0.1, &mut yaw);
        assert!(yaw > 0.0, "yaw setpoint should advance: {yaw}");
        assert!((yaw - map.max_yaw_rate * 0.1).abs() < 1e-3);
    }

    #[test]
    fn throttle_low_gate() {
        let map = RcMap::default();
        let mut a = centered();
        assert!(map.throttle_low(&a));
        a[2] = map.us_max;
        assert!(!map.throttle_low(&a));
    }

    #[test]
    fn deadband_zeroes_small_inputs() {
        let map = RcMap::default();
        let mut a = centered();
        a[0] = map.us_mid + 3; // tiny nudge, within deadband
        let mut yaw = 0.0;
        let cmd = map.map(&a, 0.01, &mut yaw);
        let up = cmd.target.rd.col(2);
        assert!(up.z > 0.9999, "deadband did not suppress tiny input");
    }
}
