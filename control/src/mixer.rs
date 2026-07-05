//! Normalised flight mixer: turns the controller's `(thrust, body-moment)` wrench
//! into four per-motor commands in `[0, 1]` for the analog ESCs.
//!
//! The geometric controller outputs a body moment in (roughly) N·m; on hardware
//! there is no calibrated thrust curve, so instead of inverting a physical mixing
//! matrix we distribute a **normalised** collective + differential:
//!
//! ```text
//! motor_i = throttle + k_roll·sχ·Mx + k_pitch·sψ·My + k_yaw·sζ·Mz
//! ```
//!
//! with a fixed quad-**X** sign table `s` and tunable gains. A differential-
//! preserving desaturation (airmode) keeps attitude authority when the sum would
//! clip. The gains `k_*` are the primary tuning knobs and are expected to be
//! retuned on the real airframe.
//!
//! # Logical motor order (X frame, body +x forward, +y right)
//!
//! | motor | position    | roll `Mx` | pitch `My` | yaw `Mz` |
//! |-------|-------------|-----------|------------|----------|
//! | M1    | front-right | `+`       | `−`        | `+`      |
//! | M2    | rear-right  | `+`       | `+`        | `−`      |
//! | M3    | rear-left   | `−`       | `+`        | `+`      |
//! | M4    | front-left  | `−`       | `−`        | `−`      |
//!
//! These are *logical* motors; the firmware [`crate::esc`] `output_map` places them
//! on the physical PWM channels, and prop spin directions are fixed by wiring — so
//! any board-specific sign fix is a remap/tuning concern, validated on the bench.

use crate::math::Vec3;

/// Quad-X sign table, rows = motors M1..M4, cols = (roll Mx, pitch My, yaw Mz).
pub const X_SIGNS: [[f32; 3]; 4] = [
    [1.0, -1.0, 1.0],  // M1 front-right
    [1.0, 1.0, -1.0],  // M2 rear-right
    [-1.0, 1.0, 1.0],  // M3 rear-left
    [-1.0, -1.0, -1.0], // M4 front-left
];

/// Tunable mixer gains (moment demand → throttle fraction) plus the throttle that
/// produces hover, used when the collective comes from the controller's thrust.
#[derive(Clone, Copy, Debug)]
pub struct MixerGains {
    pub k_roll: f32,
    pub k_pitch: f32,
    pub k_yaw: f32,
    /// Throttle fraction `[0,1]` at which the airframe hovers (maps controller
    /// thrust `f` to collective in the auto modes).
    pub hover_throttle: f32,
}

impl Default for MixerGains {
    fn default() -> Self {
        // Conservative bring-up defaults; retune on the airframe.
        MixerGains {
            k_roll: 0.02,
            k_pitch: 0.02,
            k_yaw: 0.02,
            hover_throttle: 0.5,
        }
    }
}

/// Four per-motor commands in `[0, 1]`, logical order M1..M4.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motors(pub [f32; 4]);

impl Motors {
    pub const ZERO: Motors = Motors([0.0; 4]);
    #[inline]
    pub fn as_array(&self) -> [f32; 4] {
        self.0
    }
}

/// Mix a collective `throttle ∈ [0,1]` and a body `moment` into four motor
/// commands, with differential-preserving desaturation (airmode) and a final
/// clamp to `[0, 1]`.
pub fn mix(throttle: f32, moment: Vec3, g: &MixerGains) -> Motors {
    let m = [moment.x, moment.y, moment.z];
    let gains = [g.k_roll, g.k_pitch, g.k_yaw];

    // Per-motor differential (attitude) term.
    let mut diff = [0.0f32; 4];
    for i in 0..4 {
        let mut d = 0.0;
        for axis in 0..3 {
            d += gains[axis] * X_SIGNS[i][axis] * m[axis];
        }
        diff[i] = d;
    }

    // If the differential span alone exceeds the unit range, scale it down so
    // attitude authority fits (loses a little authority but never clips one axis
    // against another).
    let (dmin, dmax) = min_max(&diff);
    let span = dmax - dmin;
    if span > 1.0 {
        let s = 1.0 / span;
        for d in diff.iter_mut() {
            *d *= s;
        }
    }

    // Apply the collective, then shift it so every motor sits within [0,1]
    // (airmode: prioritise the differential over the exact throttle).
    let mut out = [0.0f32; 4];
    for i in 0..4 {
        out[i] = throttle + diff[i];
    }
    let (omin, omax) = min_max(&out);
    let shift = if omax > 1.0 {
        1.0 - omax
    } else if omin < 0.0 {
        -omin
    } else {
        0.0
    };
    for o in out.iter_mut() {
        *o = (*o + shift).clamp(0.0, 1.0);
    }
    Motors(out)
}

fn min_max(v: &[f32; 4]) -> (f32, f32) {
    let mut lo = v[0];
    let mut hi = v[0];
    for &x in &v[1..] {
        if x < lo {
            lo = x;
        }
        if x > hi {
            hi = x;
        }
    }
    (lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_throttle_is_uniform() {
        let out = mix(0.5, Vec3::ZERO, &MixerGains::default()).0;
        for m in out {
            assert!((m - 0.5).abs() < 1e-6, "not uniform: {out:?}");
        }
    }

    #[test]
    fn pure_roll_is_antisymmetric() {
        // +Mx → M1,M2 up; M3,M4 down (sign column [+,+,-,-]).
        let out = mix(0.5, Vec3::new(1.0, 0.0, 0.0), &MixerGains::default()).0;
        assert!(out[0] > 0.5 && out[1] > 0.5);
        assert!(out[2] < 0.5 && out[3] < 0.5);
        // Symmetric about the throttle.
        assert!(((out[0] - 0.5) - (0.5 - out[2])).abs() < 1e-5);
    }

    #[test]
    fn pure_pitch_matches_sign_table() {
        // +My → M2,M3 up; M1,M4 down (sign column [-,+,+,-]).
        let out = mix(0.5, Vec3::new(0.0, 1.0, 0.0), &MixerGains::default()).0;
        assert!(out[1] > 0.5 && out[2] > 0.5);
        assert!(out[0] < 0.5 && out[3] < 0.5);
    }

    #[test]
    fn pure_yaw_matches_sign_table() {
        // +Mz → M1,M3 up; M2,M4 down (sign column [+,-,+,-]).
        let out = mix(0.5, Vec3::new(0.0, 0.0, 1.0), &MixerGains::default()).0;
        assert!(out[0] > 0.5 && out[2] > 0.5);
        assert!(out[1] < 0.5 && out[3] < 0.5);
    }

    #[test]
    fn output_is_always_within_unit_range() {
        let g = MixerGains {
            k_roll: 0.5,
            k_pitch: 0.5,
            k_yaw: 0.5,
            hover_throttle: 0.5,
        };
        // Huge demands on all axes at full throttle.
        let out = mix(1.0, Vec3::new(5.0, -4.0, 3.0), &g).0;
        for m in out {
            assert!((0.0..=1.0).contains(&m), "out of range: {out:?}");
        }
    }

    #[test]
    fn desaturation_preserves_roll_differential_direction() {
        // At full throttle a roll command must still raise the +roll pair above
        // the −roll pair (airmode keeps attitude authority by shifting collective).
        let g = MixerGains {
            k_roll: 0.3,
            ..MixerGains::default()
        };
        let out = mix(1.0, Vec3::new(1.0, 0.0, 0.0), &g).0;
        assert!(out[0] > out[2] && out[1] > out[3], "roll authority lost: {out:?}");
    }
}
