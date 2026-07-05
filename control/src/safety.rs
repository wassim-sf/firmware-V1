//! Control-side safety supervisor. Ports `se3quad .../safety.rs` to `f32`.
//!
//! Caps the *commanded* tilt (so an aggressive outer-loop command can't flip the
//! airframe), clamps total thrust, and inhibits torque when disarmed. This is the
//! control-law's own limiter; the firmware [`crate::esc`]-side master interlock and
//! watchdog are a second, independent layer.
//!
//! Tilt is measured from world-up `e3 = (0,0,1)` (Z-up firmware convention); at
//! hover the commanded body-Z axis `b3c = e3`, so the numbers match the reference.

use crate::math::Vec3;

/// Supervisor limits applied around the controller. SI units; angles in radians.
#[derive(Clone, Copy, Debug)]
pub struct Safety {
    pub enabled: bool,
    /// Motors only produce thrust/torque when armed.
    pub armed: bool,
    /// Total-thrust bounds [N].
    pub f_min: f32,
    pub f_max: f32,
    /// Maximum commanded tilt from upright [rad].
    pub max_tilt: f32,
}

impl Safety {
    /// Transparent pass-through (no limits) — the default.
    pub fn off() -> Self {
        Safety {
            enabled: false,
            armed: true,
            f_min: 0.0,
            f_max: f32::INFINITY,
            max_tilt: core::f32::consts::PI,
        }
    }

    /// A cautious preset: positive thrust, a 45° tilt cap.
    pub fn standard(hover_thrust: f32) -> Self {
        Safety {
            enabled: true,
            armed: true,
            f_min: 0.0,
            f_max: 2.5 * hover_thrust,
            max_tilt: core::f32::consts::FRAC_PI_4,
        }
    }

    /// Clamp the commanded desired body-Z axis `b3c` so its tilt from `e3` never
    /// exceeds `max_tilt`. Returns `b3c` unchanged when within the limit or disabled.
    pub fn limit_tilt(&self, b3c: Vec3) -> Vec3 {
        if !self.enabled {
            return b3c;
        }
        let e3 = Vec3::new(0.0, 0.0, 1.0);
        let cos_tilt = b3c.dot(&e3).clamp(-1.0, 1.0);
        let tilt = libm::acosf(cos_tilt);
        if tilt <= self.max_tilt {
            return b3c;
        }
        // Re-aim `b3c` to sit exactly `max_tilt` from `e3`, in the same plane.
        let tangent = b3c - cos_tilt * e3;
        let tn = tangent.norm();
        if tn < 1e-9 {
            return b3c; // exactly anti-parallel: nothing sensible to clamp toward
        }
        let t_hat = tangent * (1.0 / tn);
        (e3 * libm::cosf(self.max_tilt) + t_hat * libm::sinf(self.max_tilt)).normalize()
    }

    /// Clamp total thrust to `[f_min, f_max]`, forcing zero when disarmed.
    pub fn clamp_thrust(&self, f: f32) -> f32 {
        if !self.enabled {
            return f;
        }
        if !self.armed {
            return 0.0;
        }
        f.clamp(self.f_min, self.f_max)
    }

    /// Whether torque is currently inhibited (disarmed while the layer is on).
    pub fn inhibited(&self) -> bool {
        self.enabled && !self.armed
    }
}

impl Default for Safety {
    fn default() -> Self {
        Safety::off()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn off_is_transparent() {
        let s = Safety::off();
        let v = Vec3::new(0.3, 0.4, 0.5).normalize();
        assert_eq!(s.limit_tilt(v), v);
        assert_eq!(s.clamp_thrust(-5.0), -5.0);
        assert!(!s.inhibited());
    }

    #[test]
    fn tilt_is_capped_to_max() {
        let mut s = Safety::standard(42.6);
        s.max_tilt = 30_f32.to_radians();
        // A 60° tilt command gets clamped to 30°.
        let a = 60_f32.to_radians();
        let b3c = Vec3::new(libm::sinf(a), 0.0, libm::cosf(a));
        let out = s.limit_tilt(b3c);
        let tilt = libm::acosf(out.dot(&Vec3::new(0.0, 0.0, 1.0)).clamp(-1.0, 1.0));
        assert!((tilt - s.max_tilt).abs() < 1e-4, "tilt not clamped: {tilt}");
        assert!((out.norm() - 1.0).abs() < 1e-5, "b3c not unit");
    }

    #[test]
    fn tilt_within_limit_untouched() {
        let s = Safety::standard(42.6);
        let a = 10_f32.to_radians();
        let b3c = Vec3::new(libm::sinf(a), 0.0, libm::cosf(a));
        assert!((s.limit_tilt(b3c) - b3c).norm() < 1e-6);
    }

    #[test]
    fn thrust_clamped_and_disarm_zeroes() {
        let mut s = Safety::standard(42.6);
        assert!((s.clamp_thrust(1000.0) - s.f_max).abs() < 1e-3);
        assert_eq!(s.clamp_thrust(-1.0), 0.0);
        s.armed = false;
        assert_eq!(s.clamp_thrust(50.0), 0.0);
        assert!(s.inhibited());
    }
}
