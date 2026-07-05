//! Closed-loop flight control glue.
//!
//! Ties the tested [`scky_control`] geometric controller to the firmware's
//! estimates and outputs: it builds the controller [`QuadState`] from the AHRS +
//! EKF, turns RC sticks into a desired attitude/throttle, runs the SO(3) control
//! law, mixes the wrench into normalised motor commands, and enforces the arming
//! and RC-failsafe interlocks. All heavy maths lives in the library crate (and is
//! host-tested); this module is only wiring + safety gating.
//!
//! Default flight mode is the **manual self-levelling (angle)** inner loop
//! ([`ControlMode::Attitude`]): the pilot commands bank angle + throttle. The
//! outer loops ([`ControlMode::Position`]/[`ControlMode::Velocity`]) are ported and
//! tested in the library; position-hold is enabled here only when
//! [`ControlCfg::enable_pos_hold`] is set and the EKF has converged.

use crate::ahrs::Attitude;
use crate::crsf::RcChannels;
use crate::ekf::NavSolution;
use scky_control::mixer::{self, MixerGains};
use scky_control::rc::RcMap;
use scky_control::{ControlMode, GeometricController, QuadParams, Safety, Target};
use scky_control::frame::state_from_ahrs;

const DEG2RAD: f32 = core::f32::consts::PI / 180.0;

/// Control-loop period [s]. Must match the `control_task` scheduling.
pub const CONTROL_DT: f32 = 0.002; // 500 Hz

/// How long the RC link may go without a fresh frame before failsafe disarms [ms].
const LINK_TIMEOUT_MS: u32 = 500;

/// Static configuration (host-tunable later via a MAVLink message, mirroring the
/// `SCKY_ESC` pattern). Sensible defaults for bring-up.
#[derive(Clone, Copy)]
pub struct ControlCfg {
    pub rc_map: RcMap,
    pub mixer: MixerGains,
    pub params: QuadParams,
    /// Commanded-tilt cap for the outer loop [rad].
    pub max_tilt: f32,
    /// Allow EKF position-hold when a mode switch is high (default off).
    pub enable_pos_hold: bool,
    /// 0-based RC channel selecting position-hold (when `enable_pos_hold`).
    pub ch_mode: usize,
}

impl Default for ControlCfg {
    fn default() -> Self {
        ControlCfg {
            rc_map: RcMap::default(),
            mixer: MixerGains::default(),
            params: QuadParams {
                ts: CONTROL_DT,
                ..QuadParams::default()
            },
            max_tilt: 45_f32 * DEG2RAD,
            enable_pos_hold: false,
            ch_mode: 5,
        }
    }
}

/// The live closed-loop controller + arming state machine.
pub struct Control {
    cfg: ControlCfg,
    ctrl: GeometricController,
    safety: Safety,
    armed: bool,
    /// Heading setpoint (rad) integrated from the yaw stick while armed.
    yaw_sp: f32,
    /// RC link liveness: last frame counter + the time it last advanced.
    last_frames: u32,
    last_frame_ms: u32,
    /// Captured position hold target (N/E/Up) when position-hold is engaged.
    pos_hold: [f32; 3],
    in_pos_hold: bool,
}

impl Control {
    pub fn new() -> Self {
        let cfg = ControlCfg::default();
        let ctrl = GeometricController::new(&cfg.params);
        let safety = Safety {
            max_tilt: cfg.max_tilt,
            ..Safety::standard(cfg.params.hover_thrust())
        };
        Self {
            cfg,
            ctrl,
            safety,
            armed: false,
            yaw_sp: 0.0,
            last_frames: 0,
            last_frame_ms: 0,
            pos_hold: [0.0; 3],
            in_pos_hold: false,
        }
    }

    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// One control step. Returns `(per-logical-motor commands [0,1], armed)` for
    /// [`crate::esc::Esc::set_flight`]. When disarmed it returns idle + `false`.
    pub fn update(
        &mut self,
        att: &Attitude,
        nav: &NavSolution,
        rc: &RcChannels,
        now_ms: u32,
    ) -> ([f32; 4], bool) {
        // Snapshot the 16 channels as microseconds once.
        let mut us = [self.cfg.rc_map.us_mid; 16];
        for (i, u) in us.iter_mut().enumerate() {
            *u = rc.ch_us(i);
        }

        // --- RC link liveness (failsafe). ---
        if rc.frames != self.last_frames {
            self.last_frames = rc.frames;
            self.last_frame_ms = now_ms;
        }
        let link_alive =
            rc.frames != 0 && now_ms.wrapping_sub(self.last_frame_ms) < LINK_TIMEOUT_MS;

        let cmd = self.cfg.rc_map.map(&us, CONTROL_DT, &mut self.yaw_sp);

        // --- Arming state machine. ---
        if self.armed {
            // Disarm on switch low or link loss.
            if !cmd.arm_requested || !link_alive {
                self.disarm();
                return ([0.0; 4], false);
            }
        } else {
            // Arm only with the switch high, a live link, and throttle low.
            if cmd.arm_requested && link_alive && self.cfg.rc_map.throttle_low(&us) {
                self.arm(att);
            } else {
                return ([0.0; 4], false);
            }
        }

        // --- Armed: run the control law. ---
        let state = state_from_ahrs(att.q, att.rates, nav.pos, nav.vel);
        if !state.is_finite() {
            // Estimator failsafe: never feed non-finite state to the motors.
            self.disarm();
            return ([0.0; 4], false);
        }

        // Mode select: manual angle by default; position-hold only when enabled,
        // switched on, and the EKF has converged.
        let pos_hold_req = self.cfg.enable_pos_hold
            && nav.converged
            && us[self.cfg.ch_mode] >= self.cfg.rc_map.arm_us;

        let (moment, collective) = if pos_hold_req {
            if !self.in_pos_hold {
                self.pos_hold = nav.pos; // capture the loiter point on engage
                self.in_pos_hold = true;
            }
            let target = Target::hold(
                scky_control::Vec3::new(self.pos_hold[0], self.pos_hold[1], self.pos_hold[2]),
                scky_control::trajectory::yaw_to_b1d(self.yaw_sp),
            );
            let out = self.ctrl.control(
                &state,
                &target,
                &self.cfg.params,
                ControlMode::Position,
                &self.safety,
                None,
            );
            let coll = (out.f / self.cfg.params.hover_thrust()) * self.cfg.mixer.hover_throttle;
            (out.moment, coll.clamp(0.0, 1.0))
        } else {
            self.in_pos_hold = false;
            // Manual angle mode: attitude from sticks, collective from the throttle
            // stick directly (the controller's thrust is unused on this path).
            let out = self.ctrl.control(
                &state,
                &cmd.target,
                &self.cfg.params,
                ControlMode::Attitude,
                &self.safety,
                None,
            );
            (out.moment, cmd.throttle)
        };

        let motors = mixer::mix(collective, moment, &self.cfg.mixer);
        (motors.as_array(), true)
    }

    fn arm(&mut self, att: &Attitude) {
        self.armed = true;
        self.in_pos_hold = false;
        // Start the heading setpoint at the current heading so there is no yaw
        // jump on arm, and clear the controller's integrator/derivative filters.
        self.yaw_sp = att.yaw * DEG2RAD;
        self.ctrl.reset(&self.cfg.params);
        self.safety.armed = true;
    }

    fn disarm(&mut self) {
        self.armed = false;
        self.in_pos_hold = false;
        self.safety.armed = false;
    }
}

impl Default for Control {
    fn default() -> Self {
        Self::new()
    }
}
