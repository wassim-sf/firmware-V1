//! ESC manager for **analog / PWM ESCs**: per-motor throttle calibration, motor
//! ordering, a throttle-range teach routine, and the master safety interlock
//! that sits between commands and the [`crate::pwm`] output.
//!
//! The motors are simple analog ESCs that only understand standard servo PWM (a
//! 1000..2000 µs pulse). There is no digital frame, no signal-wire telemetry and
//! no special commands (direction / 3D / beacon) — spin direction is fixed by
//! wiring, so it is handled here only by **remapping which physical output drives
//! which logical motor**, never by a software reverse.
//!
//! There is no closed-loop flight control yet, so the **master enable** switch is
//! the arm for spinning output. A standard MAVLink motor test may also
//! temporarily enable the output path so bench testing does not depend on the
//! custom ESC-config message landing first.
//!
//! # Calibration model (equal command → equal speed)
//!
//! Each motor carries its own `min_us` / `max_us` throttle endpoints. A throttle
//! fraction `t ∈ [0,1]` maps to `min_us + t·(max_us − min_us)` for *that* motor,
//! so trimming a motor's endpoints makes a shared throttle drive every motor at
//! the same speed. A one-shot **range-teach routine** drives every channel to
//! `max_us` then `min_us` so the ESCs themselves learn their throttle range.

use crate::esc_telem::TelemFrame;

/// Number of motors / ESCs (also the number of PWM output channels).
pub const N_MOTORS: usize = 4;

/// Default motor magnetic pole count for the (now inert) BLHeli telemetry path.
/// Analog ESCs send no telemetry, but the decoder still needs a pole count.
pub const DEFAULT_POLE_COUNT: u8 = 14;

/// Default / minimum servo-PWM endpoints, microseconds.
const MIN_US_DEFAULT: u16 = 1000;
const MAX_US_DEFAULT: u16 = 2000;
/// Hard clamp so a bad host value can never command a wild pulse.
const PULSE_MIN: u16 = 800;
const PULSE_MAX: u16 = 2200;

/// How long (ms) a motor test runs if the host does not refresh it. Also the
/// watchdog horizon: if the host stops talking, motors stop within this window.
pub const DEFAULT_TEST_TIMEOUT_MS: u32 = 3000;

/// Flight-control watchdog (ms): if the closed-loop `control_task` stops refreshing
/// the per-motor commands (a hung task), the motors revert to idle and disarm
/// within this window. Must comfortably exceed the control-loop period.
pub const FLIGHT_TIMEOUT_MS: u32 = 100;

/// Range-teach routine phase durations (ms): full throttle held, then idle held.
/// Safety timeout (ms): if the host leaves the ESCs held at a calibration
/// endpoint (e.g. it disconnects mid-calibration), auto-return to idle.
const CAL_SAFETY_MS: u32 = 30_000;

/// Minimum span (µs) enforced between a motor's min and max endpoints.
const MIN_SPAN_US: u16 = 50;

/// Max pulse increase per output tick (µs). Throttle *up* is ramped so a
/// free-spinning bench motor eases in rather than kicking; throttle *down*
/// (including stop) is instant.
const RAMP_STEP_US: u16 = 20;

/// `EscCmd.command` action codes (analog ESCs take no DShot special commands).
/// Calibration is a manual two-step hold so the operator controls the timing:
/// hold MAX, connect the battery (ESC records full throttle), then set MIN.
pub const CMD_CAL_MAX: u16 = 1;
pub const CMD_CAL_MIN: u16 = 2;
pub const CMD_STOP_ALL: u16 = 3;

/// Latest decoded ESC telemetry, indexed by motor. Kept for the BLHeli/KISS
/// telemetry UART path, which is inert with analog ESCs (no telemetry wire) but
/// still populates the analog current sense aggregate via the ADC in `main`.
#[derive(Clone, Copy)]
pub struct EscTelemetry {
    pub rpm: [i32; N_MOTORS],
    pub centivolt: [u16; N_MOTORS],
    pub centiamp: [u16; N_MOTORS],
    pub temp: [u8; N_MOTORS],
    pub err: [u8; N_MOTORS],
    /// Consumption (mAh) of the most recently reported ESC.
    pub mah: u16,
    /// Round-robin slot the next decoded record is attributed to.
    rr: usize,
}

impl EscTelemetry {
    pub const fn new() -> Self {
        Self {
            rpm: [0; N_MOTORS],
            centivolt: [0; N_MOTORS],
            centiamp: [0; N_MOTORS],
            temp: [0; N_MOTORS],
            err: [0; N_MOTORS],
            mah: 0,
            rr: 0,
        }
    }

    /// Store a decoded telemetry record into the next motor slot.
    pub fn ingest(&mut self, f: TelemFrame, pole_count: u8) {
        let i = self.rr % N_MOTORS;
        self.rpm[i] = f.rpm(pole_count);
        self.centivolt[i] = f.centivolt;
        self.centiamp[i] = f.centiamp;
        self.temp[i] = f.temp_c;
        self.mah = f.mah;
        self.rr = (self.rr + 1) % N_MOTORS;
    }

    /// Sum of per-motor current in amps (aggregate pack current).
    pub fn total_current_a(&self) -> f32 {
        self.centiamp.iter().map(|&c| c as f32).sum::<f32>() / 100.0
    }
}

/// Live, host-tunable ESC configuration. Mirrors `SCKY_ESC_CONFIG` /
/// `SCKY_ESC_SET` on the wire.
#[derive(Clone, Copy)]
pub struct EscConfig {
    /// Master output enable. **Defaults to `false`** — no motor spins until the
    /// ground station explicitly turns it on. Idle (`min_us`) is still emitted so
    /// the ESCs stay armed-idle.
    pub master_enabled: bool,
    /// PWM carrier frequency (Hz). Applied to the timer at init; runtime changes
    /// are reflected to the GS but only take effect on the next boot.
    pub pwm_hz: u16,
    /// Per-motor throttle endpoints (µs), indexed by **logical** motor.
    pub min_us: [u16; N_MOTORS],
    pub max_us: [u16; N_MOTORS],
    /// Motor order remap: logical motor `i` drives physical output
    /// `output_map[i]` (0..3 = PA0..PA3). Default identity `[0,1,2,3]`.
    pub output_map: [u8; N_MOTORS],
    /// Analog current-sense calibration (C pad): scale and offset (mV).
    pub cur_scale: f32,
    pub cur_offset: f32,
}

impl EscConfig {
    pub const fn new() -> Self {
        Self {
            master_enabled: false,
            pwm_hz: 50,
            min_us: [MIN_US_DEFAULT; N_MOTORS],
            max_us: [MAX_US_DEFAULT; N_MOTORS],
            output_map: [0, 1, 2, 3],
            cur_scale: 490.0, // SpeedyBee BL32 50A default
            cur_offset: 0.0,
        }
    }
}

impl Default for EscConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Independent output state for one logical motor.
#[derive(Clone, Copy)]
struct MotorOut {
    /// Target throttle fraction 0.0..1.0 (0 = idle). Set by a motor test.
    target: f32,
    /// Monotonic deadline (ms) after which a non-zero target expires to idle.
    until_ms: u32,
    /// Pulse (µs) actually applied, slewed toward the target pulse each tick.
    applied_us: u16,
}

impl MotorOut {
    const fn new() -> Self {
        Self { target: 0.0, until_ms: 0, applied_us: MIN_US_DEFAULT }
    }

    fn reset(&mut self) {
        self.target = 0.0;
        self.until_ms = 0;
        self.applied_us = MIN_US_DEFAULT;
    }
}

const INIT_MOTOR: MotorOut = MotorOut::new();

/// Closed-loop flight command: per-**logical**-motor throttle fractions from the
/// mixer, an armed interlock, and the timestamp of the last refresh (for the
/// [`FLIGHT_TIMEOUT_MS`] watchdog). This is the path the geometric controller
/// drives; it takes precedence over the bench motor-test path when armed & fresh.
#[derive(Clone, Copy)]
struct FlightInput {
    cmds: [f32; N_MOTORS],
    armed: bool,
    stamp_ms: u32,
}

impl FlightInput {
    const fn new() -> Self {
        Self {
            cmds: [0.0; N_MOTORS],
            armed: false,
            stamp_ms: 0,
        }
    }
}

/// Throttle-range teach routine state. Each hold persists until the operator
/// moves to the next step or the [`CAL_SAFETY_MS`] deadline elapses.
#[derive(Clone, Copy, PartialEq, Eq)]
enum CalPhase {
    Idle,
    /// Hold every channel at `max_us` (operator powers the ESCs now).
    Max,
    /// Hold every channel at `min_us` (ESCs record the low endpoint).
    Min,
}

/// ESC controller state owned by the firmware and mutated by inbound commands.
pub struct Esc {
    pub config: EscConfig,
    motors: [MotorOut; N_MOTORS],
    cal_phase: CalPhase,
    cal_until_ms: u32,
    /// Closed-loop flight command (see [`FlightInput`]).
    flight: FlightInput,
}

/// Slew the currently applied pulse toward `target_us`. Increases are capped at
/// [`RAMP_STEP_US`]; decreases (incl. idle) are instant — reducing throttle never
/// needs easing.
fn slew(applied_us: u16, target_us: u16) -> u16 {
    if target_us <= applied_us {
        target_us
    } else {
        (applied_us + RAMP_STEP_US).min(target_us)
    }
}

impl Esc {
    pub const fn new() -> Self {
        Self {
            config: EscConfig::new(),
            motors: [INIT_MOTOR; N_MOTORS],
            cal_phase: CalPhase::Idle,
            cal_until_ms: 0,
            flight: FlightInput::new(),
        }
    }

    /// Publish one closed-loop control step: per-logical-motor throttle fractions
    /// (`[0,1]`, mixer output) plus the armed interlock. When `armed` is true and
    /// the input stays fresh (within [`FLIGHT_TIMEOUT_MS`]), [`Self::pulses`] flies
    /// these commands, overriding the bench motor-test path. Disarming (or a stale
    /// input) drops straight back to idle. Called at the control-loop rate.
    pub fn set_flight(&mut self, cmds: [f32; N_MOTORS], armed: bool, now_ms: u32) {
        for i in 0..N_MOTORS {
            self.flight.cmds[i] = cmds[i].clamp(0.0, 1.0);
        }
        self.flight.armed = armed;
        self.flight.stamp_ms = now_ms;
    }

    /// Whether the closed loop currently holds the output (armed + fresh input).
    pub fn flight_active(&self, now_ms: u32) -> bool {
        self.flight.armed && now_ms.wrapping_sub(self.flight.stamp_ms) < FLIGHT_TIMEOUT_MS
    }

    /// Pulse (µs) for a logical motor at throttle fraction `t`, using that
    /// motor's own endpoints and clamped to the safe range.
    fn pulse_for(&self, i: usize, t: f32) -> u16 {
        let min = self.config.min_us[i];
        let max = self.config.max_us[i];
        let span = max.max(min) - min.min(max);
        let lo = min.min(max);
        let t = t.clamp(0.0, 1.0);
        let us = lo as f32 + t * span as f32;
        (us as u16).clamp(PULSE_MIN, PULSE_MAX)
    }

    /// Apply a `SCKY_ESC_SET` config write. Disabling the master immediately
    /// cancels all motor activity (idle is emitted next tick).
    #[allow(clippy::too_many_arguments)]
    pub fn apply_set(
        &mut self,
        master_enabled: bool,
        pwm_hz: u16,
        min_us: [u16; N_MOTORS],
        max_us: [u16; N_MOTORS],
        output_map: [u8; N_MOTORS],
        cur_scale: f32,
        cur_offset: f32,
    ) {
        self.config.master_enabled = master_enabled;
        self.config.pwm_hz = pwm_hz.clamp(50, 490);
        for i in 0..N_MOTORS {
            let mut lo = min_us[i].clamp(PULSE_MIN, PULSE_MAX);
            let hi = max_us[i].clamp(PULSE_MIN, PULSE_MAX);
            // Guard against min crossing max (the UI clamps too): keep a min span.
            if lo + MIN_SPAN_US > hi {
                lo = hi.saturating_sub(MIN_SPAN_US);
            }
            self.config.min_us[i] = lo;
            self.config.max_us[i] = hi;
            // Only accept a valid physical channel; otherwise keep identity.
            self.config.output_map[i] = if (output_map[i] as usize) < N_MOTORS {
                output_map[i]
            } else {
                i as u8
            };
        }
        self.config.cur_scale = cur_scale;
        self.config.cur_offset = cur_offset;
        if !master_enabled {
            self.cal_phase = CalPhase::Idle;
            for m in self.motors.iter_mut() {
                m.reset();
            }
        }
    }

    /// Start a timed motor test from a `MAV_CMD_DO_MOTOR_TEST`. `motor` is 1-based
    /// (1..4); `throttle_pct` is 0..100; `timeout_ms` 0 uses the default. A valid
    /// spin temporarily enables the master path so standard GCS motor-test tools
    /// work even if the custom `SCKY_ESC_SET` master toggle was not sent. A zero
    /// throttle is a per-motor stop and never arms the master.
    pub fn start_test(&mut self, motor: u8, throttle_pct: f32, timeout_ms: u32, now_ms: u32) -> bool {
        // Be tolerant of host conventions: some tools send motor 0 for the first.
        let motor = if motor == 0 { 1 } else { motor };
        if motor as usize > N_MOTORS {
            return false;
        }
        let idx = motor as usize - 1;
        // Some frontends encode 10 % as 0.10 instead of 10.0. Accept both.
        let throttle_pct = if throttle_pct > 0.0 && throttle_pct <= 1.0 {
            throttle_pct * 100.0
        } else {
            throttle_pct
        };
        let frac = throttle_pct.clamp(0.0, 100.0) / 100.0;
        // A zero or tiny timeout reads as "snaps back to zero" in the UI, so give
        // bench motor tests a useful minimum window.
        let timeout = if timeout_ms < 500 {
            DEFAULT_TEST_TIMEOUT_MS
        } else {
            timeout_ms
        };
        if frac > 0.0 {
            self.config.master_enabled = true;
            self.cal_phase = CalPhase::Idle; // a test overrides an in-progress cal
        }
        let m = &mut self.motors[idx];
        m.target = frac;
        if frac > 0.0 {
            m.until_ms = now_ms.wrapping_add(timeout);
        }
        true
    }

    /// Stop all motors immediately (clears every per-motor throttle target).
    pub fn stop_all(&mut self) {
        self.cal_phase = CalPhase::Idle;
        for m in self.motors.iter_mut() {
            m.target = 0.0;
        }
    }

    /// Calibration step 1: hold full throttle (`max_us`) on every channel so the
    /// operator can power the ESCs and have them record the high endpoint. Held
    /// until [`Self::cal_hold_min`] / stop, or the safety timeout. Arms the master
    /// and cancels any motor test.
    pub fn cal_hold_max(&mut self, now_ms: u32) {
        self.config.master_enabled = true;
        for m in self.motors.iter_mut() {
            m.target = 0.0;
        }
        self.cal_phase = CalPhase::Max;
        self.cal_until_ms = now_ms.wrapping_add(CAL_SAFETY_MS);
    }

    /// Calibration step 2: hold idle (`min_us`) on every channel so the ESCs
    /// record the low endpoint and finish learning their range.
    pub fn cal_hold_min(&mut self, now_ms: u32) {
        self.config.master_enabled = true;
        self.cal_phase = CalPhase::Min;
        self.cal_until_ms = now_ms.wrapping_add(CAL_SAFETY_MS);
    }

    /// Abort calibration, returning to idle.
    pub fn stop_calibration(&mut self) {
        self.cal_phase = CalPhase::Idle;
    }

    /// Calibration state for diagnostics: `Some(true)` = holding MAX, `Some(false)`
    /// = holding MIN, `None` = not calibrating.
    pub fn cal_status(&self) -> Option<bool> {
        match self.cal_phase {
            CalPhase::Idle => None,
            CalPhase::Max => Some(true),
            CalPhase::Min => Some(false),
        }
    }

    /// Snapshot of the first actively-spinning motor for diagnostics:
    /// `(motor_1_based, pulse_us, remaining_ms)`.
    pub fn active_test(&self, now_ms: u32) -> Option<(u8, u16, u32)> {
        for (i, m) in self.motors.iter().enumerate() {
            if m.target > 0.0 {
                return Some((i as u8 + 1, m.applied_us, m.until_ms.wrapping_sub(now_ms)));
            }
        }
        None
    }

    /// Compute the PWM pulse (µs) for each **physical** output channel this tick.
    ///
    /// Order of precedence: master interlock (disabled → every motor idle at its
    /// `min_us`); then the range-teach routine (all channels max/min); then per
    /// motor an expired test reverts to idle and the pulse is slewed toward the
    /// throttle target. Logical motor `i` is finally placed on physical channel
    /// `output_map[i]`.
    pub fn pulses(&mut self, now_ms: u32) -> [u16; N_MOTORS] {
        // Safety: a calibration hold left running past the deadline returns to idle.
        if self.cal_phase != CalPhase::Idle
            && now_ms.wrapping_sub(self.cal_until_ms) < u32::MAX / 2
        {
            self.cal_phase = CalPhase::Idle;
        }

        // Range-teach: drive every physical channel to the STANDARD full/idle
        // endpoint (1000/2000 µs), bypassing per-motor trim and the output remap.
        // The ESC must see true full throttle at power-up to enter calibration, and
        // every ESC should learn the same range, so trimmed `max_us` must not apply
        // here. `cal_hold_*` already forced the master on.
        match self.cal_phase {
            CalPhase::Max => return [MAX_US_DEFAULT; N_MOTORS],
            CalPhase::Min => return [MIN_US_DEFAULT; N_MOTORS],
            CalPhase::Idle => {}
        }

        // Closed-loop flight path takes precedence when armed & fresh: fly the
        // mixer's per-logical-motor commands (ramped) and remap to physical
        // channels. Its own `armed` flag is the interlock, independent of the GS
        // master toggle. A stale input while still flagged armed trips the
        // watchdog and drops back to idle.
        if self.flight.armed {
            if now_ms.wrapping_sub(self.flight.stamp_ms) < FLIGHT_TIMEOUT_MS {
                let mut logical = [MIN_US_DEFAULT; N_MOTORS];
                for i in 0..N_MOTORS {
                    let target_us = self.pulse_for(i, self.flight.cmds[i]);
                    let next = slew(self.motors[i].applied_us, target_us);
                    self.motors[i].applied_us = next;
                    logical[i] = next;
                }
                return self.remap(logical);
            }
            // Watchdog: the control task went silent — disarm the flight path.
            self.flight.armed = false;
        }

        // Compute a pulse per logical motor, then remap to physical channels.
        let mut logical = [MIN_US_DEFAULT; N_MOTORS];
        for i in 0..N_MOTORS {
            logical[i] = if !self.config.master_enabled {
                self.motors[i].reset();
                self.config.min_us[i]
            } else {
                // Watchdog: a spin the host stopped refreshing expires to idle.
                if self.motors[i].target > 0.0
                    && now_ms.wrapping_sub(self.motors[i].until_ms) < u32::MAX / 2
                {
                    self.motors[i].target = 0.0;
                }
                let target_us = self.pulse_for(i, self.motors[i].target);
                let next = slew(self.motors[i].applied_us, target_us);
                self.motors[i].applied_us = next;
                next
            };
        }

        self.remap(logical)
    }

    /// Remap logical motors to physical channels: logical motor `i` drives
    /// physical output `output_map[i]`. Channels with no logical source idle.
    fn remap(&self, logical: [u16; N_MOTORS]) -> [u16; N_MOTORS] {
        let mut out = [MIN_US_DEFAULT; N_MOTORS];
        for i in 0..N_MOTORS {
            let ch = self.config.output_map[i] as usize;
            if ch < N_MOTORS {
                out[ch] = logical[i];
            }
        }
        out
    }
}
