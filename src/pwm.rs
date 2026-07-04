//! Hardware-timer PWM output for the four analog-ESC motor channels.
//!
//! This replaces the former bit-banged DShot path. The motors are now simple
//! analog ESCs that only understand standard servo PWM (a 1000..2000 µs pulse
//! repeated at the refresh rate), so there is no digital frame, no telemetry on
//! the signal wire and no special commands.
//!
//! The four motor pads `M1..M4 = PA0..PA3` are `TIM2_CH1..CH4` (AF1) on the
//! DAKEFPVH743 hwdef, so one general-purpose timer drives all four channels.
//! The timer generates the waveform continuously in hardware — the firmware
//! only rewrites each channel's compare register (its pulse width) whenever the
//! throttle changes. Compared with the old bit-bang this needs no interrupt
//! masking and has no per-bit timing budget to blow.

use embedded_hal::PwmPin;
use stm32h7xx_hal::gpio::{Alternate, PA0, PA1, PA2, PA3};
use stm32h7xx_hal::pac::TIM2;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::pwm::{ComplementaryImpossible, Pwm};
use stm32h7xx_hal::rcc::{rec, CoreClocks};

/// One TIM2 PWM channel in the non-complementary configuration that `.pwm()`
/// yields for TIM2 CH1..CH4.
type MotorCh<const C: u8> = Pwm<TIM2, C, ComplementaryImpossible>;

/// The four AF1 pins that carry the motor signals, in channel order CH1..CH4.
pub type MotorPins = (
    PA0<Alternate<1>>,
    PA1<Alternate<1>>,
    PA2<Alternate<1>>,
    PA3<Alternate<1>>,
);

/// Owns the four TIM2 PWM channels and converts a requested pulse width in
/// microseconds into the timer compare value.
pub struct MotorPwm {
    ch0: MotorCh<0>,
    ch1: MotorCh<1>,
    ch2: MotorCh<2>,
    ch3: MotorCh<3>,
    /// Timer compare value for 100 % duty (ARR+1) — a full PWM period.
    max_duty: u32,
    /// PWM period in microseconds (`1_000_000 / refresh_hz`).
    period_us: u32,
}

impl MotorPwm {
    /// Configure TIM2 CH1..CH4 (PA0..PA3, AF1) as `refresh_hz` PWM and enable all
    /// four channels. Channels start at 0 duty (line held low), so the caller
    /// must write the ESC idle/min pulse on the first output tick before the ESCs
    /// arm. `refresh_hz` is clamped to a sane servo-PWM range (50..490 Hz).
    pub fn new(
        tim2: TIM2,
        pins: MotorPins,
        prec: rec::Tim2,
        refresh_hz: u16,
        clocks: &CoreClocks,
    ) -> Self {
        let hz = refresh_hz.clamp(50, 490);
        let (mut c0, mut c1, mut c2, mut c3) =
            tim2.pwm(pins, (hz as u32).Hz(), prec, clocks);
        let max_duty = c0.get_max_duty();
        // Start with the line low; the ESC controller writes idle on the first tick.
        c0.set_duty(0);
        c1.set_duty(0);
        c2.set_duty(0);
        c3.set_duty(0);
        c0.enable();
        c1.enable();
        c2.enable();
        c3.enable();
        Self {
            ch0: c0,
            ch1: c1,
            ch2: c2,
            ch3: c3,
            max_duty,
            period_us: 1_000_000 / hz as u32,
        }
    }

    /// Compare value for a `us`-microsecond pulse. `u64` intermediate so
    /// `max_duty * us` cannot overflow, clamped to a full-period duty.
    fn duty_for_us(&self, us: u16) -> u32 {
        let d = (self.max_duty as u64 * us as u64) / self.period_us as u64;
        d.min(self.max_duty as u64) as u32
    }

    /// Write one physical channel's pulse width in microseconds. `ch` is 0..3
    /// (PA0..PA3); out-of-range indices are ignored.
    pub fn set_pulse_us(&mut self, ch: usize, us: u16) {
        let d = self.duty_for_us(us);
        match ch {
            0 => self.ch0.set_duty(d),
            1 => self.ch1.set_duty(d),
            2 => self.ch2.set_duty(d),
            3 => self.ch3.set_duty(d),
            _ => {}
        }
    }
}
