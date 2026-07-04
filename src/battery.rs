//! Battery monitor: pack-voltage sense on the VBAT ADC pin, Betaflight-style
//! automatic cell-count detection, and a voltage-based state-of-charge estimate.
//!
//! No live current sensor is wired yet (the ESC "C" pad ADC is a follow-up), so
//! charge is estimated from pack voltage exactly like Betaflight does when no
//! current sensor is present: detect the series cell count, then map the per-cell
//! voltage linearly between empty and full. Consumed-mAh coulomb counting needs
//! the current sensor and is left for later.

/// ADC reference voltage (V). The H743 ADC measures 0..VREF.
const VREF: f32 = 3.3;
/// Full-scale ADC count at 16-bit resolution.
const ADC_FULL_SCALE: f32 = 65535.0;

/// Pack-voltage divider ratio: **pack volts per volt seen at the ADC pin**. This
/// is the single value to calibrate against a multimeter (the Betaflight
/// `vbat_scale` equivalent). ~11.0 suits the common 10:1 sense divider — raise it
/// if the reported voltage reads low, lower it if it reads high.
pub const VBAT_SCALE: f32 = 11.0;

/// Per-cell voltage thresholds for the state-of-charge estimate (V).
const CELL_FULL_V: f32 = 4.20;
const CELL_EMPTY_V: f32 = 3.30;
/// Divisor for automatic cell-count detection. ~4.3 V/cell so even a freshly
/// charged pack still resolves to the correct series count.
const CELL_DETECT_V: f32 = 4.30;

/// Below this the pack is considered absent (USB-only power / nothing plugged).
const PACK_PRESENT_V: f32 = 1.0;

#[derive(Clone, Copy, Default)]
pub struct Battery {
    /// Pack voltage (V).
    pub volts: f32,
    /// Detected series cell count (`0` = no pack).
    pub cells: u8,
    /// Voltage-based state of charge, 0..100 %.
    pub percent: u8,
}

impl Battery {
    /// Update from a raw ADC sample (0..65535 at 16-bit resolution).
    pub fn update(&mut self, raw: u32) {
        let pin_v = raw as f32 / ADC_FULL_SCALE * VREF;
        self.volts = pin_v * VBAT_SCALE;
        self.cells = detect_cells(self.volts);
        self.percent = percent(self.volts, self.cells);
    }

    /// Pack voltage in millivolts for MAVLink `SYS_STATUS.voltage_battery`
    /// (`u16::MAX` = "no measurement", so the GS shows nothing on USB-only power).
    pub fn millivolts(&self) -> u16 {
        if self.volts < PACK_PRESENT_V {
            u16::MAX
        } else {
            (self.volts * 1000.0).min(65534.0) as u16
        }
    }

    /// Remaining charge for MAVLink `SYS_STATUS.battery_remaining` (`-1` = unknown).
    pub fn remaining_pct(&self) -> i8 {
        if self.cells == 0 {
            -1
        } else {
            self.percent as i8
        }
    }
}

/// Series cell count from pack voltage (`floor(v / 4.3) + 1`), 1..12.
fn detect_cells(volts: f32) -> u8 {
    if volts < PACK_PRESENT_V {
        return 0;
    }
    ((volts / CELL_DETECT_V) as u8 + 1).clamp(1, 12)
}

/// Voltage-based state of charge: per-cell voltage mapped linearly between the
/// empty and full thresholds.
fn percent(volts: f32, cells: u8) -> u8 {
    if cells == 0 {
        return 0;
    }
    let per = volts / cells as f32;
    let pct = (per - CELL_EMPTY_V) / (CELL_FULL_V - CELL_EMPTY_V) * 100.0;
    pct.clamp(0.0, 100.0) as u8
}
