//! Battery monitor: pack-voltage sense on the VBAT ADC pin, Betaflight-style
//! automatic cell-count detection, and a voltage-based state-of-charge estimate.
//!
//! No live current sensor is wired yet (the ESC "C" pad ADC is a follow-up), so
//! charge is estimated from pack voltage exactly like Betaflight does when no
//! current sensor is present: detect the series cell count once when the pack is
//! plugged in, then map the per-cell voltage linearly between empty and full.
//! Consumed-mAh coulomb counting needs the current sensor and is left for later.
//!
//! This mirrors Betaflight's `sensors/battery.c` / `sensors/voltage.c`:
//!   * cell count is **latched at connection** (`autoDetectCellCount`), never
//!     recomputed per sample — recomputing let a noisy reading bump the divisor
//!     and drag the percentage around;
//!   * voltage is run through a first-order low-pass (`displayFiltered`) so the
//!     reading stops fluttering;
//!   * percentage is the per-cell voltage mapped between `vbatmincellvoltage`
//!     (3.30 V) and `vbatmaxcellvoltage` (4.30 V).

/// ADC reference voltage (V). The H743 ADC measures 0..VREF.
const VREF: f32 = 3.3;
/// Full-scale ADC count at 16-bit resolution.
const ADC_FULL_SCALE: f32 = 65535.0;

/// Pack-voltage divider ratio: **pack volts per volt seen at the ADC pin**.
/// Matches the DAKEFPVH743 Betaflight default `vbat_scale = 160` (i.e. 16.0),
/// a 16:1 divider that maps the 3.3 V ADC range onto ~52 V full scale. The
/// ground station also has a live calibration multiplier on top of this.
pub const VBAT_SCALE: f32 = 16.0;

/// Per-cell voltage thresholds for the state-of-charge estimate (V), matching
/// Betaflight's `vbatmincellvoltage` / `vbatmaxcellvoltage` defaults.
const CELL_MAX_V: f32 = 4.30;
const CELL_MIN_V: f32 = 3.30;
/// Divisor for automatic cell-count detection (`floor(v / 4.3) + 1`), the same
/// `vbatmaxcellvoltage` Betaflight uses so a freshly charged pack still resolves
/// to the correct series count.
const CELL_DETECT_V: f32 = CELL_MAX_V;

/// Below this the pack is considered absent (USB-only power / nothing plugged).
const PACK_PRESENT_V: f32 = 1.0;

/// Low-pass smoothing factor for the pack voltage, applied per 5 Hz sample.
/// ~0.2 gives a ~1 s settling time, killing the ADC flutter without feeling
/// laggy (Betaflight's `vbatDisplayLpfPeriod`).
const VBAT_LPF_ALPHA: f32 = 0.2;

#[derive(Clone, Copy, Default)]
pub struct Battery {
    /// Filtered pack voltage (V).
    pub volts: f32,
    /// Detected series cell count (`0` = no pack), latched at connection.
    pub cells: u8,
    /// Voltage-based state of charge, 0..100 %.
    pub percent: u8,
    /// Last averaged raw ADC counts (0..65535) for both candidate VBAT pins,
    /// exposed so the debug line can show which pin actually tracks the pack.
    pub raw_pc0: u32,
    pub raw_pc1: u32,
    /// Low-pass filter state; `0.0` means "not yet seeded" (pack absent).
    filtered: f32,
}

impl Battery {
    /// Update from a raw ADC sample (0..65535 at 16-bit resolution).
    pub fn update(&mut self, raw: u32) {
        let pin_v = raw as f32 / ADC_FULL_SCALE * VREF;
        let inst_v = pin_v * VBAT_SCALE;

        // Pack removed / USB-only power: drop everything so the GS shows nothing
        // and the cell count re-detects cleanly on the next connection.
        if inst_v < PACK_PRESENT_V {
            self.filtered = 0.0;
            self.volts = 0.0;
            self.cells = 0;
            self.percent = 0;
            return;
        }

        // First-order low-pass. Seed straight to the reading on the first sample
        // of a freshly connected pack so cell detection sees the true voltage.
        if self.filtered <= 0.0 {
            self.filtered = inst_v;
        } else {
            self.filtered += VBAT_LPF_ALPHA * (inst_v - self.filtered);
        }
        self.volts = self.filtered;

        // Cell count is latched once, at connection — never recomputed per sample.
        if self.cells == 0 {
            self.cells = detect_cells(self.filtered);
        }
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
/// empty (3.30 V/cell) and full (4.30 V/cell) thresholds, exactly like
/// Betaflight's `calculateBatteryPercentageRemaining` voltage path.
fn percent(volts: f32, cells: u8) -> u8 {
    if cells == 0 {
        return 0;
    }
    let vmin = CELL_MIN_V * cells as f32;
    let vmax = CELL_MAX_V * cells as f32;
    ((volts - vmin) / (vmax - vmin) * 100.0).clamp(0.0, 100.0) as u8
}
