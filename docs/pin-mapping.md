# DAKEFPV H743 — pin & pad mapping

Board: **DAKEFPVH743** (STM32H743xx). This is the wiring reference for the
`scky-fc` firmware — **the source of truth is `src/main.rs` `init()`**; this file
just makes it readable so you don't have to re-derive it.

- **STM32 pin** column is authoritative (taken straight from the firmware init).
- **Board pad** column is the DAKEFPVH743 silkscreen / hwdef label. Ones the
  firmware names explicitly (M1–M4, T/R pads, GPS UART, I2C) are solid; where a
  physical pad label is uncertain it is marked _(verify on board)_.
- Core clock is the internal **64 MHz HSI** (no external crystal needed).

---

## Motor outputs (analog PWM ESCs)

Standard servo PWM, 1000–2000 µs, 50 Hz carrier. Hardware **TIM2**, driver in
`src/pwm.rs`. Logical motor → physical output is remappable in software
(`output_map`), so you never have to move wires to reorder motors.

| Motor | STM32 pin | Timer ch | Board pad |
|-------|-----------|----------|-----------|
| M1    | **PA0**   | TIM2_CH1 | `M1`      |
| M2    | **PA1**   | TIM2_CH2 | `M2`      |
| M3    | **PA2**   | TIM2_CH3 | `M3`      |
| M4    | **PA3**   | TIM2_CH4 | `M4`      |

---

## Serial ports (UARTs)

Wire the peripheral's **TX → the FC's RX pin** and vice-versa.

| Function | STM32 UART | FC TX pin | FC RX pin | Baud | Board pad / notes |
|----------|-----------|-----------|-----------|------|-------------------|
| **GPS** (uBlox NEO-M8N, NMEA) | USART1 | **PA9 = `T1`** | **PA10 = `R1`** | 9600 | ArduPilot **SERIAL1**. Wire **GPS TX → `R1`** (FC RX/PA10) and **GPS RX → `T1`** (FC TX/PA9). Swapping these is the #1 cause of a silent GPS. |
| MTF-01 optical flow + lidar | USART2 | **PD5** | **PD6** | 115200 | flow/lidar UART |
| ESC telemetry (BLHeli/KISS) — **unused with analog ESCs** | USART3 | PD8 | **PD9** | 115200 | **`T` pad** (ESC telem, SERIAL3) |
| ExpressLRS RX (CRSF) | UART5 | **PB6** | **PB5** | 420000 | RC / ELRS UART |
| TF-Luna LEFT side lidar | USART6 | PC7 (`R6`) | **PC6 (`T6`)** | 115200 | RX/TX **swapped** in firmware — lidar TX lands on `T6` |
| TF-Luna RIGHT side lidar | UART7 | **PE8** | **PE7** | 115200 | side lidar UART |

> The ESC-telemetry UART (USART3 / `T` pad) is left configured but **inert** —
> analog ESCs have no telemetry wire. It does no harm; ignore it.

---

## I2C — compass + barometer (shared bus)

**I2C2**, one RTIC task owns the bus (`i2c_task`). The external magnetometer and
the on-board SPL06 baro both hang off it.

| Signal | STM32 pin | Board pad |
|--------|-----------|-----------|
| **SCL** | **PB10** | `SCL` |
| **SDA** | **PB11** | `SDA` |

- **Magnetometer**: driver auto-probes **QMC5883L @ 0x0D** and **HMC5883L @ 0x1E**.
  "NEO-M8N + HMC5883" modules are almost always QMC5883L clones — handled.
- **Barometer**: SPL06 / SPL06-001, same bus.
- Mount **orientation** and magnetic **declination** are constants near the top of
  `src/main.rs` (`MAG_ROTATION`, `MAG_DECLINATION_DEG`) — set them for your build.

---

## IMUs (SPI, on-board — do not rewire)

| IMU | Bus | SCK | MISO | MOSI | CS |
|-----|-----|-----|------|------|-----|
| IMU1 (InvenSense v3) | SPI1 | PA5 | PA6 | PA7 | **PA4** |
| IMU2 (InvenSense v3) | SPI4 | PE12 | PE13 | PE14 | **PB1** |

---

## USB

| Function | STM32 pins | Notes |
|----------|-----------|-------|
| USB-C (CDC-ACM serial, MAVLink to ground station) | **PA11 / PA12** | OTG2_FS internal full-speed PHY (HAL `USB2`) |

---

## Battery / ADC

| Signal | STM32 pin | ADC | Notes |
|--------|-----------|-----|-------|
| **VBAT** pack voltage | **PC0** | ADC1_INP10 | Read at 5 Hz in `battery_task`; scale = `battery::VBAT_SCALE` (default 11.0, calibrate vs a multimeter). Cell count + charge % are derived Betaflight-style. **If voltage reads 0/garbage, VBAT is on a different pad — change `PC0` in `main.rs` (and the `VbatPin` alias).** |

Firmware fills `SYS_STATUS` with voltage + charge %, which the platform already
renders (voltage, cell count, %). Per-cell = pack ÷ detected cells.

## Not yet wired in firmware

- **`C` pad — battery current sense.** `cur_scale` / `cur_offset` exist in the ESC
  config, but no current-sense ADC channel is read yet, so pack **current** and
  consumed **mAh** are not live (charge % is voltage-based). A follow-up can add a
  second ADC channel on the `C` pad.

---

## Quick reference — what's free for expansion

Ports already claimed: SPI1, SPI4, I2C2, USART1/2/3/6, UART5/7, TIM2 (PA0–3),
OTG2 (PA11/12). Anything else on the H743 (e.g. other timers, I2C1, SPI2/3,
remaining UARTs) is unused by this firmware and available.
