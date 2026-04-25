# Maritime Active Gimbal Platform

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python)
![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Jetson-green?logo=nvidia)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Status](https://img.shields.io/badge/Status-Active%20Development-orange)
![University](https://img.shields.io/badge/University-Florida%20Atlantic%20University-blue)

**2-DOF active stabilization gimbal that keeps a drone landing platform level on an unmanned surface vessel (USV) in open-water sea states.**

---

## Interactive Control Diagram

[**View Full Control System Diagram →**](https://htmlpreview.github.io/?https://raw.githubusercontent.com/click-b8/Maritime-Active-Gimbal-Platform/main/docs/Gimbal_control_diagram%20(1).html)

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Setup](#hardware-setup)
3. [Software Architecture](#software-architecture)
4. [Quick Start](#quick-start)
5. [Gain Tuning](#gain-tuning)
6. [Data & Results](#data--results)
7. [Repo Structure](#repo-structure)
8. [Communication Protocol](#communication-protocol)
9. [Team & Acknowledgments](#team--acknowledgments)
10. [License](#license)

---

## System Overview

Autonomous drone delivery and retrieval at sea requires a stable landing zone. Wave-induced vessel motion introduces continuous roll and pitch disturbances that exceed safe drone landing tolerances. This platform solves that problem.

A two-axis gimbal driven by direct-drive torque motors actively counteracts vessel motion in real time, maintaining the landing deck within **±2° of horizontal** across **±15° sea-state disturbances**.

### Key Specs

| Parameter | Value |
|---|---|
| Axes | Roll + Pitch (2-DOF) |
| Control loop rate | 200 Hz |
| Disturbance rejection | ±15° sea-state |
| Platform accuracy | < 2° RMS |
| Actuators | CubeMars AK60-39 V3.0 (×2) |
| IMU | BNO08x MEMS (×2) |
| Compute | NVIDIA Jetson (Ubuntu) |
| Supply voltage | 22 V |
| Control law | FF + PID (feedforward from hull IMU + PID from platform IMU) |

### Control Law

```
erpm = SIGN × (Kp × imu1_angle + Ki × ∫imu1_angle + Kd × imu1_rate + Kff × imu2_rate)
```

- **P term** — reacts to current platform tilt (IMU1)
- **I term** — eliminates steady-state bias
- **D term** — damps platform velocity (IMU1 gyro), second-order EMA filtered
- **FF term** — anticipates hull disturbance before it reaches the platform (IMU2 gyro)

---

## Hardware Setup

### Component List

| Component | Part | Qty |
|---|---|---|
| Compute | NVIDIA Jetson (Ubuntu) | 1 |
| Platform IMU | BNO08x (I²C) | 1 |
| Hull IMU | BNO08x (I²C) | 1 |
| Roll actuator | CubeMars AK60-39 V3.0 | 1 |
| Pitch actuator | CubeMars AK60-39 V3.0 | 1 |
| USB-UART adapters | CH340 / FTDI | 2 |
| Power supply | 22 V regulated | 1 |

### Wiring & Port Map

| Signal | Device | Connection |
|---|---|---|
| Roll motor | CubeMars AK60-39 | `/dev/ttyROLL` (USB-UART symlink) |
| Pitch motor | CubeMars AK60-39 | `/dev/ttyPITCH` (USB-UART symlink) |
| Platform IMU (IMU1) | BNO08x | I²C bus 7 |
| Hull IMU (IMU2) | BNO08x | I²C bus 1 |

### Motor Direction Signs

| Axis | Sign constant | Notes |
|---|---|---|
| Roll | `SIGN_ROLL = -1.0` | Positive platform roll → negative ERPM |
| Pitch | `SIGN_PITCH = +1.0` | Positive platform pitch → positive ERPM |

Flip the sign constant in `main_fpd_tune_headless.py` if a motor runs backwards after mechanical changes.

### Port Symlinks (udev rules on Jetson)

```bash
# /etc/udev/rules.d/99-motors.rules
SUBSYSTEM=="tty", ATTRS{serial}=="<ROLL_SERIAL>",  SYMLINK+="ttyROLL"
SUBSYSTEM=="tty", ATTRS{serial}=="<PITCH_SERIAL>", SYMLINK+="ttyPITCH"
```

### SSH Access

```bash
ssh edg5@172.20.10.10
# Password on Jetson keychain
```

### UART Settings

Baud: `921600` | Format: `8N1` | Protocol: CubeMars AK series (NOT VESC)

---

## Software Architecture

All runnable scripts live in `firmware/`. Install dependencies with `pip install -r requirements.txt`.

### Primary Operation

| Script | Purpose |
|---|---|
| `main_fpd_tune_headless.py` | **PRIMARY.** Headless 200 Hz FF+PID loop. SSH-safe (no display). Logs CSV + auto-generates analysis PNG on stop. Live gain reload from `gains.json` every 0.5 s without restart. |
| `main_fpd_tune.py` | Same as above with optional live scrolling plot (`LIVE_PLOT = True`). Requires display or X11 forwarding. |
| `main_fpd_control.py` | Fixed-gain FPD controller. No live gain reload. Use when gains are finalized and you want a locked-in run. |

### Controller Variants (Development / Comparison)

| Script | Control Mode | Use Case |
|---|---|---|
| `main_ff_control.py` | Feedforward only | Isolate FF contribution; baseline test |
| `main_pd_control.py` | PD only, no FF | Classic PD — sanity check, wave-tank baseline |
| `main_pdf_control.py` | PDF variant | Derivative on measurement, no FF |
| `main_vel_control.py` | Velocity loop | Motor velocity tracking tests |
| `main_pos_control.py` | Position loop | Motor position tracking tests |
| `main_rpm_stabilizer.py` | RPM stabilization | Motor RPM hold under load |
| `main_dual_imu.py` | Dual IMU dev | IMU alignment, offset debug, raw data inspection |
| `main.py` | Single-axis entry | Minimal single-axis PD — development reference |

### Hardware Abstraction

| Script | Purpose |
|---|---|
| `imu_sensor.py` | BNO08x HAL. I²C read, rotation vector + gyro, fallback to single-IMU on bus failure. |
| `serial_motor_driver.py` | CubeMars UART HAL. Frame build/parse, CRC16, ERPM/torque/position commands, telemetry parse. |
| `pd_controller.py` | Reusable PD controller class with derivative filter. Imported by several main scripts. |

### Utilities

| Script | Purpose |
|---|---|
| `motor_test.py` | Pre-run motor utility. Recover from fault state, run ERPM sweep, verify direction. Run this if a motor is unresponsive. |
| `system_check_uart.py` | Pre-flight diagnostics. Checks both UART ports, IMU buses, baud rate. Run before every session. |
| `calibration.py` | IMU calibration routine. Runs BNO08x calibration and reports stability status. |
| `auto_tune.py` | Bayesian gain optimizer (Optuna/TPE). Seeds from historical CSV logs, scores runs, suggests next `gains.json`. |
| `parse_fpd_run.py` | Post-run CSV parser. Converts a `fpd_tune_data_*.csv` into a JSON summary with per-axis stats and performance score. |

---

## Quick Start

New to the system? Follow these steps in order.

### Step 1 — SSH into the Jetson

```bash
ssh edg5@172.20.10.10
cd ~/captone_OG
```

### Step 2 — Pre-flight check

```bash
python3 firmware/system_check_uart.py
```

Verifies both motor UART ports and both IMU buses. Fix any reported failures before proceeding.

### Step 3 — Recover motors (if needed)

```bash
python3 firmware/motor_test.py
```

Run this if either motor is unresponsive or in a fault state. Clears faults and sweeps motors to confirm direction.

### Step 4 — Start the control loop

```bash
python3 firmware/main_fpd_tune_headless.py
```

The script will:
- Connect to both motors and IMUs
- Create `gains.json` with defaults if it doesn't exist
- Open a timestamped CSV log (`fpd_tune_data_YYYYMMDD_HHMMSS.csv`)
- Enter the 200 Hz FF+PID control loop
- Print live telemetry to terminal (`Roll`, `Pitch`, ERPM components, amps, dt)

### Step 5 — Live gain tuning (second terminal)

```bash
# Open a second SSH session:
ssh edg5@172.20.10.10
cd ~/captone_OG
nano gains.json
```

Edit and save `gains.json`. Changes are picked up within **0.5 seconds** — no restart needed. A `[GAIN CHANGE]` line prints in the main terminal whenever a value updates.

To reset all gains to defaults, delete `gains.json` — it will be recreated on the next loop tick.

### Step 6 — Stop and review

Press `Ctrl+C` to stop. The script auto-saves:
- `fpd_tune_data_<timestamp>.csv` — full telemetry log
- `fpd_tune_data_<timestamp>_analysis.png` — auto-generated 4-panel analysis plot

To generate a JSON summary from any run:

```bash
python3 firmware/parse_fpd_run.py fpd_tune_data_20260409_143022.csv
```

---

## Gain Tuning

### gains.json Parameter Reference

| Parameter | Default | Description |
|---|---|---|
| `ROLL_GAIN` | `600.0` | Roll proportional gain (Kp) |
| `PITCH_GAIN` | `300.0` | Pitch proportional gain (Kp) |
| `ROLL_KI` | `3.0` | Roll integral gain |
| `PITCH_KI` | `2.0` | Pitch integral gain |
| `MAX_I_ERPM` | `300.0` | Integral windup clamp (ERPM) |
| `ROLL_KD` | `10.0` | Roll derivative gain |
| `PITCH_KD` | `-5.0` | Pitch derivative gain (negative — axis polarity) |
| `MAX_D_ERPM` | `600.0` | Derivative output clamp (ERPM) |
| `D_FILTER_ALPHA` | `0.15` | EMA alpha for D-term filter (0 = heavy filtering, 1 = off) |
| `ROLL_KFF` | `70.0` | Roll feedforward gain from hull IMU rate |
| `PITCH_KFF` | `40.0` | Pitch feedforward gain from hull IMU rate |
| `FF_FILTER_ALPHA` | `0.4` | EMA alpha for FF-term filter |
| `FF_RATE_DEADBAND_DPS` | `1.5` | FF suppressed below this hull rate (noise guard at rest) |
| `ANGLE_FILTER_ALPHA` | `1.0` | EMA alpha for angle input (1.0 = no filter) |
| `ROLL_DEADBAND_DEG` | `0.5` | Roll command zeroed inside this angle band |
| `PITCH_DEADBAND_DEG` | `1.0` | Pitch command zeroed inside this angle band |
| `MAX_MOTOR_AMPS` | `10.0` | Per-motor current limit (A) |
| `MAX_ERPM` | `5000.0` | Total command clamp (ERPM) |

### Known Stability Ceilings (from testing)

| Axis | Stable ceiling | Instability threshold | Notes |
|---|---|---|---|
| Roll Kp (`ROLL_GAIN`) | 600 | 750+ | Oscillates above 750 without D increase |
| Pitch Kp (`PITCH_GAIN`) | 300 | 400+ | Oscillates above 400 without sufficient D damping |

### Tuning Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Slow, sluggish response | Kp too low | Increase `ROLL_GAIN` or `PITCH_GAIN` by 50 |
| High-frequency oscillation | Kp too high or D too low | Reduce Kp by 50, or increase Kd; lower `D_FILTER_ALPHA` |
| Sustained low-frequency oscillation | Ki windup | Reduce `ROLL_KI`/`PITCH_KI`; verify `MAX_I_ERPM` clamp |
| Overshoot and ringing after step | D too low | Increase `ROLL_KD`/`PITCH_KD` by 2–5 |
| Buzzing / chatter at rest | Deadband too small | Increase `ROLL_DEADBAND_DEG` or `PITCH_DEADBAND_DEG` |
| FF causing oscillation | Kff too high | Reduce `ROLL_KFF`/`PITCH_KFF` by 10; increase `FF_RATE_DEADBAND_DPS` |
| Platform drifts to one side | IMU bias or Ki too small | Run `calibration.py`; small Ki increase |
| Overcurrent fault / motor stop | Gains too aggressive | Reduce `MAX_MOTOR_AMPS`; reduce Kp/Kd |

---

## Data & Results

### Test Environments

| Environment | Description | Folder |
|---|---|---|
| Dry bench | Lab bench, manual disturbance — initial controller development | `data/early_tests/` |
| Pool tests | FAU pool, controlled 0.5–1 Hz sinusoidal disturbances | `data/pool_tests/` |
| Wave tank | FAU wave tank, programmatic sea-state waveforms | `data/wavetank/` |
| WAM-V vessel | Open water on WAM-V USV at FAU marine facility | `data/wamv/` |

### Controller Development Progression

Development followed an iterative complexity ladder:

1. **PD** — baseline derivative damping, no feedforward
2. **PDF** — derivative on measurement variant (reduced kick on setpoint change)
3. **FPD** — feedforward from hull IMU added (major disturbance rejection improvement)
4. **FPID** — integral term added to eliminate steady-state trim bias

Analysis plots for all test phases are in `analysis/`.

### Analyzing a Run

```bash
# Print stats + save JSON summary
python3 firmware/parse_fpd_run.py data/wavetank/fpd_tune_data_20260409_143022.csv

# Each run also auto-generates an _analysis.png on Ctrl+C
```

---

## Repo Structure

```
Maritime-Active-Gimbal-Platform/
├── firmware/                        # All runnable Python scripts
│   ├── main_fpd_tune_headless.py    # PRIMARY: headless 200 Hz FF+PID
│   ├── main_fpd_tune.py             # FF+PID with live plot option
│   ├── main_fpd_control.py          # Fixed-gain FPD
│   ├── main_ff_control.py           # Feedforward only
│   ├── main_pd_control.py           # PD only
│   ├── main_pdf_control.py          # PDF variant
│   ├── main_vel_control.py          # Velocity loop test
│   ├── main_pos_control.py          # Position loop test
│   ├── main_rpm_stabilizer.py       # RPM stabilization
│   ├── main_dual_imu.py             # Dual IMU debug
│   ├── main.py                      # Single-axis minimal entry
│   ├── imu_sensor.py                # BNO08x HAL
│   ├── serial_motor_driver.py       # CubeMars UART HAL
│   ├── pd_controller.py             # PD controller class
│   ├── motor_test.py                # Motor recover/sweep utility
│   ├── system_check_uart.py         # Pre-flight diagnostics
│   ├── calibration.py               # IMU calibration
│   ├── auto_tune.py                 # Bayesian gain optimizer (Optuna)
│   └── parse_fpd_run.py             # CSV → JSON run parser
│
├── analysis/                        # Post-run analysis plots (PNG)
│   └── dry_tests/                   # Dry bench test plots
│
├── data/                            # Telemetry CSVs by test phase
│   ├── early_tests/
│   ├── pool_tests/
│   ├── wavetank/
│   ├── wamv/
│   ├── gains.json                   # Active gains config (live-editable)
│   └── gains_system_tune.json       # Saved gains snapshot
│
├── config/                          # Configuration files
│
├── docs/                            # Documentation and references
│   ├── Gimbal_control_diagram (1).html
│   └── ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator.pdf
│
├── hardware/                        # Hardware design files (CAD, schematics)
├── media/                           # Photos and video
│
├── requirements.txt
├── .gitignore
└── README.md
```

---

## Communication Protocol

### CubeMars AK Series UART Protocol

Reference: *AK Series Product Manual v3.2.0, Section 4.3.2*

#### Frame Format

```
[0xAA] [Len] [Cmd] [Data...] [CRC_H] [CRC_L] [0xBB]
```

| Byte | Field | Description |
|---|---|---|
| `0xAA` | Start | Fixed frame start byte |
| `Len` | Length | `1 + len(Data)` — counts Cmd + Data, excludes header/CRC/stop |
| `Cmd` | Command | Command byte (see table below) |
| `Data...` | Payload | Command-specific payload |
| `CRC_H/L` | CRC16 | CRC16-CCITT over `[Cmd] + Data` (poly=0x1021, init=0x0000, MSB-first, no reflection) |
| `0xBB` | Stop | Fixed frame stop byte |

#### Command Bytes

| Cmd | Name | Description |
|---|---|---|
| `0x01` | Set ERPM | Set motor electrical RPM |
| `0x02` | Set Current | Set motor torque current (A) |
| `0x03` | Set Duty | Set duty cycle (%) |
| `0x04` | Set Position | Set motor position (deg) |
| `0x05` | Set Velocity | Set velocity with current limit |
| `0x45` | Get Telemetry | Request motor state packet |

#### Telemetry Response Fields

| Field | Bytes | Scale | Notes |
|---|---|---|---|
| Position | 2 | raw / 100.0 | degrees |
| ERPM | 2 | signed, direct | electrical RPM |
| Voltage | 2 | raw / 10.0 | volts |
| Current | 2 | signed / 100.0 | amps |
| Temperature | 2 | raw / 10.0 | °C |

CRC16 example verification (from manual pages 57–58):  
`crc16([0x45]) == 0x1861` | `crc16([0x47,0x00,0x00,0x13,0x88]) == 0x301C`

---

## Team & Acknowledgments

**Noah Brande** — Controls & Software Engineer  
Florida Atlantic University  
College of Engineering & Computer Science  
Senior Design Capstone — Class of 2026

Special thanks to the FAU Marine & Environmental Systems department for wave tank and WAM-V vessel access.

---

## License

This project is licensed under the **MIT License**.  
© 2026 Noah Brande — Florida Atlantic University
