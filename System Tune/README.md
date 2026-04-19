# WAM-V Stabilization Platform — FPD Control

Feed-forward + PD (FPD) active stabilization for a 2-DOF (roll/pitch) gimbal
mounted on a WAM-V USV.  200 Hz control loop using dual BNO08x IMUs and
CubeMars AK60-39 brushless motors.

---

## Repository layout

```
.
├── calibration.py          # Motor safeguard calibration (run before first use)
├── main_fpd_tune.py        # Main 200 Hz control loop with live gains reloading
├── gains.json              # Active gains — edit while running to tune live
├── tools/
│   └── parse_fpd_run.py    # Parse a CSV log → tracker entry JSON
└── tracker/
    └── index.html          # Gains version tracker (open in browser, no server needed)
```

---

## Quick start

### 1. Calibrate safeguards
```bash
python3 calibration.py
```
Rotate the platform to each physical limit.  Copy the printed limit values
into the `IMU_ROLL_LIMIT` / `MOTOR_PITCH_LIMIT` constants at the top of
`main_fpd_tune.py`.

### 2. Run the controller
```bash
python3 main_fpd_tune.py
```
The script arms both motors, waits 3 s for IMU stabilisation, then enters the
200 Hz control loop.  Press **Ctrl+C** to stop — motors are zeroed and a
`_analysis.png` plot is auto-generated beside the CSV.

**Live tuning:** in a second terminal, edit `gains.json` and save.  Changes
are picked up within ~0.5 s without restarting.

```bash
nano gains.json
```

### 3. Enable the live plot (optional)
Set `LIVE_PLOT = True` at the top of `main_fpd_tune.py`.
Requires a display (or X11 forwarding over SSH).

---

## Gains reference

| Key | Default | Description |
|---|---|---|
| `ROLL_GAIN` | 600 | P gain — roll angle error → ERPM |
| `PITCH_GAIN` | 300 | P gain — pitch angle error → ERPM |
| `ROLL_KI` / `PITCH_KI` | 3 / 2 | Integral gain (anti-windup via `MAX_I_ERPM`) |
| `ROLL_KD` / `PITCH_KD` | 10 / −5 | D gain on IMU1 angular rate |
| `ROLL_KFF` / `PITCH_KFF` | 70 / 40 | Feed-forward gain on IMU2 (boat) angular rate |
| `MAX_D_ERPM` | 600 | Hard cap on D-term contribution |
| `D_FILTER_ALPHA` | 0.15 | EMA coefficient on IMU1 rate (lower = more filtering) |
| `FF_FILTER_ALPHA` | 0.4 | Second-order EMA on IMU2 rate |
| `FF_RATE_DEADBAND_DPS` | 1.5 | FF disabled below this IMU2 rate (noise gate) |
| `ROLL_DEADBAND_DEG` | 0.5 | P/I disabled inside this angle band |
| `PITCH_DEADBAND_DEG` | 1.0 | P/I disabled inside this angle band |
| `MAX_MOTOR_AMPS` | 10 | Overcurrent fault threshold |
| `MAX_ERPM` | 5000 | Global ERPM clamp |

**Known stability ceilings:**
- `PITCH_GAIN` ≥ 400 oscillates without sufficient D damping
- `ROLL_GAIN` ≥ 750 oscillates

---

## Gains tracker

Open `tracker/index.html` directly in a browser (no server needed).
Data is stored in `localStorage` — it persists between sessions.

### Workflow after each run

```bash
# Parse the CSV and generate a tracker entry
python3 tools/parse_fpd_run.py fpd_tune_data_20250419_142301.csv gains.json
```

This prints a summary and writes `fpd_tune_data_*_tracker_entry.json`.
Paste the JSON into the **Import CSV result** tab of the tracker.

The tracker extracts automatically:
- All gains (from `gains.json`, or from `[GAIN CHANGE]` log entries in the CSV)
- Roll / pitch RMS reduction %
- Peak disturbance and platform angles
- Max motor currents
- Median wave periods (same zero-crossing algorithm as the live plot)
- FF and D term peak ERPM values

---

## Hardware

| Component | Part |
|---|---|
| Stabilisation platform | Custom 2-DOF gimbal |
| Roll / Pitch motors | CubeMars AK60-39 V3.0 |
| Platform IMU (IMU1) | BNO08x on I2C bus 7 |
| Boat reference IMU (IMU2) | BNO08x on I2C bus 1 |
| Host computer | Raspberry Pi / Jetson (Ubuntu) |
| Motor ports | `/dev/ttyROLL`, `/dev/ttyPITCH` (udev symlinks) |

---

## Test results summary

| Test | Duration | Roll %↓ | Pitch %↓ | Notes |
|---|---|---|---|---|
| Wavetank Test 4 | 67 s | 80.8 % | 59.0 % | Controlled waves, low current draw |
| Pool Test 2 | 153 s | 81.9 % | 66.0 % | Irregular disturbances, higher current |

---

## Dependencies

```bash
pip install numpy pandas matplotlib scipy
```

Hardware drivers (`imu_sensor`, `serial_motor_driver`) are not included here —
add them to the repo root or install as a package.
