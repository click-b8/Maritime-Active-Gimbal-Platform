"""Parse a timestamped FPD run CSV into a JSON summary.

Usage:
    python3 parse_fpd_run.py <path/to/fpd_tune_data_YYYYMMDD_HHMMSS.csv>

Output:
    <same directory>/<csv_stem>_summary.json   — machine-readable summary
    stdout                                      — human-readable report
"""

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path


def _rms(values: list) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v ** 2 for v in values) / len(values))


def _stats(values: list) -> dict:
    n = len(values)
    if n == 0:
        return {"mean": 0.0, "std": 0.0, "rms": 0.0, "max_abs": 0.0, "n": 0}
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / n)
    rms = _rms(values)
    max_abs = max(abs(v) for v in values)
    return {"mean": round(mean, 4), "std": round(std, 4),
            "rms": round(rms, 4), "max_abs": round(max_abs, 4), "n": n}


def parse_csv(csv_path: Path) -> dict:
    import csv

    rows = []
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)

    if not rows:
        raise ValueError("CSV is empty or has no data rows.")

    def col(name: str) -> list:
        return [float(r[name]) for r in rows if name in r and r[name].strip() != ""]

    t          = col("t_s")
    roll1      = col("imu1_roll_deg")
    pitch1     = col("imu1_pitch_deg")
    roll2      = col("imu2_roll_deg")   if "imu2_roll_deg"  in rows[0] else []
    pitch2     = col("imu2_pitch_deg")  if "imu2_pitch_deg" in rows[0] else []
    dt_ms      = col("dt_ms")

    duration   = round(t[-1] - t[0], 3) if len(t) > 1 else 0.0
    loop_count = len(rows)
    mean_hz    = round(loop_count / duration, 1) if duration > 0 else 0.0

    # Infer run timestamp from filename  (fpd_tune_data_YYYYMMDD_HHMMSS.csv)
    stem = csv_path.stem
    m = re.search(r"(\d{8})_(\d{6})", stem)
    run_ts = f"{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]}T{m.group(2)[:2]}:{m.group(2)[2:4]}:{m.group(2)[4:]}" if m else "unknown"

    # Gain event log — unique non-empty entries
    gain_events = list({r.get("gain_event", "") for r in rows if r.get("gain_event", "").strip()})

    # Controller performance: RMS of platform angle (imu1)
    roll_rms  = _rms(roll1)
    pitch_rms = _rms(pitch1)
    perf_score = round(math.sqrt((roll_rms ** 2 + pitch_rms ** 2) / 2), 4)

    # Disturbance RMS (hull IMU) for reduction ratio
    def reduction(dist: list, plat: list) -> float:
        d = _rms(dist); p = _rms(plat)
        return round((1 - p / d) * 100, 1) if d > 0 else 0.0

    summary = {
        "run_metadata": {
            "timestamp": run_ts,
            "source_file": csv_path.name,
            "duration_s": duration,
            "loop_count": loop_count,
            "mean_loop_hz": mean_hz,
            "mean_dt_ms": round(sum(dt_ms) / len(dt_ms), 3) if dt_ms else None,
        },
        "roll": {
            "platform_imu1": _stats(roll1),
            "hull_imu2": _stats(roll2) if roll2 else None,
            "disturbance_reduction_pct": reduction(roll2, roll1) if roll2 else None,
        },
        "pitch": {
            "platform_imu1": _stats(pitch1),
            "hull_imu2": _stats(pitch2) if pitch2 else None,
            "disturbance_reduction_pct": reduction(pitch2, pitch1) if pitch2 else None,
        },
        "controller_performance": {
            "roll_rms_error_deg": round(roll_rms, 4),
            "pitch_rms_error_deg": round(pitch_rms, 4),
            "combined_rms_score_deg": perf_score,
        },
        "gain_events": gain_events,
    }
    return summary


def print_report(summary: dict) -> None:
    m   = summary["run_metadata"]
    r   = summary["roll"]
    p   = summary["pitch"]
    perf = summary["controller_performance"]

    print("=" * 60)
    print("  FF+PD RUN SUMMARY")
    print("=" * 60)
    print(f"  Timestamp  : {m['timestamp']}")
    print(f"  File       : {m['source_file']}")
    print(f"  Duration   : {m['duration_s']} s  ({m['loop_count']} loops @ {m['mean_loop_hz']} Hz)")
    if m['mean_dt_ms']:
        print(f"  Mean dt    : {m['mean_dt_ms']} ms")
    print()
    print("  ROLL (IMU1 platform)")
    ri = r["platform_imu1"]
    print(f"    RMS={ri['rms']:.3f}°  std={ri['std']:.3f}°  max={ri['max_abs']:.3f}°  mean={ri['mean']:.3f}°")
    if r["hull_imu2"]:
        rh = r["hull_imu2"]
        print(f"    Hull RMS={rh['rms']:.3f}°  →  reduction={r['disturbance_reduction_pct']}%")
    print()
    print("  PITCH (IMU1 platform)")
    pi = p["platform_imu1"]
    print(f"    RMS={pi['rms']:.3f}°  std={pi['std']:.3f}°  max={pi['max_abs']:.3f}°  mean={pi['mean']:.3f}°")
    if p["hull_imu2"]:
        ph = p["hull_imu2"]
        print(f"    Hull RMS={ph['rms']:.3f}°  →  reduction={p['disturbance_reduction_pct']}%")
    print()
    print("  PERFORMANCE SCORE")
    print(f"    Roll RMS error  : {perf['roll_rms_error_deg']:.4f}°")
    print(f"    Pitch RMS error : {perf['pitch_rms_error_deg']:.4f}°")
    print(f"    Combined score  : {perf['combined_rms_score_deg']:.4f}° RMS  (lower is better)")
    if summary["gain_events"]:
        print()
        print("  GAIN CHANGES DURING RUN")
        for ev in summary["gain_events"]:
            print(f"    • {ev}")
    print("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse an FPD run CSV into a JSON summary.")
    parser.add_argument("csv_file", help="Path to fpd_tune_data_*.csv")
    args = parser.parse_args()

    csv_path = Path(args.csv_file).resolve()
    if not csv_path.exists():
        print(f"[ERROR] File not found: {csv_path}", file=sys.stderr)
        sys.exit(1)
    if csv_path.suffix.lower() != ".csv":
        print(f"[ERROR] Expected a .csv file, got: {csv_path.name}", file=sys.stderr)
        sys.exit(1)

    summary = parse_csv(csv_path)
    print_report(summary)

    out_path = csv_path.with_name(csv_path.stem + "_summary.json")
    with open(out_path, "w") as fh:
        json.dump(summary, fh, indent=2)
    print(f"\n[INFO] JSON summary saved to: {out_path}")


if __name__ == "__main__":
    main()
