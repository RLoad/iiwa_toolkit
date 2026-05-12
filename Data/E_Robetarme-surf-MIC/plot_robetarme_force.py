#!/usr/bin/env python3
"""
Plot all Robetarme_surf_MIC force logs under a recording root.

Expected layout:
  <root>/<timestamp>/Force.txt

Each Force.txt block is expected as:
  time: <float>
  real_force_filtered_: fx fy fz
  desired_force_: fx fy fz
  ----------------------------------------
"""

import argparse
import math
import re
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt


TIME_RE = re.compile(r"^time:\s*([0-9eE+\-\.]+)\s*$")
REAL_RE = re.compile(r"^real_force_filtered_:\s*(.+)$")
DES_RE = re.compile(r"^desired_force_:\s*(.+)$")


def _parse_vec3(raw: str) -> Tuple[float, float, float]:
    parts = raw.strip().split()
    if len(parts) < 3:
        raise ValueError(f"Expected 3 values, got: {raw}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def parse_force_file(path: Path) -> Tuple[List[float], List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    times: List[float] = []
    real_vals: List[Tuple[float, float, float]] = []
    desired_vals: List[Tuple[float, float, float]] = []

    current_time = None
    current_real = None
    current_desired = None

    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue

        m = TIME_RE.match(line)
        if m:
            current_time = float(m.group(1))
            continue

        m = REAL_RE.match(line)
        if m:
            current_real = _parse_vec3(m.group(1))
            continue

        m = DES_RE.match(line)
        if m:
            current_desired = _parse_vec3(m.group(1))
            continue

        if line.startswith("---"):
            if current_time is not None and current_real is not None and current_desired is not None:
                times.append(current_time)
                real_vals.append(current_real)
                desired_vals.append(current_desired)
            current_time = None
            current_real = None
            current_desired = None

    # Handle files that do not end with separator
    if current_time is not None and current_real is not None and current_desired is not None:
        times.append(current_time)
        real_vals.append(current_real)
        desired_vals.append(current_desired)

    return times, real_vals, desired_vals


def vec_norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def plot_single_run(force_file: Path, save_only: bool = False) -> Path:
    times, real_vals, desired_vals = parse_force_file(force_file)
    if not times:
        raise ValueError(f"No valid samples parsed from: {force_file}")

    t0 = times[0]
    t = [x - t0 for x in times]

    real_x = [v[0] for v in real_vals]
    real_y = [v[1] for v in real_vals]
    real_z = [v[2] for v in real_vals]
    des_x = [v[0] for v in desired_vals]
    des_y = [v[1] for v in desired_vals]
    des_z = [v[2] for v in desired_vals]
    real_n = [vec_norm(v) for v in real_vals]
    des_n = [vec_norm(v) for v in desired_vals]

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    run_name = force_file.parent.name

    axes[0].plot(t, real_x, label="real_fx")
    axes[0].plot(t, des_x, "--", label="desired_fx")
    axes[0].set_ylabel("Fx [N]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(t, real_y, label="real_fy")
    axes[1].plot(t, des_y, "--", label="desired_fy")
    axes[1].set_ylabel("Fy [N]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(t, real_z, label="real_fz")
    axes[2].plot(t, des_z, "--", label="desired_fz")
    axes[2].set_ylabel("Fz [N]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    axes[3].plot(t, real_n, label="|real_force|")
    axes[3].plot(t, des_n, "--", label="|desired_force|")
    axes[3].set_ylabel("Norm [N]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")

    fig.suptitle(f"Robetarme force log: {run_name}")
    fig.tight_layout(rect=[0, 0.02, 1, 0.98])

    out_png = force_file.parent / "Force_plot.png"
    fig.savefig(out_png, dpi=150)
    if not save_only:
        plt.show()
    plt.close(fig)
    return out_png


def collect_force_files(root: Path) -> List[Path]:
    return sorted(root.rglob("Force.txt"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot all Robetarme_surf_MIC Force.txt logs.")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("/home/ros/ros_ws/src/ds_motion_generator_iiwa/Recordings/robot_exe/2024-3-6-Robetarme-surf-MIC-adaptive"),
        help="Root folder containing timestamp subfolders with Force.txt",
    )
    parser.add_argument(
        "--save-only",
        action="store_true",
        help="Save PNGs only (do not open interactive windows).",
    )
    args = parser.parse_args()

    if not args.root.exists():
        raise FileNotFoundError(f"Root path not found: {args.root}")

    force_files = collect_force_files(args.root)
    if not force_files:
        raise FileNotFoundError(f"No Force.txt found under: {args.root}")

    print(f"Found {len(force_files)} Force.txt files under {args.root}")
    for f in force_files:
        try:
            out = plot_single_run(f, save_only=args.save_only)
            print(f"[OK] {f} -> {out}")
        except Exception as exc:
            print(f"[SKIP] {f}: {exc}")


if __name__ == "__main__":
    main()
