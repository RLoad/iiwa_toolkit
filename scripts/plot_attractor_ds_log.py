#!/usr/bin/env python3
"""Validate + plot an attractor_ds_py log CSV produced by node_attractor_ds_gazebo.

Usage:
    plot_attractor_ds_log.py <csv> [<outdir> [<schedule.txt>]]

Outputs (next to csv unless outdir given):
    <stem>_summary.txt              text-only sanity report
    <stem>_2d_position.png          x/y/z (m) vs t, ee_pos + attractor_pos
    <stem>_2d_velocity.png          vx/vy/vz (m/s) vs t, ee_vel + v_des + v_des_raw
    <stem>_2d_quat.png              ee_quat + attractor_quat (wxyz) over t
    <stem>_2d_gains.png             K_linear, damping_a, damping_b over t
    <stem>_2d_torques.png           tau_0..tau_6 joint torques over t
    <stem>_3d_path.png              3D ee path + attractor scatter

Sanity checks:
  * effective control rate
  * fraction of time |v_des_raw| > velocity_limit  (saturation rate)
  * RMS position-tracking error vs attractor
  * RMS velocity-tracking error vs v_des
  * convergence to each attractor segment (from schedule.txt if provided)
"""

from __future__ import annotations

import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401


# ---------------------------------------------------------------------------
def load_csv(path: str):
    with open(path) as fh:
        header = fh.readline().rstrip("\n").split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return header, data


def col(header, data, name):
    return data[:, header.index(name)]


def cols(header, data, names):
    return np.stack([col(header, data, n) for n in names], axis=1)


def parse_schedule(path: str):
    """Return list of (t_rel_s, pos[3], quat[4], gains_or_none, label)."""
    if not path or not os.path.isfile(path):
        return []
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 12:
                continue
            try:
                t   = float(parts[0])
                pos = [float(parts[1]), float(parts[2]), float(parts[3])]
                qt  = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
            except ValueError:
                continue
            try:
                gains = [float(parts[8]), float(parts[9]), float(parts[10])]
            except ValueError:
                gains = None
            label = parts[11]
            out.append((t, pos, qt, gains, label))
    return out


# ---------------------------------------------------------------------------
def summarize(header, data, out_txt: str, schedule, velocity_limit=0.15) -> dict:
    t       = col(header, data, "t")
    t_csv0  = t[0]
    t       = t - t_csv0
    dur     = t[-1]
    n       = len(t)

    ee_pos    = cols(header, data, ["ee_pos_x",       "ee_pos_y",       "ee_pos_z"])
    ee_vel    = cols(header, data, ["ee_vel_x",       "ee_vel_y",       "ee_vel_z"])
    att_pos   = cols(header, data, ["attractor_pos_x","attractor_pos_y","attractor_pos_z"])
    v_des     = cols(header, data, ["v_des_x",        "v_des_y",        "v_des_z"])
    v_des_raw = cols(header, data, ["v_des_raw_x",    "v_des_raw_y",    "v_des_raw_z"])
    first     = col (header, data, "first_flag").astype(int)

    pos_err  = np.linalg.norm(ee_pos - att_pos, axis=1)
    vel_err  = np.linalg.norm(ee_vel - v_des,   axis=1)
    raw_mag  = np.linalg.norm(v_des_raw, axis=1)
    sat_frac = float((raw_mag > velocity_limit + 1e-6).mean())

    # Per-segment convergence: take the last sample of each segment, before
    # the next segment starts.  Treat the trailing portion as the final segment.
    per_seg = []
    if schedule:
        seg_starts = [s[0] for s in schedule]
        seg_ends   = seg_starts[1:] + [dur + 1e6]
        for k, (t_s, _, _, _, lbl) in enumerate(schedule):
            t_e = seg_ends[k]
            mask = (t >= t_s) & (t < t_e)
            if not mask.any():
                continue
            i_end = np.nonzero(mask)[0][-1]
            per_seg.append({
                "seg": k,
                "label": lbl,
                "t_start": float(t_s),
                "t_end_observed": float(t[i_end]),
                "final_pos_err_m": float(pos_err[i_end]),
                "rms_vel_err_m_s": float(np.sqrt(np.mean(vel_err[mask] ** 2))),
                "sat_frac":         float((raw_mag[mask] > velocity_limit + 1e-6).mean()),
            })

    tau_cols = ["tau%d" % i for i in range(7)]
    if all(c in header for c in tau_cols):
        tau = cols(header, data, tau_cols)
        tau_max_abs = [float(np.max(np.abs(tau[:, i]))) for i in range(7)]
        tau_finite  = bool(np.isfinite(tau).all())
    else:
        tau_max_abs = None
        tau_finite  = None

    info = dict(
        n_rows=n, duration_s=float(dur), eff_rate_hz=float(n / max(dur, 1e-9)),
        velocity_saturation_frac=sat_frac,
        pos_err_overall_rms_m=float(np.sqrt(np.mean(pos_err ** 2))),
        vel_err_overall_rms_m_s=float(np.sqrt(np.mean(vel_err ** 2))),
        pos_err_final_m=float(pos_err[-1]),
        ee_pos_first=[float(x) for x in ee_pos[0]],
        ee_pos_last=[float(x) for x in ee_pos[-1]],
        attractor_first=[float(x) for x in att_pos[0]],
        attractor_last=[float(x) for x in att_pos[-1]],
        frac_init_phase=float(first.mean()),
        tau_max_abs_per_joint=tau_max_abs,
        tau_all_finite=tau_finite,
        per_segment=per_seg,
    )

    with open(out_txt, "w") as fh:
        fh.write("attractor_ds_py log summary\n")
        fh.write("=" * 42 + "\n")
        for k, v in info.items():
            if k == "per_segment":
                fh.write("per_segment:\n")
                for ps in v:
                    fh.write("  seg %d (%s):\n" % (ps["seg"], ps["label"]))
                    for kk, vv in ps.items():
                        if kk in ("seg", "label"): continue
                        fh.write("    %-20s : %s\n" % (kk, vv))
                continue
            fh.write("%-26s : %s\n" % (k, v))
    return info


# ---------------------------------------------------------------------------
def plot_all(header, data, stem: str, outdir: str, schedule=None):
    t = col(header, data, "t")
    t = t - t[0]
    schedule = schedule or []

    ee_pos    = cols(header, data, ["ee_pos_x", "ee_pos_y", "ee_pos_z"])
    ee_quat   = cols(header, data, ["ee_quat_w", "ee_quat_x", "ee_quat_y", "ee_quat_z"])
    ee_vel    = cols(header, data, ["ee_vel_x", "ee_vel_y", "ee_vel_z"])
    att_pos   = cols(header, data, ["attractor_pos_x","attractor_pos_y","attractor_pos_z"])
    att_quat  = cols(header, data, ["attractor_quat_w","attractor_quat_x","attractor_quat_y","attractor_quat_z"])
    v_des     = cols(header, data, ["v_des_x", "v_des_y", "v_des_z"])
    v_des_raw = cols(header, data, ["v_des_raw_x","v_des_raw_y","v_des_raw_z"])
    K_lin = col(header, data, "K_linear")
    damp_a= col(header, data, "damping_a")
    damp_b= col(header, data, "damping_b")

    def annotate_schedule(ax, with_labels=True):
        ylo, yhi = ax.get_ylim()
        for k, (t_s, _, _, _, lbl) in enumerate(schedule):
            ax.axvline(t_s, color="red", alpha=0.55, lw=0.9, ls=":")
            if with_labels:
                y = ylo + (yhi - ylo) * (0.93 - 0.04 * (k % 5))
                ax.text(t_s, y, lbl.split("(")[0].strip(),
                        color="red", fontsize=7, rotation=90,
                        ha="right", va="top", alpha=0.9)

    # --- 2D: position vs time ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes_lbl = ["x", "y", "z"]
    for i, ax in enumerate(axs):
        ax.plot(t, ee_pos[:, i],  label="ee_pos_%s" % axes_lbl[i])
        ax.plot(t, att_pos[:, i], label="attractor_pos_%s" % axes_lbl[i],
                ls="--", lw=1.0)
        ax.set_ylabel("m")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
        annotate_schedule(ax, with_labels=(i == 0))
    axs[-1].set_xlabel("t (s)")
    fig.suptitle("EE position vs attractor")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "%s_2d_position.png" % stem), dpi=120)
    plt.close(fig)

    # --- 2D: velocity vs time ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, ax in enumerate(axs):
        ax.plot(t, ee_vel[:, i],    label="ee_vel_%s" % axes_lbl[i])
        ax.plot(t, v_des[:, i],     label="v_des_%s (clamped)" % axes_lbl[i], ls="--")
        ax.plot(t, v_des_raw[:, i], label="v_des_raw_%s" % axes_lbl[i],
                ls=":", color="grey", alpha=0.7)
        ax.set_ylabel("m/s")
        ax.legend(loc="best", fontsize=7)
        ax.grid(True, alpha=0.3)
        annotate_schedule(ax, with_labels=(i == 0))
    axs[-1].set_xlabel("t (s)")
    fig.suptitle("EE linear velocity vs commanded (clamped vs raw)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "%s_2d_velocity.png" % stem), dpi=120)
    plt.close(fig)

    # --- 2D: quaternion components ---
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    q_lbl = ["w", "x", "y", "z"]
    for i, ax in enumerate(axs):
        ax.plot(t, ee_quat[:, i],  label="ee_quat_%s" % q_lbl[i])
        ax.plot(t, att_quat[:, i], label="attractor_quat_%s" % q_lbl[i], ls="--")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
        annotate_schedule(ax, with_labels=(i == 0))
    axs[-1].set_xlabel("t (s)")
    fig.suptitle("EE quaternion vs attractor quaternion")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "%s_2d_quat.png" % stem), dpi=120)
    plt.close(fig)

    # --- 2D: gains over time ---
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    ax.plot(t, K_lin,  label="K_linear")
    ax.plot(t, damp_a, label="damping_a")
    ax.plot(t, damp_b, label="damping_b")
    ax.set_xlabel("t (s)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    annotate_schedule(ax, with_labels=True)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "%s_2d_gains.png" % stem), dpi=120)
    plt.close(fig)

    # --- 2D: joint torques over time ---
    tau_cols = ["tau%d" % i for i in range(7)]
    if all(c in header for c in tau_cols):
        tau = cols(header, data, tau_cols)
        fig, ax = plt.subplots(1, 1, figsize=(10, 5))
        for i in range(7):
            ax.plot(t, tau[:, i], label="tau_%d" % i, lw=0.8)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("torque (N·m)")
        ax.legend(loc="best", ncol=4, fontsize=7)
        ax.grid(True, alpha=0.3)
        annotate_schedule(ax, with_labels=True)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "%s_2d_torques.png" % stem), dpi=120)
        plt.close(fig)

    # --- 3D: ee path + attractor waypoints ---
    fig = plt.figure(figsize=(8, 8))
    ax  = fig.add_subplot(111, projection="3d")
    ax.plot(ee_pos[:, 0], ee_pos[:, 1], ee_pos[:, 2], label="actual EE", lw=1.5)
    ax.plot(att_pos[:, 0], att_pos[:, 1], att_pos[:, 2],
            label="attractor (per-step)", ls="--", lw=0.8, alpha=0.5)
    if schedule:
        wp = np.array([s[1] for s in schedule])
        ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2], c="red", s=40,
                   label="attractor waypoints")
    ax.scatter(ee_pos[0, 0],  ee_pos[0, 1],  ee_pos[0, 2],  c="g", s=40, label="start")
    ax.scatter(ee_pos[-1, 0], ee_pos[-1, 1], ee_pos[-1, 2], c="orange", s=40, label="end")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.legend(loc="best", fontsize=8)
    ax.set_title("EE Cartesian path through attractor schedule")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "%s_3d_path.png" % stem), dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr); sys.exit(2)
    csv_path = os.path.abspath(sys.argv[1])
    outdir   = os.path.abspath(sys.argv[2]) if len(sys.argv) >= 3 else os.path.dirname(csv_path)
    sched_path = sys.argv[3] if len(sys.argv) >= 4 else os.path.join(outdir, "attractor_schedule.txt")
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(csv_path))[0]

    header, data = load_csv(csv_path)
    if len(data) < 10:
        print("[plot] CSV has only %d rows — controller likely crashed early." % len(data),
              file=sys.stderr)
        sys.exit(3)

    schedule = parse_schedule(sched_path)

    summary_txt = os.path.join(outdir, "%s_summary.txt" % stem)
    info = summarize(header, data, summary_txt, schedule)
    plot_all(header, data, stem, outdir, schedule=schedule)

    print("=== %s ===" % stem)
    for k, v in info.items():
        if k == "per_segment":
            print("per_segment:")
            for ps in v:
                print("  seg %d (%s):" % (ps["seg"], ps["label"]))
                for kk, vv in ps.items():
                    if kk in ("seg", "label"): continue
                    print("    %-20s : %s" % (kk, vv))
            continue
        print("%-26s : %s" % (k, v))
    print("plots written under:", outdir)


if __name__ == "__main__":
    main()
