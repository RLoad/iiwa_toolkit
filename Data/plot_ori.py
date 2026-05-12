#!/usr/bin/env python3
"""
Plot orientation and velocity data from Robot_state.txt.

Usage:
    python3 plot_ori.py                          # uses latest timestamped folder
    python3 plot_ori.py 2026-02-15-17-29-42      # specific folder
"""
import os
import sys
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# ─── paths ───────────────────────────────────────────────────────────────
DATA_ROOT = os.path.dirname(os.path.abspath(__file__))


def find_latest_folder(root):
    """Return the latest timestamped subfolder that contains Robot_state.txt."""
    candidates = sorted(glob.glob(os.path.join(root, "20*")))
    for c in reversed(candidates):
        if os.path.isfile(os.path.join(c, "Robot_state.txt")):
            return c
    raise FileNotFoundError("No timestamped folder with Robot_state.txt found in " + root)


def parse_robot_state(filepath):
    """Parse Robot_state.txt into dict of numpy arrays."""
    data = {
        "time": [],
        "target_pose": [],
        "real_pose": [],
        "eig_value": [],
        "real_force_filtered": [],
        "desired_force": [],
        "desired_velocity": [],
        "desired_velocity_e1": [],
        "desired_velocity_magnetic": [],
        "desired_ori_filtered": [],   # quat (x, y, z, w)
        "real_pose_ori": [],          # quat (x, y, z, w)
        "target_pose_ori": [],        # quat (x, y, z, w)
    }

    key_map = {
        "time:": "time",
        "target_pose_:": "target_pose",
        "real_pose_:": "real_pose",
        "eig_value:": "eig_value",
        "real_force_filtered_:": "real_force_filtered",
        "desired_force_:": "desired_force",
        "desired_velocity_:": "desired_velocity",
        "desired_velocity_e1:": "desired_velocity_e1",
        "desired_velocity_magnetic:": "desired_velocity_magnetic",
        "desired_ori_filtered:": "desired_ori_filtered",
        "real_pose_ori:": "real_pose_ori",
        "target_pose_ori:": "target_pose_ori",
    }

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("---"):
                continue
            for prefix, key in key_map.items():
                if line.startswith(prefix):
                    vals = line[len(prefix):].strip().split()
                    if key == "time":
                        data[key].append(float(vals[0]))
                    else:
                        data[key].append([float(v) for v in vals])
                    break

    # Convert to numpy, handle potentially missing fields
    out = {}
    for k, v in data.items():
        if len(v) > 0:
            out[k] = np.array(v)
        else:
            out[k] = None
    return out


def quat_to_euler(q_xyzw):
    """Convert (N,4) quaternion array [x,y,z,w] to (N,3) Euler angles [roll,pitch,yaw] in degrees."""
    # scipy expects [x,y,z,w]
    r = R.from_quat(q_xyzw)
    return r.as_euler("xyz", degrees=True)


def quat_to_z_axis(q_xyzw):
    """Extract z-axis direction from quaternion (N,4) -> (N,3)."""
    r = R.from_quat(q_xyzw)
    return r.as_matrix()[:, :, 2]  # third column = z-axis


def main():
    # Determine folder
    if len(sys.argv) > 1:
        folder = os.path.join(DATA_ROOT, sys.argv[1])
    else:
        folder = find_latest_folder(DATA_ROOT)

    filepath = os.path.join(folder, "Robot_state.txt")
    print(f"Reading: {filepath}")
    d = parse_robot_state(filepath)

    # Make time relative to start
    t = d["time"]
    if t is None or len(t) == 0:
        print("No data found!"); return
    t = t - t[0]

    # Create figure output folder
    fig_dir = os.path.join(folder, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    # ═══════════════════════════════════════════════════════════════════
    # 1. End-effector position (real vs target)
    # ═══════════════════════════════════════════════════════════════════
    if d["real_pose"] is not None and d["target_pose"] is not None:
        n = min(len(t), len(d["real_pose"]), len(d["target_pose"]))
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        labels = ["X", "Y", "Z"]
        for i, ax in enumerate(axes):
            ax.plot(t[:n], d["real_pose"][:n, i], "b-", label="real_pose")
            ax.plot(t[:n], d["target_pose"][:n, i], "r--", label="target_pose")
            ax.set_ylabel(f"{labels[i]} (m)")
            ax.legend(loc="upper right")
            ax.grid(True)
        axes[0].set_title("End-Effector Position")
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "position.png"), dpi=150)
        print(f"  Saved: {fig_dir}/position.png")

    # ═══════════════════════════════════════════════════════════════════
    # 2. Desired velocities
    # ═══════════════════════════════════════════════════════════════════
    vel_keys = ["desired_velocity", "desired_velocity_e1", "desired_velocity_magnetic"]
    vel_labels = ["desired_velocity (cmd)", "desired_velocity_e1", "desired_velocity_magnetic"]
    has_vel = any(d.get(k) is not None for k in vel_keys)
    if has_vel:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        colors = ["b", "g", "r"]
        coord_labels = ["X", "Y", "Z"]
        for i, ax in enumerate(axes):
            for vk, vl, vc in zip(vel_keys, vel_labels, colors):
                if d.get(vk) is not None:
                    n = min(len(t), len(d[vk]))
                    ax.plot(t[:n], d[vk][:n, i], color=vc, label=vl)
            ax.set_ylabel(f"Vel {coord_labels[i]} (m/s)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True)
        axes[0].set_title("Desired Velocities")
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "velocities.png"), dpi=150)
        print(f"  Saved: {fig_dir}/velocities.png")

    # ═══════════════════════════════════════════════════════════════════
    # 3. Orientation (Euler angles): real vs desired vs target
    # ═══════════════════════════════════════════════════════════════════
    ori_keys = ["desired_ori_filtered", "real_pose_ori", "target_pose_ori"]
    ori_labels = ["desired_ori (filtered)", "real_pose_ori", "target_pose_ori"]
    ori_colors = ["b", "g", "r"]
    has_ori = any(d.get(k) is not None for k in ori_keys)
    if has_ori:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        euler_labels = ["Roll", "Pitch", "Yaw"]
        for ok, ol, oc in zip(ori_keys, ori_labels, ori_colors):
            if d.get(ok) is not None:
                euler = quat_to_euler(d[ok])
                n = min(len(t), len(euler))
                for i, ax in enumerate(axes):
                    ax.plot(t[:n], euler[:n, i], color=oc, label=ol)
        for i, ax in enumerate(axes):
            ax.set_ylabel(f"{euler_labels[i]} (deg)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True)
        axes[0].set_title("Orientation (Euler Angles)")
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "orientation_euler.png"), dpi=150)
        print(f"  Saved: {fig_dir}/orientation_euler.png")

    # ═══════════════════════════════════════════════════════════════════
    # 4. Orientation quaternion components
    # ═══════════════════════════════════════════════════════════════════
    if has_ori:
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        quat_labels = ["qx", "qy", "qz", "qw"]
        for ok, ol, oc in zip(ori_keys, ori_labels, ori_colors):
            if d.get(ok) is not None:
                n = min(len(t), len(d[ok]))
                for i, ax in enumerate(axes):
                    ax.plot(t[:n], d[ok][:n, i], color=oc, label=ol)
        for i, ax in enumerate(axes):
            ax.set_ylabel(quat_labels[i])
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True)
        axes[0].set_title("Orientation (Quaternion Components)")
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "orientation_quat.png"), dpi=150)
        print(f"  Saved: {fig_dir}/orientation_quat.png")

    # ═══════════════════════════════════════════════════════════════════
    # 5. Tool z-axis direction (from desired_ori_filtered)
    # ═══════════════════════════════════════════════════════════════════
    if d.get("desired_ori_filtered") is not None:
        z_desired = quat_to_z_axis(d["desired_ori_filtered"])
        n = min(len(t), len(z_desired))
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        coord_labels = ["Zx", "Zy", "Zz"]
        for i, ax in enumerate(axes):
            ax.plot(t[:n], z_desired[:n, i], "b-", label="desired z-axis")
            # Also plot desired_velocity_magnetic direction for comparison
            if d.get("desired_velocity_magnetic") is not None:
                vm = d["desired_velocity_magnetic"]
                norms = np.linalg.norm(vm, axis=1, keepdims=True)
                norms = np.maximum(norms, 1e-8)
                vm_dir = vm / norms
                nm = min(len(t), len(vm_dir))
                ax.plot(t[:nm], vm_dir[:nm, i], "r--", alpha=0.6, label="vel_magnetic dir")
            ax.set_ylabel(coord_labels[i])
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True)
        axes[0].set_title("Tool Z-Axis Direction vs desired_velocity_magnetic Direction")
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "tool_z_axis.png"), dpi=150)
        print(f"  Saved: {fig_dir}/tool_z_axis.png")

    # ═══════════════════════════════════════════════════════════════════
    # 6. 3D trajectory + tool z-axis arrows (subsample for clarity)
    # ═══════════════════════════════════════════════════════════════════
    if d.get("real_pose") is not None and d.get("desired_ori_filtered") is not None:
        pos = d["real_pose"]
        z_ax = quat_to_z_axis(d["desired_ori_filtered"])
        n = min(len(pos), len(z_ax))
        pos = pos[:n]; z_ax = z_ax[:n]

        fig = plt.figure(figsize=(10, 8))
        ax3d = fig.add_subplot(111, projection="3d")
        ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], "b-", linewidth=0.8, label="trajectory")

        # Subsample arrows
        step = max(1, n // 50)
        scale = 0.02
        ax3d.quiver(pos[::step, 0], pos[::step, 1], pos[::step, 2],
                     z_ax[::step, 0] * scale, z_ax[::step, 1] * scale, z_ax[::step, 2] * scale,
                     color="r", alpha=0.7, label="tool z-axis")

        if d.get("target_pose") is not None:
            tp = d["target_pose"]
            ax3d.scatter(tp[0, 0], tp[0, 1], tp[0, 2], c="g", s=60, marker="*", label="target")

        ax3d.set_xlabel("X (m)")
        ax3d.set_ylabel("Y (m)")
        ax3d.set_zlabel("Z (m)")
        ax3d.set_title("3D Trajectory with Tool Z-Axis")
        ax3d.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(fig_dir, "trajectory_3d.png"), dpi=150)
        print(f"  Saved: {fig_dir}/trajectory_3d.png")

    plt.show()
    print("Done.")


if __name__ == "__main__":
    main()
