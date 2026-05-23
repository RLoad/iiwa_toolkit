#!/usr/bin/env python3
"""ROS-free verification of the AttractorDSController pure-Python interface.

This is the closest analogue to how the controller will be called in Isaac
Lab: a same-process Python simulator (PyBullet here, Isaac Sim there) loads
the iiwa URDF, advances physics, and at every step we

  1. read joint state                  q, dq
  2. compute FK + Jacobian externally  (kin provider — Isaac Lab uses the
                                        articulation API; here we use
                                        TorchKinematicsPK)
  3. call ctrl.compute_from_state(...) (kinematics=None construction)
  4. apply tau back to the simulator   (setJointMotorControl2 → TORQUE_CONTROL)

No ROS topics, no rosmaster — but we do shell out to ``rospack`` / ``xacro``
once at the start to xacro-expand iiwa_description's URDF so PyBullet can
load THE SAME ROBOT Gazebo loads (with F/T sensor + grabber).  This way the
attractor positions are identical to ds_planner.py's 5-segment schedule, and
the controller gains transfer directly between the two simulators.

Schedule: same 5 segments as the Gazebo run (`run_ds_planner_demo.sh` →
ds_planner.py SCHEDULE).  Pass criterion: best EE-attractor distance during
each segment < 5 cm.  All 5 should PASS — the controller converges and
holds each attractor to single-digit cm or better.

Dependencies:  torch, pytorch_kinematics, pybullet, numpy>=1.19  (all in the
docker image; for Isaac Lab's env install them the same way).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import numpy as np
import torch

# pytorch_kinematics imports matplotlib transitively; head off the tkagg default
os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — register 3d projection

import pybullet as p
import pybullet_data

# Make the library importable when the test runs from inside the package dir.
HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))   # iiwa_toolkit/src/

from iiwa_passive_ds import PassiveDSCore, AttractorDSController     # noqa: E402
from iiwa_passive_ds.kinematics import TorchKinematicsPK             # noqa: E402
from iiwa_passive_ds.examples.planner import (                       # noqa: E402
    Segment, SCHEDULE, TOTAL_DURATION, current_segment,
)


# ---------------------------------------------------------------------------
# Load the SAME URDF Gazebo loads: iiwa_description/urdf/iiwa14.urdf.xacro
# expanded with force_sensor:=True, grabber:=True.  This way attractor
# positions can be copied verbatim from ds_planner.py — no frame conversion
# needed.  We need rospack + xacro from the sourced ROS workspace, but only
# at URDF-build time; the actual control loop is still ROS-free.
JOINT_NAMES = [f"iiwa_joint_{i}" for i in range(1, 8)]
EE_LINK     = "iiwa_link_ee"
ROOT_LINK   = "iiwa_link_0"

# Per-joint torque clip — matches the URDF <effort> limits exactly (320, 320,
# 176, 176, 110, 40, 40 N·m) so we mirror real-iiwa firmware saturation and
# never command more than joints 6/7's 40 N·m wrist limit.
TAU_ABS_LIMIT = np.array([320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0])

# Schedule + current_segment() live in examples/planner.py — that file is
# the "planner" in this demo (substitute for an RL policy).  When you port
# to Isaac Lab, delete planner.py and let the policy supply these tensors
# directly each step.
DURATION   = TOTAL_DURATION   # 5 + 5*12 + 1 = 66 s
PASS_TOL_M = 0.05             # per-segment "best position error" tolerance

# Initial config = null_q so the null-space PD starts at zero error and
# doesn't fight the task DS during the transient.
INITIAL_Q = [0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0]

# Controller gains — identical to the Gazebo / Isaac-Lab interface test, since
# we now load the SAME URDF (with F/T sensor + grabber mass).
LAM0_POS      = 300.0
LAM1_POS      = 300.0
LAM0_ORI      = 5.0
LAM1_ORI      = 5.0
DS_GAIN_POS   = 5.0
DS_GAIN_ORI   = 2.5
NULL_SCALE    = 2.0     # Gazebo demo uses 10.0; PyBullet's slightly different
                        # inertia distribution makes the null-space PD pull the
                        # config away from each attractor's natural posture
                        # hard enough to leave 5–8 cm of steady-state position
                        # error (no integral action in the DS).  At 2.0 the
                        # null term still nudges back toward null_q during
                        # transients but doesn't dominate steady state.
NULL_DAMPING  = 1.0

JOINT_DAMPING = 2.0     # N·m / (rad/s) injected via changeDynamics on top of
                        # URDF's damping=0.5 — keeps the integrator stable at
                        # the SIM_HZ rate while the controller's effective
                        # joint damping (~lam·J·Jᵀ) takes over.
SIM_HZ        = 1000
SOLVER_ITERS  = 100


# ---------------------------------------------------------------------------
def _resolve_package_paths(urdf_text: str) -> str:
    """Rewrite ``package://<pkg>/<rest>`` URIs to absolute paths via rospack."""
    import re as _re

    cache: dict = {}

    def _sub(m):
        pkg, rest = m.group(1), m.group(2)
        if pkg not in cache:
            cache[pkg] = subprocess.check_output(
                ["rospack", "find", pkg], text=True
            ).strip()
        return f"{cache[pkg]}/{rest}"

    return _re.sub(r"package://([^/]+)/([^\"' >]+)", _sub, urdf_text)


def expand_iiwa_description_urdf(out_path: str = "/tmp/iiwa14_pybullet.urdf") -> str:
    """xacro-expand iiwa_description's iiwa14.urdf.xacro and resolve package://.

    Mirrors the Gazebo launch chain (iiwa_description/launch/iiwa14_upload.launch
    invoked with force_sensor:=True, grabber:=True via iiwa_gazebo.launch).
    """
    share = subprocess.check_output(
        ["rospack", "find", "iiwa_description"], text=True
    ).strip()
    xacro_in = os.path.join(share, "urdf", "iiwa14.urdf.xacro")
    urdf_text = subprocess.check_output(
        ["xacro", xacro_in,
         "hardware_interface:=EffortJointInterface",
         "robot_name:=iiwa",
         "force_sensor:=True",
         "grabber:=True"],
        text=True,
    )
    urdf_text = _resolve_package_paths(urdf_text)
    with open(out_path, "w") as fh:
        fh.write(urdf_text)
    return out_path


def bundled_iiwa_urdf() -> str:
    """Default URDF for the test: iiwa_description's (same as Gazebo loads)."""
    return expand_iiwa_description_urdf()


# ---------------------------------------------------------------------------
def setup_pybullet(urdf_path: str, gui: bool = False):
    """Connect, load iiwa, return (robot_id, joint_index_list).

    gui=False → ``p.DIRECT`` (headless, fast — used by CI / shell test runner).
    gui=True  → ``p.GUI``    (X11 window with the iiwa; needs DISPLAY).
    """
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        # A reasonable starting camera for the iiwa.
        p.resetDebugVisualizerCamera(cameraDistance=1.8,
                                     cameraYaw=45, cameraPitch=-25,
                                     cameraTargetPosition=[0.0, 0.0, 0.6])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # No gravity → mirrors Gazebo with the iiwa_gazebo_gravity_compensation_hw_sim
    # plugin, i.e. the robot is treated as if its motors already cancel gravity.
    # The controller can then focus on task tracking exactly like in real-iiwa /
    # Isaac-Lab-with-grav-comp setups.
    p.setGravity(0.0, 0.0, 0.0)
    p.setTimeStep(1.0 / SIM_HZ)
    p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERS)

    robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    n_joints = p.getNumJoints(robot_id)
    name2idx = {p.getJointInfo(robot_id, i)[1].decode(): i for i in range(n_joints)}
    missing = [n for n in JOINT_NAMES if n not in name2idx]
    if missing:
        sys.exit(f"[pybullet_test] joints not found in URDF: {missing}\n"
                 f"available: {list(name2idx)}")
    joint_indices = [name2idx[n] for n in JOINT_NAMES]

    # CRITICAL: disable PyBullet's default velocity motor controllers so our
    # torque commands aren't fought.  Apply zero-force velocity hold first,
    # then switch to pure torque control mode going forward.
    for j in joint_indices:
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0.0)

    # PyBullet quirk: ``setJointMotorControl2(VELOCITY_CONTROL, force=0)``
    # disables the URDF's <dynamics damping=...> motor friction.  Without
    # joint-level damping, the integrator at dt=2ms can blow up in a single
    # step under our stiff inner-DS gains (lam0_pos=100 → effective
    # joint-damping ~100 N·m/(rad/s) BUT only applied at the controller's
    # 500 Hz, between which the joint integrates freely).  Re-inject a hefty
    # viscous damping via changeDynamics so the controller's task-space DS
    # only needs to do task tracking, not also dynamic stabilization.
    for j in joint_indices:
        p.changeDynamics(robot_id, j, jointDamping=JOINT_DAMPING)

    return robot_id, joint_indices


def read_joint_state(robot_id, joint_indices):
    states = p.getJointStates(robot_id, joint_indices)
    q  = np.array([s[0] for s in states], dtype=np.float64)
    dq = np.array([s[1] for s in states], dtype=np.float64)
    return q, dq


def apply_torque(robot_id, joint_indices, tau: np.ndarray):
    # PyBullet's TORQUE_CONTROL ignores URDF effort limits, so clip manually
    # to the per-joint limits in iiwa_description (320, 320, 176, 176, 110,
    # 40, 40 N·m).  Joints 6/7 are the wrist with tight 40 N·m caps; if those
    # saturate it's usually a sign of bad orientation tuning.
    tau_clipped = np.clip(tau, -TAU_ABS_LIMIT, TAU_ABS_LIMIT)
    p.setJointMotorControlArray(robot_id, joint_indices,
                                p.TORQUE_CONTROL, forces=tau_clipped.tolist())


# ---------------------------------------------------------------------------
def build_controller():
    """Construct AttractorDSController the Isaac-Lab way: kinematics=None.

    Gain values exactly match the Gazebo test (`test_isaac_lab_interface.py`)
    so behaviour is comparable between the two simulators.
    """
    dtype = torch.float64
    null_q     = torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0], dtype=dtype)
    null_gains = torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0], dtype=dtype)
    core = PassiveDSCore(
        lam0_pos=LAM0_POS, lam1_pos=LAM1_POS,
        lam0_ori=LAM0_ORI, lam1_ori=LAM1_ORI,
        ds_gain_pos=DS_GAIN_POS, ds_gain_ori=DS_GAIN_ORI,
        max_dx=0.10, max_dq=0.20,
        null_q=null_q, null_gains=null_gains,
        null_damping=NULL_DAMPING, null_scale=NULL_SCALE,
        enable_init_phase=False,
    )
    return AttractorDSController(core, kinematics=None, velocity_limit=0.15)


def save_plots(log, per_seg, plot_dir: str, verbose: bool = True):
    """Write three PNGs to ``plot_dir``: 3D EE trajectory, per-axis error, tau norm."""
    if not log:
        if verbose:
            print("[pybullet_test] no log entries — skipping plots")
        return
    os.makedirs(plot_dir, exist_ok=True)

    t_arr   = np.array([r["t"]        for r in log])
    ee_arr  = np.array([r["ee_pos"]   for r in log])              # (N, 3)
    att_arr = np.array([r["att_pos"]  for r in log])              # (N, 3)
    err_arr = ee_arr - att_arr                                    # (N, 3)
    tau_arr = np.array([r["tau_norm"] for r in log])

    # --- 1. 3D EE trajectory + attractors ---------------------------------
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(ee_arr[:, 0], ee_arr[:, 1], ee_arr[:, 2],
            "-", color="#1f77b4", linewidth=1.2, label="EE trajectory")
    ax.scatter(ee_arr[0, 0], ee_arr[0, 1], ee_arr[0, 2],
               c="green", s=40, label="start")
    # Mark each unique attractor target
    seen = set()
    for r in log:
        key = tuple(r["att_pos"].round(4))
        if key in seen:
            continue
        seen.add(key)
        ax.scatter(*r["att_pos"], c="red", s=60, marker="*",
                   label="attractor" if len(seen) == 1 else None)
    ax.set_xlabel("x [m]");  ax.set_ylabel("y [m]");  ax.set_zlabel("z [m]")
    ax.set_title("PyBullet iiwa — EE trajectory vs attractor")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    f1 = os.path.join(plot_dir, "ee_trajectory_3d.png")
    fig.savefig(f1, dpi=140);  plt.close(fig)

    # --- 2. Per-axis position error vs time -------------------------------
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t_arr, err_arr[:, 0], label="x error", color="C0")
    ax.plot(t_arr, err_arr[:, 1], label="y error", color="C1")
    ax.plot(t_arr, err_arr[:, 2], label="z error", color="C2")
    norm = np.linalg.norm(err_arr, axis=1)
    ax.plot(t_arr, norm, label="||error||", color="k", linewidth=1.5)
    ax.axhline(PASS_TOL_M, color="gray", linestyle="--", linewidth=0.8,
               label=f"pass tol ({PASS_TOL_M*100:.0f} cm)")
    ax.axhline(-PASS_TOL_M, color="gray", linestyle="--", linewidth=0.8)
    ax.set_xlabel("t [s]");  ax.set_ylabel("ee − attractor  [m]")
    ax.set_title("Position tracking error")
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    f2 = os.path.join(plot_dir, "position_error.png")
    fig.savefig(f2, dpi=140);  plt.close(fig)

    # --- 3. Torque norm vs time -------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 3.0))
    ax.plot(t_arr, tau_arr, color="C3", linewidth=1.0)
    ax.set_xlabel("t [s]");  ax.set_ylabel("||tau||  [N·m]")
    ax.set_title("Commanded torque magnitude")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    f3 = os.path.join(plot_dir, "torque_norm.png")
    fig.savefig(f3, dpi=140);  plt.close(fig)

    if verbose:
        print(f"[pybullet_test] plots saved to {plot_dir}/")
        for f in (f1, f2, f3):
            print(f"    {os.path.basename(f)}")


# ---------------------------------------------------------------------------
def run_test(urdf_path: str, duration: float = DURATION, verbose: bool = True,
             gui: bool = False, plot_dir: str = ""):
    """Run the controller against PyBullet.  Returns (pass: bool, per_segment_info)."""
    # Build the controller AND the external kin provider (the Isaac-Lab pattern:
    # FK/J source lives outside the controller).
    ctrl = build_controller()
    if verbose:
        print("[pybullet_test] controller built with kinematics=None  velocity_limit=%.3f"
              % ctrl.velocity_limit)
    with open(urdf_path, "rb") as fh:
        urdf_bytes = fh.read()
    kin = TorchKinematicsPK(urdf_string=urdf_bytes,
                            ee_link=EE_LINK, root_link=ROOT_LINK,
                            device="cpu", dtype=torch.float64)

    # Set up PyBullet
    robot_id, joint_indices = setup_pybullet(urdf_path, gui=gui)
    if verbose:
        print("[pybullet_test] PyBullet ready; iiwa loaded with %d joints"
              % len(joint_indices))

    # Set null_q as the initial config (same as Gazebo demo's warmup pose).
    for j, q0 in zip(joint_indices, INITIAL_Q):
        p.resetJointState(robot_id, j, q0)

    initial_q_t = torch.tensor(INITIAL_Q, dtype=torch.float64).unsqueeze(0)
    ee_pos_home, ee_quat_home = kin.fk(initial_q_t)
    if verbose:
        print("[pybullet_test] home EE pos  = %s"
              % np.array2string(ee_pos_home[0].cpu().numpy(), precision=4))
        print("[pybullet_test] home EE quat = %s"
              % np.array2string(ee_quat_home[0].cpu().numpy(), precision=4))

    dt = 1.0 / SIM_HZ
    n_steps = int(duration / dt)

    # Pacing for the GUI: walk wall-clock alongside sim time so the window
    # animates at ~1× real-time.  In headless mode we just run as fast as
    # possible (the OS won't care).
    wall_start = time.time()

    log = []
    for step in range(n_steps):
        t = step * dt

        q, dq = read_joint_state(robot_id, joint_indices)
        q_t   = torch.from_numpy(q ).unsqueeze(0)            # (1, 7)
        dq_t  = torch.from_numpy(dq).unsqueeze(0)

        # ====================================================================
        # External FK + Jacobian — this is what Isaac Lab's articulation API
        # gives you.  AttractorDSController never sees the kin provider.
        # ====================================================================
        ee_pos, ee_quat = kin.fk(q_t)
        J_pos, J_ang    = kin.jacobian(q_t)
        # ====================================================================

        seg = current_segment(t)
        if seg is None:
            # warmup: just hold (no torque) so the robot settles in initial pose
            p.stepSimulation()
            continue

        attractor_pos_np = np.asarray(seg.pos,  dtype=np.float64)
        att_pos_t  = torch.tensor([seg.pos],  dtype=torch.float64)
        att_quat_t = torch.tensor([seg.quat], dtype=torch.float64)
        K_t = torch.tensor([seg.K_linear],  dtype=torch.float64)
        a_t = torch.tensor([seg.damping_a], dtype=torch.float64)
        b_t = torch.tensor([seg.damping_b], dtype=torch.float64)

        # ====================================================================
        # The crucial call — pure Python interface.  No ROS, no Gazebo, no
        # pytorch_kinematics inside the controller (we built it with
        # kinematics=None).
        # ====================================================================
        out = ctrl.compute_from_state(
            q=q_t, dq=dq_t,
            ee_pos=ee_pos, ee_quat=ee_quat,
            J_pos=J_pos, J_ang=J_ang,
            attractor_pos=att_pos_t, attractor_quat=att_quat_t,
            K_linear=K_t, damping_a=a_t, damping_b=b_t,
        )
        tau = out["tau"][0].cpu().numpy()

        apply_torque(robot_id, joint_indices, tau)

        log.append({
            "t": t,
            "ee_pos": ee_pos[0].cpu().numpy().copy(),
            "att_pos": attractor_pos_np.copy(),
            "seg_idx": SCHEDULE.index(seg),
            "tau_norm": float(np.linalg.norm(tau)),
            "v_des": out["v_des"][0].cpu().numpy().copy(),
        })

        # Once per simulated second, print a heartbeat so we can see progress
        if verbose and (step % SIM_HZ) == 0:
            print("  t=%5.2fs  ee=%s  att=%s  err=%.3f  tau_norm=%.2f"
                  % (t,
                     np.array2string(ee_pos[0].cpu().numpy(), precision=3),
                     np.array2string(attractor_pos_np, precision=3),
                     float(np.linalg.norm(ee_pos[0].cpu().numpy() - attractor_pos_np)),
                     float(np.linalg.norm(tau))))

        p.stepSimulation()

        if gui:
            # Sleep so wall-clock catches up to sim time.  No-op when behind.
            sleep_for = (wall_start + t + dt) - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

    p.disconnect()

    # Per-segment pass criterion: best EE-attractor distance during the
    # segment < 5 cm.  In practice all 5 segments converge AND hold (best ≈
    # final), so passing on "best" is equivalent to passing on "final" — the
    # weaker formulation just leaves room for a tracker-overshoots-and-comes-
    # back trajectory to still pass cleanly.
    per_seg = []
    all_pass = True
    if verbose:
        print("[pybullet_test] per-segment best position errors during segment:")
    for k, seg in enumerate(SCHEDULE):
        seg_log = [r for r in log if r["seg_idx"] == k]
        if not seg_log:
            continue
        errs   = np.array([np.linalg.norm(r["ee_pos"] - r["att_pos"]) for r in seg_log])
        best   = float(errs.min())
        final  = float(errs[-1])
        ok = bool(best < PASS_TOL_M)
        all_pass = all_pass and ok
        per_seg.append({
            "seg": k, "label": seg.label,
            "best_pos_err_m":  best,
            "final_pos_err_m": final,
            "tau_norm_mean":   float(np.mean([r["tau_norm"] for r in seg_log])),
            "ok": ok,
        })
        if verbose:
            print("  seg %d (%s): best=%.4f m  final=%.4f m  [%s]"
                  % (k, seg.label, best, final, "PASS" if ok else "FAIL"))
    if verbose:
        print("[pybullet_test] OVERALL: %s" % ("PASS" if all_pass else "FAIL"))
        print("                (PASS = best EE-attractor distance < %.0f cm "
              "during every segment.)"
              % (PASS_TOL_M * 100))

    if plot_dir:
        save_plots(log, per_seg, plot_dir, verbose=verbose)

    return all_pass, per_seg


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf",     type=str, default="",
                    help="path to iiwa URDF.  Default = PyBullet's bundled "
                         "kuka_iiwa/model.urdf (recommended — has joint damping "
                         "and inertia tuned for the PyBullet integrator).")
    ap.add_argument("--duration", type=float, default=DURATION)
    ap.add_argument("--gui",      action="store_true",
                    help="open the PyBullet GUI window (needs DISPLAY).  "
                         "Without this flag the sim runs headless (DIRECT).")
    ap.add_argument("--plot-dir", type=str, default="",
                    help="write EE trajectory / error / tau-norm PNG plots "
                         "into this directory.  Off by default; the shell "
                         "wrapper sets it to the test run directory.")
    args = ap.parse_args()

    urdf_path = args.urdf or bundled_iiwa_urdf()
    print(f"[pybullet_test] using URDF: {urdf_path}")
    if args.gui:
        print("[pybullet_test] GUI mode — opening PyBullet window")
    if args.plot_dir:
        print(f"[pybullet_test] plots will be written to {args.plot_dir}")
    ok, _ = run_test(urdf_path, duration=args.duration, verbose=True,
                     gui=args.gui, plot_dir=args.plot_dir)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
