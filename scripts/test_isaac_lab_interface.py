#!/usr/bin/env python3
"""Verify the Isaac-Lab-facing pure-Python interface drives Gazebo correctly.

Setup that proves the point:
  * The controller is constructed with ``kinematics=None`` — i.e. the
    AttractorDSController is told it has NO kin provider, matching how Isaac
    Lab would build it (since Isaac's articulation API gives FK + Jacobian
    directly).
  * FK + Jacobian come from a SEPARATE TorchKinematicsPK instance that lives
    in this test script.  Functionally this is what Isaac Lab does when it
    calls ``articulation.data.body_pos_w`` + ``get_jacobians()`` — supplies
    state to the controller from the simulator side.
  * The controller is driven via ``compute_from_state(...)`` ONLY.  Never
    ``compute()``.  If this drives Gazebo to the attractor cleanly, the
    Isaac Lab path is verified end to end.

Usage (inside docker, after `roslaunch iiwa_gazebo iiwa_gazebo.launch`):

    rosrun iiwa_toolkit test_isaac_lab_interface.py [--duration 40]

For a one-shot end-to-end test, use ``scripts/test_isaac_lab_interface.sh``
which boots Gazebo + this script together and reports PASS/FAIL.
"""

import argparse
import datetime as _dt
import os
import sys
import threading
import time

import numpy as np
import torch

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "src"))
from iiwa_passive_ds import PassiveDSCore, AttractorDSController         # noqa: E402
from iiwa_passive_ds.kinematics import TorchKinematicsPK                 # noqa: E402


# ---------------------------------------------------------------------------
# Tiny 3-attractor schedule.  Different from ds_planner.py's so the test
# stays independent of that file's contents.
SCHEDULE = [
    # (t_start_s, attractor_pos[3], attractor_quat[wxyz], K_linear, damping_a, damping_b, label)
    ( 6.0, [0.65,  0.10, 0.55], [0.7071068, 0.0, 0.7071068, 0.0], 0.5, 1.0, 1.0, "A"),
    (18.0, [0.55, -0.10, 0.40], [0.7071068, 0.0, 0.7071068, 0.0], 0.5, 1.0, 1.0, "B"),
    (30.0, [0.50,  0.00, 0.60], [0.7071068, 0.0, 0.7071068, 0.0], 0.8, 1.5, 1.5, "C — bigger K + stiffer damping"),
]


# ---------------------------------------------------------------------------
class IsaacLabPathTest:
    """Single-env Gazebo driver that exercises compute_from_state only."""

    JOINT_NAMES = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
                   "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]

    def __init__(self, duration: float, log_csv: str, rate_hz: float = 500.0):
        self.duration = float(duration)
        self.rate_hz  = float(rate_hz)
        self.log_csv  = log_csv

        urdf_string = rospy.get_param("/iiwa/robot_description")
        rospy.loginfo("[isaac_test] building kinematics chain (cpu, float64)")
        self.kin = TorchKinematicsPK(urdf_string=urdf_string,
                                     ee_link="iiwa_link_ee", root_link="iiwa_link_0",
                                     device="cpu", dtype=torch.float64)

        # Build the controller WITHOUT a kinematics provider — this is the
        # Isaac Lab construction path.  If anybody downstream calls
        # ctrl.compute() it will raise; compute_from_state() must be used.
        null_q     = torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0], dtype=torch.float64)
        null_gains = torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0], dtype=torch.float64)
        core = PassiveDSCore(
            lam0_pos=100.0, lam1_pos=100.0,
            lam0_ori=5.0,   lam1_ori=5.0,
            ds_gain_pos=5.0, ds_gain_ori=2.5,
            max_dx=0.10, max_dq=0.20,
            null_q=null_q, null_gains=null_gains,
            null_damping=1.0, null_scale=10.0,
            enable_init_phase=False,                # Isaac-Lab style
        )
        self.ctrl = AttractorDSController(core, kinematics=None, velocity_limit=0.15)
        rospy.loginfo("[isaac_test] controller built with kinematics=None  "
                      "velocity_limit=%.3f", self.ctrl.velocity_limit)

        # robot state buffer
        self.joint_name2idx = {n: i for i, n in enumerate(self.JOINT_NAMES)}
        self.q  = np.zeros(7, dtype=np.float64)
        self.dq = np.zeros(7, dtype=np.float64)
        self.q_received = False
        self.lock = threading.Lock()

        rospy.Subscriber("/iiwa/joint_states", JointState, self._on_joints, queue_size=1)
        self.pub_torque = rospy.Publisher("/iiwa/TorqueController/command",
                                          Float64MultiArray, queue_size=1)

        self._log_rows = []
        self._log_lock = threading.Lock()
        rospy.on_shutdown(self._flush_log)

    # ----------------------------------------------------------------
    def _on_joints(self, msg: JointState):
        with self.lock:
            for name, p, v in zip(msg.name, msg.position, msg.velocity):
                idx = self.joint_name2idx.get(name)
                if idx is not None:
                    self.q[idx]  = p
                    self.dq[idx] = v
            self.q_received = True

    # ----------------------------------------------------------------
    def _current_seg(self, t_elapsed: float):
        seg = None
        for entry in SCHEDULE:
            if t_elapsed >= entry[0]:
                seg = entry
        return seg

    # ----------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        # wait for the first joint_states message so the controller has q
        rospy.loginfo("[isaac_test] waiting for /iiwa/joint_states ...")
        while not rospy.is_shutdown() and not self.q_received:
            rate.sleep()
        rospy.loginfo("[isaac_test] got joint_states; running for %.1fs",
                      self.duration)

        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t_elapsed = now - t_start
            if t_elapsed > self.duration:
                break

            seg = self._current_seg(t_elapsed)
            if seg is None:
                # warmup → hold at current pose: trivial attractor = ee_pos
                self._hold_in_place()
            else:
                _, att_pos, att_quat, K, a, b, _label = seg
                self._tick(t_elapsed, att_pos, att_quat, K, a, b)
            rate.sleep()

        rospy.loginfo("[isaac_test] schedule done; verifying convergence")
        return self._verify()

    # ----------------------------------------------------------------
    def _hold_in_place(self):
        with self.lock:
            if not self.q_received:
                return
            q  = self.q.copy()
            dq = self.dq.copy()
        q_t  = torch.from_numpy(q ).unsqueeze(0)   # (1,7)
        dq_t = torch.from_numpy(dq).unsqueeze(0)
        # Compute FK externally (this is what Isaac Lab would do via articulation API)
        ee_pos, ee_quat = self.kin.fk(q_t)
        # Hold at current pose → attractor = current EE, K=0.5
        att_pos  = ee_pos.clone()
        att_quat = ee_quat.clone()
        # Skip publishing torque during warmup (let Gazebo gravity-compensate
        # via the ros_control plugin defaults).  This mirrors Isaac Lab's
        # behaviour when no action has been chosen yet.

    # ----------------------------------------------------------------
    def _tick(self, t_elapsed: float, att_pos_list, att_quat_list, K, a, b):
        with self.lock:
            if not self.q_received:
                return
            q  = self.q.copy()
            dq = self.dq.copy()

        device = "cpu"
        dtype = torch.float64
        q_t   = torch.from_numpy(q ).to(device).unsqueeze(0)
        dq_t  = torch.from_numpy(dq).to(device).unsqueeze(0)

        # ====================================================================
        # The crucial bit: FK + Jacobian computed OUTSIDE the controller
        # (this is exactly what Isaac Lab's articulation API gives you).
        # ====================================================================
        ee_pos, ee_quat = self.kin.fk(q_t)
        J_pos, J_ang    = self.kin.jacobian(q_t)
        # ====================================================================

        att_pos_t  = torch.tensor([att_pos_list ], dtype=dtype, device=device)
        att_quat_t = torch.tensor([att_quat_list], dtype=dtype, device=device)
        K_t = torch.tensor([K], dtype=dtype, device=device)
        a_t = torch.tensor([a], dtype=dtype, device=device)
        b_t = torch.tensor([b], dtype=dtype, device=device)

        # ====================================================================
        # Pure Isaac-Lab path: compute_from_state with pre-computed FK + J.
        # Controller was built with kinematics=None, so this is the ONLY
        # way to invoke it.
        # ====================================================================
        out = self.ctrl.compute_from_state(
            q=q_t, dq=dq_t,
            ee_pos=ee_pos, ee_quat=ee_quat,
            J_pos=J_pos, J_ang=J_ang,
            attractor_pos=att_pos_t, attractor_quat=att_quat_t,
            K_linear=K_t, damping_a=a_t, damping_b=b_t,
        )
        tau       = out["tau"][0].cpu().numpy()
        v_des     = out["v_des"][0].cpu().numpy()
        v_des_raw = out["v_des_raw"][0].cpu().numpy()
        ee_pos_np = ee_pos[0].cpu().numpy()
        ee_vel_np = out["ee_vel"][0].cpu().numpy()

        self.pub_torque.publish(Float64MultiArray(data=tau.tolist()))

        with self._log_lock:
            self._log_rows.append((
                t_elapsed,
                *ee_pos_np, *ee_vel_np,
                *att_pos_list, *v_des, *v_des_raw,
                *tau,
                float(K), float(a), float(b),
            ))

    # ----------------------------------------------------------------
    def _verify(self):
        """Per-segment final-position-error check.  PASS if all < 5cm."""
        if not self._log_rows:
            rospy.logerr("[isaac_test] no log rows — nothing to verify")
            return False

        data = np.array(self._log_rows, dtype=np.float64)
        t_col = data[:, 0]
        ee    = data[:, 1:4]
        att   = data[:, 7:10]
        err   = np.linalg.norm(ee - att, axis=1)

        boundaries = [s[0] for s in SCHEDULE] + [t_col[-1] + 1e6]
        all_pass = True
        rospy.loginfo("[isaac_test] per-segment final position errors:")
        for k, (t_s, _, _, _, _, _, label) in enumerate(SCHEDULE):
            mask = (t_col >= t_s) & (t_col < boundaries[k + 1])
            if not mask.any():
                continue
            i_end = np.nonzero(mask)[0][-1]
            final = float(err[i_end])
            ok = final < 0.05
            all_pass = all_pass and ok
            rospy.loginfo("  seg %d (%s): final pos err = %.4f m  [%s]",
                          k, label, final, "PASS" if ok else "FAIL")
        rospy.loginfo("[isaac_test] OVERALL: %s", "PASS" if all_pass else "FAIL")
        return all_pass

    # ----------------------------------------------------------------
    def _flush_log(self):
        with self._log_lock:
            rows = self._log_rows
        if not rows:
            return
        header = (["t"] + ["ee_pos_%s" % c for c in "xyz"]
                  + ["ee_vel_%s" % c for c in "xyz"]
                  + ["att_pos_%s" % c for c in "xyz"]
                  + ["v_des_%s"   % c for c in "xyz"]
                  + ["v_des_raw_%s" % c for c in "xyz"]
                  + ["tau%d" % i for i in range(7)]
                  + ["K_linear", "damping_a", "damping_b"])
        try:
            os.makedirs(os.path.dirname(self.log_csv) or ".", exist_ok=True)
            with open(self.log_csv, "w") as fh:
                fh.write(",".join(header) + "\n")
                for r in rows:
                    fh.write(",".join(repr(x) for x in r) + "\n")
            rospy.loginfo("[isaac_test] wrote %d rows to %s",
                          len(rows), self.log_csv)
        except Exception as e:
            rospy.logerr("[isaac_test] log write failed: %s", e)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=42.0)
    ap.add_argument("--rate-hz",  type=float, default=500.0)
    ap.add_argument("--log-csv",  type=str,   default="")
    args = ap.parse_args()

    rospy.init_node("test_isaac_lab_interface", anonymous=False)

    log_csv = args.log_csv or (
        "/tmp/test_isaac_lab_interface_%s.csv"
        % _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"))

    test = IsaacLabPathTest(duration=args.duration, log_csv=log_csv,
                            rate_hz=args.rate_hz)
    ok = test.run()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
