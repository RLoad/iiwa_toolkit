#!/usr/bin/env python3
"""ROS node — runs the AttractorDSController against Gazebo iiwa.

This is the Gazebo-side validation harness for the RL-facing controller in
iiwa_passive_ds/attractor_ds.py.  In Isaac Lab the controller is called
directly from the env.step(); here we wrap it with ROS topics so you can
drive the same code path with a planner-less ROS publisher.

Topics:
  in   /iiwa/joint_states                       sensor_msgs/JointState
  in   /attractor_ds_py/attractor               geometry_msgs/Pose
                                                  position    = attractor pos
                                                  orientation = attractor quat
  in   /attractor_ds_py/gains                   std_msgs/Float64MultiArray
                                                  data = [K_linear, damping_a, damping_b]

  out  /iiwa/TorqueController/command           std_msgs/Float64MultiArray (7)
  out  /iiwa/ee_info/Pose                       geometry_msgs/Pose
  out  /iiwa/ee_info/Vel                        geometry_msgs/Twist
  out  /attractor_ds_py/v_des                   geometry_msgs/Vector3
  out  /attractor_ds_py/v_des_raw               geometry_msgs/Vector3

CSV log columns (one row per control step, ~`rate_hz` Hz):
  t, q[0..6], dq[0..6],
  ee_pos[xyz], ee_quat[wxyz], ee_vel[xyz], ee_angVel[xyz],
  attractor_pos[xyz], attractor_quat[wxyz],
  v_des[xyz], v_des_raw[xyz],
  tau[0..6],
  K_linear, damping_a, damping_b,
  attractor_age_s, gains_age_s, first_flag
"""

import datetime as _dt
import os
import sys
import threading

import numpy as np
import torch

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Twist, Vector3

# Ensure the sibling package is importable when run via `rosrun`.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from iiwa_passive_ds.core import PassiveDSCore                  # noqa: E402
from iiwa_passive_ds.attractor_ds import AttractorDSController  # noqa: E402
from iiwa_passive_ds.kinematics import TorchKinematicsPK        # noqa: E402


# ---------------------------------------------------------------------------
class AttractorDSNode:

    def __init__(self):
        # ---- params ------------------------------------------------------
        self.rate_hz     = float(rospy.get_param("~rate_hz", 500.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        ee_link          = rospy.get_param("~ee_link",   "iiwa_link_ee")
        root_link        = rospy.get_param("~root_link", "iiwa_link_0")
        device           = rospy.get_param("~device",    "cpu")
        urdf_param       = rospy.get_param("~urdf_param", "robot_description")

        joint_names = rospy.get_param("~joint_names", [
            "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
            "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7",
        ])

        # PassiveDSCore base params (system constants — RL doesn't tune these)
        core_kwargs = dict(
            lam0_pos     = float(rospy.get_param("~lam0_pos", 100.0)),
            lam1_pos     = float(rospy.get_param("~lam1_pos", 100.0)),
            lam0_ori     = float(rospy.get_param("~lam0_ori",   5.0)),
            lam1_ori     = float(rospy.get_param("~lam1_ori",   5.0)),
            ds_gain_pos  = float(rospy.get_param("~ds_gain_pos", 5.0)),
            ds_gain_ori  = float(rospy.get_param("~ds_gain_ori", 2.5)),
            max_dx       = float(rospy.get_param("~max_dx", 0.10)),
            max_dq       = float(rospy.get_param("~max_dq", 0.20)),
            null_damping = float(rospy.get_param("~null_damping", 1.0)),
            null_scale   = float(rospy.get_param("~null_scale", 10.0)),
            init_phase_norm   = float(rospy.get_param("~init_phase_norm", 1.5)),
            enable_init_phase = bool (rospy.get_param("~enable_init_phase", True)),
            load_added   = float(rospy.get_param("~load_added", 0.0)),
        )
        null_q = rospy.get_param("~null_q",    [0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0])
        null_g = rospy.get_param("~null_gains",[5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0])

        # AttractorDSController-specific
        velocity_limit = float(rospy.get_param("~velocity_limit", 0.15))

        # RL-knob defaults (used until /attractor_ds_py/gains arrives)
        self.K_linear_def  = float(rospy.get_param("~default_K_linear",  0.5))
        self.damping_a_def = float(rospy.get_param("~default_damping_a", 1.0))
        self.damping_b_def = float(rospy.get_param("~default_damping_b", 1.0))

        # Default attractor (used until /attractor_ds_py/attractor arrives).
        # Per the design discussion this is REQUIRED — there is no "hold at
        # current EE" fallback.  Empty list / wrong length → node refuses to
        # publish torques.
        init_pos  = rospy.get_param("~default_attractor_pos",  [])
        init_quat = rospy.get_param("~default_attractor_quat", [])
        if not (isinstance(init_pos, (list, tuple)) and len(init_pos) == 3
                and isinstance(init_quat, (list, tuple)) and len(init_quat) == 4):
            rospy.logwarn("[attractor_ds_py] no default_attractor_pos/quat in yaml; "
                          "node will hold zero torque until /attractor_ds_py/attractor arrives")
            self.attractor_pos  = None
            self.attractor_quat = None
        else:
            self.attractor_pos  = np.asarray(init_pos,  dtype=np.float64)
            self.attractor_quat = np.asarray(init_quat, dtype=np.float64)
            rospy.loginfo("[attractor_ds_py] default attractor  pos=%s  quat=%s",
                          self.attractor_pos.tolist(), self.attractor_quat.tolist())

        # ---- kinematics --------------------------------------------------
        urdf_string = rospy.get_param(urdf_param)
        rospy.loginfo("[attractor_ds_py] building kinematics chain %s -> %s on %s",
                      root_link, ee_link, device)
        kin = TorchKinematicsPK(urdf_string=urdf_string, ee_link=ee_link,
                                root_link=root_link, device=device,
                                dtype=torch.float64)

        # ---- core + attractor controller --------------------------------
        core = PassiveDSCore(
            null_q     = torch.tensor(null_q, dtype=torch.float64),
            null_gains = torch.tensor(null_g, dtype=torch.float64),
            **core_kwargs,
        )
        self.controller = AttractorDSController(core, kin, velocity_limit=velocity_limit)
        self.device = device

        # ---- robot state buffer -----------------------------------------
        self.joint_names    = list(joint_names)
        self.joint_name2idx = {n: i for i, n in enumerate(self.joint_names)}
        self.q  = np.zeros(7, dtype=np.float64)
        self.dq = np.zeros(7, dtype=np.float64)
        self.q_received = False

        # ---- gains buffer (defaults applied until a gains msg arrives) --
        self.K_linear  = self.K_linear_def
        self.damping_a = self.damping_a_def
        self.damping_b = self.damping_b_def

        # ---- timestamps -------------------------------------------------
        self.last_attractor_t = -1.0
        self.last_gains_t     = -1.0

        self.lock = threading.Lock()

        # ---- ROS I/O ----------------------------------------------------
        rospy.Subscriber("/iiwa/joint_states", JointState,         self._on_joints,    queue_size=1)
        rospy.Subscriber("/attractor_ds_py/attractor", Pose,        self._on_attractor, queue_size=1)
        rospy.Subscriber("/attractor_ds_py/gains", Float64MultiArray, self._on_gains,   queue_size=1)

        self.pub_torque       = rospy.Publisher("/iiwa/TorqueController/command",
                                                Float64MultiArray, queue_size=1)
        self.pub_ee_info_pose = rospy.Publisher("/iiwa/ee_info/Pose", Pose,    queue_size=1)
        self.pub_ee_info_vel  = rospy.Publisher("/iiwa/ee_info/Vel",  Twist,   queue_size=1)
        self.pub_v_des        = rospy.Publisher("/attractor_ds_py/v_des",     Vector3, queue_size=1)
        self.pub_v_des_raw    = rospy.Publisher("/attractor_ds_py/v_des_raw", Vector3, queue_size=1)

        # ---- CSV log buffer ---------------------------------------------
        default_log = "/tmp/attractor_ds_py_log_%s.csv" % _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        self.log_path = rospy.get_param("~log_csv", default_log)
        self._log_rows = []
        self._log_lock = threading.Lock()
        rospy.on_shutdown(self._flush_log)
        rospy.loginfo("[attractor_ds_py] logging to %s", self.log_path)

        rospy.loginfo("[attractor_ds_py] ready, %.0f Hz, device=%s, velocity_limit=%.3f",
                      self.rate_hz, self.device, velocity_limit)

    # ----------------------------------------------------------------
    def _on_joints(self, msg: JointState):
        with self.lock:
            for name, p, v in zip(msg.name, msg.position, msg.velocity):
                idx = self.joint_name2idx.get(name)
                if idx is not None:
                    self.q[idx]  = p
                    self.dq[idx] = v
            self.q_received = True

    def _on_attractor(self, msg: Pose):
        with self.lock:
            self.attractor_pos = np.array(
                [msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
            self.attractor_quat = np.array(
                [msg.orientation.w, msg.orientation.x,
                 msg.orientation.y, msg.orientation.z], dtype=np.float64)
            self.last_attractor_t = rospy.Time.now().to_sec()
        rospy.loginfo_throttle(2.0,
            "[attractor_ds_py] attractor pos=%s quat=%s",
            self.attractor_pos.tolist(), self.attractor_quat.tolist())

    def _on_gains(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            rospy.logwarn_throttle(2.0,
                "[attractor_ds_py] /gains expects [K_linear, damping_a, damping_b]; "
                "got %d elements — ignored", len(msg.data))
            return
        with self.lock:
            self.K_linear  = float(msg.data[0])
            self.damping_a = float(msg.data[1])
            self.damping_b = float(msg.data[2])
            self.last_gains_t = rospy.Time.now().to_sec()
        rospy.loginfo_throttle(2.0,
            "[attractor_ds_py] gains K=%.3f a=%.3f b=%.3f",
            self.K_linear, self.damping_a, self.damping_b)

    # ---- main loop ---------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            tau = self._step()
            if tau is not None:
                self.pub_torque.publish(Float64MultiArray(data=tau.tolist()))
            rate.sleep()

    def _step(self):
        with self.lock:
            if not self.q_received:
                return None
            if self.attractor_pos is None or self.attractor_quat is None:
                # No attractor at all → publish nothing (safer than guessing).
                return None
            q  = self.q.copy()
            dq = self.dq.copy()
            attractor_pos  = self.attractor_pos.copy()
            attractor_quat = self.attractor_quat.copy()
            K  = self.K_linear
            a  = self.damping_a
            b  = self.damping_b
            now = rospy.Time.now().to_sec()
            attractor_age = (now - self.last_attractor_t) if self.last_attractor_t > 0 else -1.0
            gains_age     = (now - self.last_gains_t)     if self.last_gains_t     > 0 else -1.0

        device = self.device
        q_t              = torch.from_numpy(q ).to(device).unsqueeze(0)   # (1,7)
        dq_t             = torch.from_numpy(dq).to(device).unsqueeze(0)
        att_pos_t        = torch.from_numpy(attractor_pos ).to(device).unsqueeze(0)
        att_quat_t       = torch.from_numpy(attractor_quat).to(device).unsqueeze(0)
        K_t = torch.tensor([K], dtype=torch.float64, device=device)
        a_t = torch.tensor([a], dtype=torch.float64, device=device)
        b_t = torch.tensor([b], dtype=torch.float64, device=device)

        out = self.controller.compute(
            q=q_t, dq=dq_t,
            attractor_pos=att_pos_t, attractor_quat=att_quat_t,
            K_linear=K_t, damping_a=a_t, damping_b=b_t,
        )
        tau         = out["tau"][0].cpu().numpy()
        v_des       = out["v_des"][0].cpu().numpy()
        v_des_raw   = out["v_des_raw"][0].cpu().numpy()
        ee_pos      = out["ee_pos"][0].cpu().numpy()
        ee_quat     = out["ee_quat"][0].cpu().numpy()
        ee_vel      = out["ee_vel"][0].cpu().numpy()
        ee_angVel   = out["ee_angVel"][0].cpu().numpy()
        first_flag  = bool(out["first_flag"][0].item()) if out["first_flag"] is not None else False

        # publish observation + debug
        self._publish_ee_feedback(ee_pos, ee_quat, ee_vel, ee_angVel)
        self._publish_vec3(self.pub_v_des,     v_des)
        self._publish_vec3(self.pub_v_des_raw, v_des_raw)

        # buffer log row
        with self._log_lock:
            self._log_rows.append((
                now,
                *q, *dq,
                *ee_pos, *ee_quat, *ee_vel, *ee_angVel,
                *attractor_pos, *attractor_quat,
                *v_des, *v_des_raw,
                *tau,
                float(K), float(a), float(b),
                float(attractor_age), float(gains_age), int(first_flag),
            ))

        return tau

    # ----------------------------------------------------------------
    def _publish_ee_feedback(self, p, q, v, w):
        m = Pose()
        m.position.x, m.position.y, m.position.z = (float(p[0]), float(p[1]), float(p[2]))
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = \
            (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self.pub_ee_info_pose.publish(m)
        t = Twist()
        t.linear.x,  t.linear.y,  t.linear.z  = (float(v[0]), float(v[1]), float(v[2]))
        t.angular.x, t.angular.y, t.angular.z = (float(w[0]), float(w[1]), float(w[2]))
        self.pub_ee_info_vel.publish(t)

    @staticmethod
    def _publish_vec3(pub, v):
        msg = Vector3()
        msg.x, msg.y, msg.z = (float(v[0]), float(v[1]), float(v[2]))
        pub.publish(msg)

    # ---- log flush ---------------------------------------------------
    def _flush_log(self):
        with self._log_lock:
            rows = self._log_rows
            self._log_rows = []
        if not rows:
            rospy.logwarn("[attractor_ds_py] no log rows to write")
            return
        header = (
            ["t"]
            + ["q%d" % i for i in range(7)]
            + ["dq%d" % i for i in range(7)]
            + ["ee_pos_x", "ee_pos_y", "ee_pos_z"]
            + ["ee_quat_w", "ee_quat_x", "ee_quat_y", "ee_quat_z"]
            + ["ee_vel_x", "ee_vel_y", "ee_vel_z"]
            + ["ee_angVel_x", "ee_angVel_y", "ee_angVel_z"]
            + ["attractor_pos_x", "attractor_pos_y", "attractor_pos_z"]
            + ["attractor_quat_w", "attractor_quat_x", "attractor_quat_y", "attractor_quat_z"]
            + ["v_des_x", "v_des_y", "v_des_z"]
            + ["v_des_raw_x", "v_des_raw_y", "v_des_raw_z"]
            + ["tau%d" % i for i in range(7)]
            + ["K_linear", "damping_a", "damping_b"]
            + ["attractor_age_s", "gains_age_s", "first_flag"]
        )
        try:
            os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
            with open(self.log_path, "w") as fh:
                fh.write(",".join(header) + "\n")
                for r in rows:
                    fh.write(",".join(repr(x) for x in r) + "\n")
            rospy.loginfo("[attractor_ds_py] wrote %d rows to %s",
                          len(rows), self.log_path)
        except Exception as e:
            rospy.logerr("[attractor_ds_py] failed to write log: %s", e)


# ---------------------------------------------------------------------------
def main():
    rospy.init_node("attractor_ds_py", anonymous=False)
    node = AttractorDSNode()
    node.run()


if __name__ == "__main__":
    main()
