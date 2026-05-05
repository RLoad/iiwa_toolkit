#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Online comparison metrics for DS + iiwa_toolkit experiments.

Subscribes to desired/measured twist and measured wrench; streams a CSV and writes a
summary TXT on shutdown with scalars for plotting across methods.

Metric definitions (operational — tune thresholds via ROS params):
  max_velocity_error: max ||v_des - v_meas|| (linear part of Twist).
  max_force_error: max ||f_meas - f_ref|| with f_ref from ~desired_force [fx,fy,fz].
  worst_case_tracking_error: max over time of
      sqrt(w_v * ||e_v||^2 + w_f * ||e_f||^2) with w_v + w_f = 1.
  disturbance_recovery_time: if combined error crosses ~disturbance_spike_threshold,
      time until it stays below ~recovery_threshold for ~recovery_hold_s (first episode).
  settling_time: time from ~settling_start_delay_s until ||e_v|| stays below
      ~settling_epsilon_v for ~settling_hold_s (first episode).
  overshoot_velocity_error: max(0, max_velocity_error - steady_state_velocity_error_mean).
  steady_state_velocity_error: mean ||e_v|| over last ~steady_state_window_s.
  steady_state_force_error: mean ||e_f|| over last ~steady_state_window_s.
  normalized_performance_score: weighted sum of min(1, metric/scale) using *_scale params.

Outputs (in ~output_dir):
  metrics_timeseries_<method>_<stamp>.csv
  metrics_summary_<method>_<stamp>.txt
"""
from __future__ import print_function

import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import Twist, WrenchStamped


def _twist_lin(tw):
    return (tw.linear.x, tw.linear.y, tw.linear.z)


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class ComparisonMetricsLogger(object):
    def __init__(self):
        rospy.init_node("comparison_metrics_logger", anonymous=True)

        self.method_name = rospy.get_param("~method_name", "method")
        out = rospy.get_param("~output_dir", "/tmp/iiwa_compare_metrics")
        self.output_dir = os.path.expanduser(out)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        self.des_twist_topic = rospy.get_param("~desired_twist_topic", "/ds1/desired_velocity")
        self.meas_twist_topic = rospy.get_param("~measured_twist_topic", "/iiwa/ee_info/Vel")
        self.meas_wrench_topic = rospy.get_param("~measured_wrench_topic", "/iiwa/iiwa_FTS_topic")

        df = rospy.get_param("~desired_force", [0.0, 0.0, -12.0])
        self.f_ref = (float(df[0]), float(df[1]), float(df[2]))

        self.w_v = float(rospy.get_param("~velocity_error_weight", 0.5))
        self.w_f = float(rospy.get_param("~force_error_weight", 0.5))
        s = self.w_v + self.w_f
        if s > 1e-9:
            self.w_v /= s
            self.w_f /= s

        self.steady_window = float(rospy.get_param("~steady_state_window_s", 5.0))
        self.settling_eps_v = float(rospy.get_param("~settling_epsilon_v", 0.01))
        self.settling_hold = float(rospy.get_param("~settling_hold_s", 1.0))
        self.settling_start_delay = float(rospy.get_param("~settling_start_delay_s", 2.0))

        self.dist_spike = float(rospy.get_param("~disturbance_spike_threshold", 0.15))
        self.recovery_thresh = float(rospy.get_param("~recovery_threshold", 0.05))
        self.recovery_hold = float(rospy.get_param("~recovery_hold_s", 0.5))

        self.norm_v_scale = float(rospy.get_param("~norm_velocity_scale", 0.2))
        self.norm_f_scale = float(rospy.get_param("~norm_force_scale", 20.0))
        self.norm_wc_scale = float(rospy.get_param("~norm_worst_case_scale", 0.2))
        self.norm_settle_scale = float(rospy.get_param("~norm_settling_scale", 30.0))
        self.norm_rec_scale = float(rospy.get_param("~norm_recovery_scale", 10.0))

        self._des_v = None
        self._meas_v = None
        self._meas_f = None
        self._have_wrench = False

        self._rows = []  # (t_ros, ev, ef, ewc)

        # Extrema
        self.max_ev = 0.0
        self.max_ef = 0.0
        self.max_wc = 0.0

        # Settling state (time from arm to sustained low velocity error)
        self._settle_arm_time = None
        self._below_since = None
        self.settling_time = float("nan")

        # Disturbance recovery
        self._dist_active = False
        self._dist_t_start = None
        self._rec_ok_since = None
        self.recovery_time = float("nan")

        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = "metrics_{}_{}".format(self.method_name, stamp)
        self.csv_path = os.path.join(self.output_dir, base + "_timeseries.csv")
        self.summary_path = os.path.join(self.output_dir, base + "_summary.txt")

        self._csv_file = open(self.csv_path, "w")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["ros_time", "ev", "ef", "ewc"])

        rospy.Subscriber(self.des_twist_topic, Twist, self._cb_des, queue_size=1)
        rospy.Subscriber(self.meas_twist_topic, Twist, self._cb_meas, queue_size=1)
        rospy.Subscriber(self.meas_wrench_topic, WrenchStamped, self._cb_wrench, queue_size=1)

        self._timer = rospy.Timer(rospy.Duration(0.01), self._tick)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo("comparison_metrics_logger: CSV -> %s", self.csv_path)

    def _cb_des(self, msg):
        self._des_v = _twist_lin(msg)

    def _cb_meas(self, msg):
        self._meas_v = _twist_lin(msg)

    def _cb_wrench(self, msg):
        self._have_wrench = True
        w = msg.wrench
        self._meas_f = (w.force.x, w.force.y, w.force.z)

    def _tick(self, _evt):
        if self._des_v is None or self._meas_v is None:
            return
        now = rospy.Time.now().to_sec()

        ev = _norm3(_sub3(self._des_v, self._meas_v))
        if self._meas_f is not None:
            ef = _norm3(_sub3(self._meas_f, self.f_ref))
        else:
            ef = 0.0

        ewc = math.sqrt(self.w_v * ev * ev + self.w_f * ef * ef)

        self.max_ev = max(self.max_ev, ev)
        self.max_ef = max(self.max_ef, ef)
        self.max_wc = max(self.max_wc, ewc)

        self._rows.append((now, ev, ef, ewc))
        self._csv.writerow(["{:.6f}".format(now), "{:.8f}".format(ev), "{:.8f}".format(ef), "{:.8f}".format(ewc)])
        if len(self._rows) % 200 == 0:
            self._csv_file.flush()

        # Settling (velocity only): arm after delay, then time until sustained low error
        t0_run = self._rows[0][0]
        if self._settle_arm_time is None and now - t0_run >= self.settling_start_delay:
            self._settle_arm_time = now
        if self._settle_arm_time is not None and math.isnan(self.settling_time):
            if ev < self.settling_eps_v:
                if self._below_since is None:
                    self._below_since = now
                elif now - self._below_since >= self.settling_hold:
                    self.settling_time = self._below_since - self._settle_arm_time
            else:
                self._below_since = None

        # Disturbance recovery on worst-case error
        if ewc > self.dist_spike:
            if not self._dist_active:
                self._dist_active = True
                self._dist_t_start = now
                self._rec_ok_since = None
        if self._dist_active and math.isnan(self.recovery_time):
            if ewc < self.recovery_thresh:
                if self._rec_ok_since is None:
                    self._rec_ok_since = now
                elif now - self._rec_ok_since >= self.recovery_hold:
                    self.recovery_time = self._rec_ok_since - self._dist_t_start
            else:
                self._rec_ok_since = None

    def _steady_means(self):
        if not self._rows:
            return float("nan"), float("nan")
        t_end = self._rows[-1][0]
        t_cut = t_end - self.steady_window
        acc_v = []
        acc_f = []
        for t, ev, ef, _ in self._rows:
            if t >= t_cut:
                acc_v.append(ev)
                acc_f.append(ef)
        if not acc_v:
            return float("nan"), float("nan")
        return sum(acc_v) / len(acc_v), sum(acc_f) / len(acc_f)

    def _normalized_score(self):
        m = [
            min(1.0, self.max_ev / max(self.norm_v_scale, 1e-9)),
            min(1.0, self.max_ef / max(self.norm_f_scale, 1e-9)),
            min(1.0, self.max_wc / max(self.norm_wc_scale, 1e-9)),
        ]
        if not math.isnan(self.settling_time) and self.settling_time >= 0:
            m.append(min(1.0, self.settling_time / max(self.norm_settle_scale, 1e-9)))
        else:
            m.append(1.0)
        if not math.isnan(self.recovery_time) and self.recovery_time >= 0:
            m.append(min(1.0, self.recovery_time / max(self.norm_rec_scale, 1e-9)))
        else:
            m.append(0.0)
        return sum(m) / len(m)

    def _shutdown(self):
        try:
            self._timer.shutdown()
        except Exception:
            pass
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None

        ss_ev, ss_ef = self._steady_means()
        nscore = self._normalized_score()
        overshoot_v = max(0.0, self.max_ev - ss_ev) if not math.isnan(ss_ev) else float("nan")

        lines = [
            "method_name: {}".format(self.method_name),
            "desired_twist_topic: {}".format(self.des_twist_topic),
            "measured_twist_topic: {}".format(self.meas_twist_topic),
            "measured_wrench_topic: {}".format(self.meas_wrench_topic),
            "desired_force_ref: {} {} {}".format(self.f_ref[0], self.f_ref[1], self.f_ref[2]),
            "wrench_messages_received: {}".format(str(self._have_wrench)),
            "duration_s: {:.6f}".format(self._rows[-1][0] - self._rows[0][0]) if len(self._rows) > 1 else "duration_s: 0",
            "max_velocity_error: {:.8f}".format(self.max_ev),
            "max_force_error: {:.8f}".format(self.max_ef),
            "worst_case_tracking_error: {:.8f}".format(self.max_wc),
            "disturbance_recovery_time_s: {}".format(
                "{:.6f}".format(self.recovery_time) if not math.isnan(self.recovery_time) else "nan"
            ),
            "settling_time_s: {}".format(
                "{:.6f}".format(self.settling_time) if not math.isnan(self.settling_time) else "nan"
            ),
            "overshoot_velocity_error: {:.8f}".format(overshoot_v),
            "steady_state_velocity_error_mean: {:.8f}".format(ss_ev),
            "steady_state_force_error_mean: {:.8f}".format(ss_ef),
            "normalized_performance_score: {:.8f}".format(nscore),
            "timeseries_csv: {}".format(self.csv_path),
        ]
        with open(self.summary_path, "w") as f:
            f.write("\n".join(lines) + "\n")
        rospy.loginfo("comparison_metrics_logger: summary -> %s", self.summary_path)


def main():
    ComparisonMetricsLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
