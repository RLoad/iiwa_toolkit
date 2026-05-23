#!/usr/bin/env python3
"""DS_planner — a tiny attractor + damping-gain publisher.

Pretends to be an Isaac-Lab RL policy but lives in ROS: walks the controller
through a hand-picked schedule of attractor poses and damping gains so we can
watch the Python passive DS + attractor DS controller respond.

Topics (published, latched):
  /attractor_ds_py/attractor   geometry_msgs/Pose
  /attractor_ds_py/gains       std_msgs/Float64MultiArray  data = [K_linear, a, b]

Usage:
    rosrun iiwa_toolkit ds_planner.py [--n-attractors 5] [--seg-duration 12]
                                       [--warmup 5]      [--schedule-out FILE]
                                       [--shutdown-master]

CLI flags:
  --n-attractors      how many segments from the built-in list (max = len(SCHEDULE))
  --seg-duration      seconds to hold each attractor before stepping to the next
  --warmup            seconds to wait after node start before publishing seg 0
                      (gives Gazebo + controller time to settle at init pose)
  --schedule-out      tab-separated dump of the schedule (used by the plotter
                      to annotate phase boundaries with red dotted lines)
  --shutdown-master   call rospy.signal_shutdown after the final segment.  In
                      the demo orchestrator we usually let the bash wrapper
                      SIGINT roslaunch instead, so this is off by default.

The schedule mixes attractor moves and gain changes so plots show both axes
of RL tunability:
"""

import argparse
import os
import sys
import time

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose


# Tool-z-along-world-+x (90° about y) — matches C++ phase-2 desired quat in
# task_space_control.cpp.  All 5 segments share the same orientation; only
# position + gains change.
_TOOL_Z_ALONG_X = [0.7071068, 0.0, 0.7071068, 0.0]   # [w, x, y, z]

# (attractor_pos[3], attractor_quat[wxyz], gains[K_linear, damping_a, damping_b], label)
SCHEDULE = [
    ([0.7,  0.0,  0.6], _TOOL_Z_ALONG_X, [0.5, 1.0, 1.0], "front high      (K=0.5)"),
    ([0.7, -0.2,  0.4], _TOOL_Z_ALONG_X, [0.5, 1.0, 1.0], "front-right low (K=0.5)"),
    ([0.4,  0.2,  0.6], _TOOL_Z_ALONG_X, [0.8, 1.0, 1.0], "back-left high  (K=0.8)"),
    ([0.5,  0.0,  0.4], _TOOL_Z_ALONG_X, [0.5, 0.5, 0.5], "center low      (soft  damping a=b=0.5)"),
    ([0.5,  0.0,  0.6], _TOOL_Z_ALONG_X, [0.5, 2.0, 2.0], "center          (stiff damping a=b=2.0)"),
]


# ---------------------------------------------------------------------------
def make_pose(pos, quat_wxyz):
    m = Pose()
    m.position.x, m.position.y, m.position.z = (float(pos[0]), float(pos[1]), float(pos[2]))
    m.orientation.w = float(quat_wxyz[0])
    m.orientation.x = float(quat_wxyz[1])
    m.orientation.y = float(quat_wxyz[2])
    m.orientation.z = float(quat_wxyz[3])
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-attractors", type=int, default=len(SCHEDULE),
                    help="how many schedule segments to play (max %d)" % len(SCHEDULE))
    ap.add_argument("--seg-duration", type=float, default=12.0,
                    help="seconds to hold each attractor before stepping to the next")
    ap.add_argument("--warmup", type=float, default=5.0,
                    help="seconds to wait after node start before publishing seg 0 "
                         "(lets the robot settle at the controller's init pose)")
    ap.add_argument("--schedule-out", type=str, default="",
                    help="if given, write a TSV of (t_rel_s, pos, quat, gains, label)")
    ap.add_argument("--shutdown-master", action="store_true",
                    help="call rospy.signal_shutdown after the final segment")
    args = ap.parse_args()

    n = min(args.n_attractors, len(SCHEDULE))
    if n <= 0:
        print("[ds_planner] --n-attractors must be >= 1", file=sys.stderr); sys.exit(2)
    schedule = SCHEDULE[:n]

    rospy.init_node("ds_planner", anonymous=True, disable_signals=False)
    pub_pose  = rospy.Publisher("/attractor_ds_py/attractor", Pose,
                                queue_size=1, latch=True)
    pub_gains = rospy.Publisher("/attractor_ds_py/gains", Float64MultiArray,
                                queue_size=1, latch=True)

    # Wait for subscribers (so the latched first publish actually lands).
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if pub_pose.get_num_connections() > 0 and pub_gains.get_num_connections() > 0:
            break
        time.sleep(0.05)
    rospy.loginfo("[ds_planner] subscribers: pose=%d gains=%d",
                  pub_pose.get_num_connections(), pub_gains.get_num_connections())

    # Build the per-segment timing (relative to node start, after warmup).
    seg_start_rel = [args.warmup + k * args.seg_duration for k in range(n)]
    total_end_rel = args.warmup + n * args.seg_duration

    if args.schedule_out:
        try:
            with open(args.schedule_out, "w") as fh:
                fh.write("# t_rel_s\tpos_x\tpos_y\tpos_z\tquat_w\tquat_x\tquat_y\tquat_z"
                         "\tK_linear\tdamping_a\tdamping_b\tlabel\n")
                for k, (p, q, g, lbl) in enumerate(schedule):
                    fh.write("%.3f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%s\n"
                             % (seg_start_rel[k],
                                p[0], p[1], p[2],
                                q[0], q[1], q[2], q[3],
                                g[0], g[1], g[2], lbl))
        except OSError as e:
            rospy.logwarn("[ds_planner] failed to write schedule: %s", e)

    rospy.loginfo("[ds_planner] %d segments, warmup=%.1fs, seg=%.1fs, total=%.1fs",
                  n, args.warmup, args.seg_duration, total_end_rel)

    rate = rospy.Rate(50.0)
    t0 = time.time()
    current_seg = -1   # -1 → haven't started seg 0 yet (still in warmup)

    while not rospy.is_shutdown():
        elapsed = time.time() - t0
        if elapsed >= total_end_rel:
            break

        # Decide which segment we should be on
        seg = -1
        for k in range(n):
            if elapsed >= seg_start_rel[k]:
                seg = k
        # else: still warming up; don't publish (the controller's yaml has its
        # own init pose latched as the default attractor — robot will hold).

        if seg >= 0:
            pos, quat, gains, label = schedule[seg]
            if seg != current_seg:
                rospy.loginfo("[ds_planner] t=%6.2fs  seg %d → %s  pos=%s  K/a/b=%s",
                              elapsed, seg, label, pos, gains)
                current_seg = seg
            # Re-publish at 50 Hz so attractor_age stays small (defensive even
            # though the topic is latched).
            pub_pose.publish(make_pose(pos, quat))
            msg = Float64MultiArray(); msg.data = list(gains)
            pub_gains.publish(msg)
        rate.sleep()

    rospy.loginfo("[ds_planner] schedule complete (%.1fs)", time.time() - t0)
    if args.shutdown_master:
        rospy.signal_shutdown("ds_planner schedule complete")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
