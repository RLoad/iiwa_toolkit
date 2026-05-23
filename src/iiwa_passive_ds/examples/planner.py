"""Open-loop "planner" — supplies AttractorDSController's RL-tunable knobs.

In a real RL setup the policy network's action tensor IS the planner: every
step, the policy outputs ``attractor_pos``, ``attractor_quat``, ``K_linear``,
``damping_a``, ``damping_b`` and the env feeds them into
``ctrl.compute_from_state(...)``.

For an offline demo (Gazebo, PyBullet, or any deterministic dry-run) we
substitute a hand-written **time-indexed schedule** for the policy.  This
file is exactly that substitute: a list of segments, each a constant
attractor pose + gain triple, switched at fixed times.

When you port to Isaac Lab, you DELETE this file and let the policy output
those tensors directly.  Nothing else changes.

Five segments, identical to ``iiwa_toolkit/src/ds_planner.py`` (the ROS
node version of this same planner) — same world-frame attractors, same
``[K_linear, damping_a, damping_b]`` per segment.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


# Tool z along world +x (90° rotation about y), as wxyz.  Shared by every
# segment in this schedule — orientation is constant, only position + gains
# change.  Matches the C++ phase-2 desired quat in task_space_control.cpp.
TOOL_Z_ALONG_X = [0.7071068, 0.0, 0.7071068, 0.0]


@dataclass
class Segment:
    """One open-loop "policy action" held constant for ``t_start`` → next."""
    t_start:   float        # seconds (sim time) at which this segment starts
    pos:       List[float]  # (3,) world-frame attractor position
    quat:      List[float]  # (4,) wxyz attractor orientation
    K_linear:  float        # outer DS gain
    damping_a: float        # inner DS λ0 multiplier
    damping_b: float        # inner DS λ1 multiplier
    label:     str          # human-readable name (for plots / logs)


# Five demo segments, 12 s each, after a 5 s warmup.  Match ds_planner.py.
WARMUP     = 5.0
SEG_DUR    = 12.0
SCHEDULE: List[Segment] = [
    Segment(WARMUP + 0 * SEG_DUR, [0.7,  0.0, 0.6], TOOL_Z_ALONG_X, 0.5, 1.0, 1.0, "A — front high      (K=0.5)"),
    Segment(WARMUP + 1 * SEG_DUR, [0.7, -0.2, 0.4], TOOL_Z_ALONG_X, 0.5, 1.0, 1.0, "B — front-right low (K=0.5)"),
    Segment(WARMUP + 2 * SEG_DUR, [0.4,  0.2, 0.6], TOOL_Z_ALONG_X, 0.8, 1.0, 1.0, "C — back-left high  (K=0.8)"),
    Segment(WARMUP + 3 * SEG_DUR, [0.5,  0.0, 0.4], TOOL_Z_ALONG_X, 0.5, 0.5, 0.5, "D — center low      (soft damping)"),
    Segment(WARMUP + 4 * SEG_DUR, [0.5,  0.0, 0.6], TOOL_Z_ALONG_X, 0.5, 2.0, 2.0, "E — center          (stiff damping)"),
]
TOTAL_DURATION = WARMUP + len(SCHEDULE) * SEG_DUR + 1.0   # +1 s tail


def current_segment(t_elapsed: float, schedule: List[Segment] = SCHEDULE) -> Optional[Segment]:
    """Return the latest segment whose ``t_start`` ≤ ``t_elapsed``, else None.

    None means we're in the warmup window before segment 0 starts — caller
    should hold zero torque so the robot settles at its initial pose.
    """
    chosen: Optional[Segment] = None
    for s in schedule:
        if t_elapsed >= s.t_start:
            chosen = s
    return chosen
