"""Pure-Python passive DS controller for iiwa.

A single self-contained Python controller — no external planner, no multi-phase
state machine.  One Python ROS node (node_attractor_ds_gazebo.py) runs the same
code path that Isaac Lab calls directly; the only difference is whether
attractor / gains come from ROS topics or from the RL policy.

Algorithm = one DS phase, three layers (all batched, GPU-friendly):

   outer  : linear attractor DS    v_des = -K_linear * (x - x_attractor)
            + velocity-norm saturation ‖v‖ ≤ velocity_limit
   inner  : anisotropic passive DS (mirrors C++ passive_control.cpp)
   joint  : null-space PD pulling joints toward null_q

Package layout:
  core.py         — PassiveDSCore: inner passive DS + null-space PD + (optional)
                    coarse-positioning init phase.  Pure algorithm — no FK, no
                    ROS, no sim.  Inputs are tensors of robot/EE state.
  kinematics.py   — TorchKinematicsPK: pytorch_kinematics-based FK + Jacobian
                    provider for stand-alone use (Gazebo).  Isaac Lab can use
                    it too, or substitute its own articulation-API adapter.
  attractor_ds.py — AttractorDSController: outer attractor DS + core + kin
                    combined into one batched compute().  The main entry point
                    for both Gazebo (via node_attractor_ds_gazebo.py) and
                    Isaac Lab (direct import).
"""

from .core import PassiveDS, PassiveDSCore  # noqa: F401
from .attractor_ds import AttractorDSController  # noqa: F401

__all__ = ["PassiveDS", "PassiveDSCore", "AttractorDSController"]
