"""Self-contained attractor DS + passive DS controller for RL use.

This wraps the existing ``PassiveDSCore`` (inner anisotropic damping + null-space
PD + init-phase) with a *linear attractor DS* on the outer loop, so RL doesn't
need a separate planner.  All knobs that an RL policy would want to tune are
exposed per-step as batched tensors:

  attractor_pos    (B, 3)         — desired Cartesian attractor
  attractor_quat   (B, 4)         — desired orientation [w, x, y, z]
  K_linear         (B,)           — outer DS gain:  v_des = -K_linear * (x - x_a)
  damping_a        (B,)           — inner passive DS λ0 scale  (lam0_eff = a*lam0_base)
  damping_b        (B,)           — inner passive DS λ1 scale  (lam1_eff = b*lam1_base)

Outputs a dict so the policy can use the intermediate state for observations,
reward shaping, or debug logging.

Matches the C++ RL_controller phase-2 logic with these explicit choices:

  * Linear attractor DS:      v_raw = -K_linear * (x - x_a)
  * Velocity-norm clamp:      ‖v‖ ≤ velocity_limit              (C++ Velocity_limit_=0.15 m/s)
  * NO force feedback         (force lives in the RL cost, not the control loop)
  * NO low-pass filter        (the C++ "filtered" variable was misnamed —
                               it's only the norm clamp, no time history)
  * Orientation reference     attractor_quat ; the outer-quat DS inside
                              PassiveDSCore handles the angular tracking.

Tensor / dtype conventions match PassiveDSCore (see core.py docstring).
"""

from __future__ import annotations

import torch

from .core import PassiveDSCore


class AttractorDSController:
    """Attractor DS + passive DS, batchable, no ROS, no sim assumptions.

    Construct ONCE with a kinematics provider and a PassiveDSCore.  Call
    ``compute(...)`` per step with the robot state + RL-tunable knobs.

    All tensor inputs to ``compute`` may live on CPU or CUDA; the controller
    will follow ``q``'s device/dtype.  ``ee_force`` is not accepted — force is
    expected to be used in the RL reward, outside this control loop.
    """

    def __init__(self,
                 core: PassiveDSCore,
                 kinematics,
                 velocity_limit: float = 0.15):
        """
        Args:
          core             : PassiveDSCore instance (gains, null-space, init phase).
          kinematics       : object exposing fk(q)->(ee_pos, ee_quat),
                             jacobian(q)->(J_pos, J_ang).
                             See kinematics.py for the conventions.
          velocity_limit   : max ‖v_des‖ before passing to passive DS.  Mirrors
                             C++ test_ori_control::Velocity_limit_ (0.15 m/s).
                             A scalar — kept fixed at construct time on purpose
                             (it's a system / safety bound, not an RL knob).
        """
        self.core = core
        self.kin  = kinematics
        self.velocity_limit = float(velocity_limit)

    # ----------------------------------------------------------------
    @staticmethod
    def _clamp_norm(v: torch.Tensor, max_n: float, eps: float = 1e-9) -> torch.Tensor:
        """Saturate ‖v‖ ≤ max_n along the last dim.  Mirrors C++ clamp."""
        n = torch.linalg.vector_norm(v, dim=-1, keepdim=True)
        scale = torch.where(n > max_n, max_n / n.clamp_min(eps), torch.ones_like(n))
        return v * scale

    # ----------------------------------------------------------------
    def reset_init_phase(self, mask: torch.Tensor | None = None) -> None:
        """Re-arm the init-phase latch for the given envs.  Pass a (B,) bool
        tensor with True at indices that just got reset; pass None to re-arm
        every env.  Forwards to PassiveDSCore.
        """
        self.core.reset_init_phase(mask)

    # ----------------------------------------------------------------
    def compute(self,
                q: torch.Tensor,
                dq: torch.Tensor,
                attractor_pos: torch.Tensor,
                attractor_quat: torch.Tensor,
                K_linear: torch.Tensor,
                damping_a: torch.Tensor,
                damping_b: torch.Tensor) -> dict:
        """One control step.

        Args (all batched along leading B dim; dtype/device take from q):
          q                : (B, 7)  joint position
          dq               : (B, 7)  joint velocity
          attractor_pos    : (B, 3)  outer DS attractor (Cartesian)
          attractor_quat   : (B, 4)  desired EE orientation [w, x, y, z]
          K_linear         : (B,)    outer DS gain
          damping_a        : (B,)    inner passive DS λ0 scale
          damping_b        : (B,)    inner passive DS λ1 scale

        Returns dict:
          tau         : (B, 7)  joint torque command  (primary output)
          v_des       : (B, 3)  velocity reference sent to passive DS (clamped)
          v_des_raw   : (B, 3)  velocity reference before norm clamp
          ee_pos      : (B, 3)  measured EE position  (via FK)
          ee_quat     : (B, 4)  measured EE quat      (via FK)
          ee_vel      : (B, 3)  measured EE linear velocity  (J_pos @ dq)
          ee_angVel   : (B, 3)  measured EE angular velocity (J_ang @ dq)
          first_flag  : (B,) bool, or None — True iff still in init phase
                        (only present if PassiveDSCore.enable_init_phase=True)
        """
        # ---- FK + Jacobian (once; reused for control and observation) ----
        ee_pos, ee_quat = self.kin.fk(q)
        J_pos, J_ang    = self.kin.jacobian(q)
        ee_vel    = (J_pos @ dq.unsqueeze(-1)).squeeze(-1)
        ee_angVel = (J_ang @ dq.unsqueeze(-1)).squeeze(-1)

        # ---- outer linear attractor DS ----
        # v_raw = -K_linear * (x - x_a)
        v_des_raw = -K_linear[..., None] * (ee_pos - attractor_pos)

        # ---- velocity-norm saturation (C++ Velocity_limit_) ----
        v_des = self._clamp_norm(v_des_raw, self.velocity_limit)

        # ---- inner passive DS (re-use PassiveDSCore VEL mode) ----
        tau = self.core.compute(
            q=q, dq=dq,
            ee_pos=ee_pos, ee_quat=ee_quat,
            ee_vel=ee_vel, ee_angVel=ee_angVel,
            J_pos=J_pos, J_ang=J_ang,
            ee_des_pos=None,                   # VEL mode: outer-pos DS bypassed
            ee_des_quat=attractor_quat,
            ee_des_vel=v_des,
            a=damping_a, b=damping_b,
        )

        # ---- assemble observation dict ----
        first_flag = None
        if self.core._first is not None:
            first_flag = self.core._first.clone()

        return {
            "tau":        tau,
            "v_des":      v_des,
            "v_des_raw":  v_des_raw,
            "ee_pos":     ee_pos,
            "ee_quat":    ee_quat,
            "ee_vel":     ee_vel,
            "ee_angVel":  ee_angVel,
            "first_flag": first_flag,
        }
