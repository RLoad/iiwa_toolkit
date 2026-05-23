"""Isaac Lab wiring example for AttractorDSController.

This file is documentation-grade Python: it shows the call structure for
wiring AttractorDSController into an Isaac Lab env, but does not actually
import isaaclab — the real env code lives in the user's Isaac Lab project.

Key points:
  * The controller is fully batched on the leading B (= num_envs) dim.
  * Skip the kinematics provider entirely — Isaac Lab's articulation API
    already exposes body_pos_w / body_quat_w / get_jacobians().  Feed those
    tensors straight into compute_from_state().
  * RL policy outputs become the controller's tunable inputs (attractor_pos,
    attractor_quat, K_linear, damping_a, damping_b).
  * Force / contact stays OUT of the control loop — it's read separately
    and used in the reward.

Two scenarios are illustrated:

  (A) build_controller_from_config(num_envs, device)
      One-time construction of PassiveDSCore + AttractorDSController.

  (B) env_step(...)
      A typical per-step call inside Isaac Lab's MyTaskEnv._apply_action()
      or similar — translate the policy's action tensor to a torque
      command via the controller.
"""
from __future__ import annotations

import torch

from iiwa_passive_ds import PassiveDSCore, AttractorDSController


# ---------------------------------------------------------------------------
# (A) one-time construction (env __init__)
# ---------------------------------------------------------------------------
def build_controller_from_config(num_envs: int, device: str = "cuda") -> AttractorDSController:
    """Construct the controller with iiwa14 defaults.  Call once in env init.

    PassiveDSCore parameters mirror C++ passive_control.cpp defaults — same
    inner-damping eigenvalues and null-space gains the real robot uses.  Tweak
    here at init time if your task needs different system constants; RL only
    tunes the per-step knobs.
    """
    dtype = torch.float32  # GPU-friendly; float64 also works
    null_q     = torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0], dtype=dtype, device=device)
    null_gains = torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0], dtype=dtype, device=device)

    core = PassiveDSCore(
        lam0_pos=100.0, lam1_pos=100.0,    # inner passive DS λ base values
        lam0_ori=5.0,   lam1_ori=5.0,
        ds_gain_pos=5.0,                    # unused: outer-pos DS is bypassed in attractor mode
        ds_gain_ori=2.5,
        max_dx=0.10, max_dq=0.20,           # saturation on outer-DS error vectors
        null_q=null_q,
        null_gains=null_gains,
        null_damping=1.0,
        null_scale=10.0,                    # τ = τ_task + null_scale * N * τ_null
        enable_init_phase=False,            # turn OFF for RL training — no latched state
    )
    return AttractorDSController(core, kinematics=None, velocity_limit=0.15)


# ---------------------------------------------------------------------------
# (B) per-env-step torque computation
# ---------------------------------------------------------------------------
def env_step(ctrl: AttractorDSController,
             articulation,                 # isaaclab.assets.Articulation, batched
             ee_body_idx: int,             # index of the iiwa_link_ee body in articulation
             action: torch.Tensor,         # (B, 10) RL policy output
             default_attractor_quat: torch.Tensor):  # (B, 4) if you keep quat fixed
    """Translate an action tensor into joint torques and apply them.

    `action` layout (one common choice; pick what suits your task):
        action[:, 0:3]   — attractor_pos   (Cartesian world frame)
        action[:, 3:7]   — attractor_quat  [w,x,y,z]   *or* skip these dims
                                                          if you want quat fixed
        action[:, 7]     — K_linear
        action[:, 8]     — damping_a
        action[:, 9]     — damping_b

    Returns the dict from compute_from_state() so you can pull intermediate
    state for observations / rewards.
    """
    q  = articulation.data.joint_pos                # (B, n_dof)
    dq = articulation.data.joint_vel                # (B, n_dof)

    # FK from Isaac Lab — body_*_w is in world frame, exactly what we want.
    ee_pos  = articulation.data.body_pos_w[:, ee_body_idx]   # (B, 3)
    ee_quat = articulation.data.body_quat_w[:, ee_body_idx]  # (B, 4) [w,x,y,z]

    # Jacobian — Isaac Lab's PhysX articulation view exposes a (B, n_bodies, 6, n_dof)
    # geometric Jacobian.  Linear rows first, angular rows after.
    J_full = articulation.root_physx_view.get_jacobians()[:, ee_body_idx]  # (B, 6, n_dof)
    J_pos, J_ang = J_full[:, 0:3, :], J_full[:, 3:6, :]

    # Slice the policy action
    attractor_pos  = action[:, 0:3]
    attractor_quat = default_attractor_quat        # or action[:, 3:7] if you let RL spin it
    K_linear       = action[:, 7]
    damping_a      = action[:, 8]
    damping_b      = action[:, 9]

    out = ctrl.compute_from_state(
        q=q, dq=dq,
        ee_pos=ee_pos, ee_quat=ee_quat,
        J_pos=J_pos, J_ang=J_ang,
        attractor_pos=attractor_pos, attractor_quat=attractor_quat,
        K_linear=K_linear, damping_a=damping_a, damping_b=damping_b,
    )

    articulation.set_joint_effort_target(out["tau"])
    return out                              # for obs / reward shaping


# ---------------------------------------------------------------------------
# (C) env reset hook — keeps init-phase latch consistent across resets
# ---------------------------------------------------------------------------
def on_env_reset(ctrl: AttractorDSController, reset_mask: torch.Tensor):
    """If enable_init_phase=True (it's off by default in build_controller_from_config
    above, but turn it on if you want C++-style coarse-positioning at reset),
    re-arm the init-phase latch for the envs that just reset.

    `reset_mask` is the (B,) bool tensor of envs that were reset this step.
    """
    ctrl.reset_init_phase(reset_mask)


# ---------------------------------------------------------------------------
# (D) reward / cost — what to do with the EE force, since it stays OUT of the
#     control loop
# ---------------------------------------------------------------------------
def compute_contact_reward(contact_sensor,           # isaaclab.sensors.ContactSensor
                           ee_body_idx: int,
                           controller_out: dict,
                           desired_force_mag: float = 10.0):
    """Sample reward using the EE contact force and the controller's debug fields.

    The controller is NOT given the force as input — that's intentional.  Force
    feedback lives at the RL level, where you can shape the reward to make the
    policy *learn* the right K_linear / damping_a / damping_b for the contact.
    """
    F_world = contact_sensor.data.net_forces_w[:, ee_body_idx, :]      # (B, 3)
    F_mag   = torch.linalg.vector_norm(F_world, dim=-1)                # (B,)

    # Encourage |F| → desired_force_mag while reaching the attractor
    force_err = (F_mag - desired_force_mag) ** 2                       # (B,)
    pos_err   = torch.linalg.vector_norm(
        controller_out["ee_pos"] - controller_out.get("attractor_pos", controller_out["ee_pos"]),
        dim=-1,
    )                                                                  # (B,)

    return -0.1 * force_err - 1.0 * pos_err                            # (B,)
