"""Sketch — using PassiveDSCore in Isaac Lab for RL policy training.

NOT a runnable script. Drop the relevant pieces into your IsaacLab task
script (e.g. an env or a custom action term).

The key insight is that PassiveDSCore.compute() takes already-resolved EE
state and Jacobians as tensors. Isaac Lab's articulation API gives you
those for free, batched over envs, on GPU. So no FK/Jacobian library is
needed; you just feed Isaac's outputs into the core.

Repo layout assumed:  the iiwa_passive_ds package is on PYTHONPATH (e.g.
    sys.path.insert(0, "<repo>/src/iiwa_toolkit/src")
).
"""

from __future__ import annotations

import torch

from iiwa_passive_ds.core import PassiveDSCore


# ---------------------------------------------------------------------------
def build_controller(num_envs: int, device: str = "cuda") -> PassiveDSCore:
    """Construct one PassiveDSCore that batched-controls every env.

    For Isaac Lab RL we typically want the C++ "first / coarse positioning"
    phase OFF — the env is reset to a sensible initial pose every episode and
    the policy should drive the arm directly.
    """
    core = PassiveDSCore(
        lam0_pos=100.0, lam1_pos=100.0,
        lam0_ori=5.0,   lam1_ori=5.0,
        ds_gain_pos=5.0, ds_gain_ori=2.5,
        max_dx=0.10, max_dq=0.20,
        null_damping=1.0, null_scale=10.0,
        enable_init_phase=False,
    )
    return core


# ---------------------------------------------------------------------------
def step_example(core: PassiveDSCore, robot, action: torch.Tensor, default_quat: torch.Tensor):
    """One control step inside an Isaac Lab task.

    Args:
        robot          : an Isaac Lab Articulation (or whatever your task uses).
        action         : (B, 6)  — policy output.  Convention here:
                            action[:, 0:3] = ee_des_pos              (m)
                            action[:, 3]   = damping scale a         (lam0)
                            action[:, 4]   = damping scale b         (lam1)
                            action[:, 5]   = unused (placeholder for future)
                          Adapt to your action space; you can also feed a quat.
        default_quat   : (B, 4) — quaternion to track when the policy doesn't
                                  command one (e.g. tool-vertical pose).

    Returns:
        tau_full : (B, n_total_dofs) — torque command in Isaac Lab's joint
                   ordering. The slice for the iiwa joints comes from
                   ``robot.find_joints(...)``.
    """
    # --- read the iiwa joint subset out of Isaac's articulation -----------
    iiwa_joint_ids, _ = robot.find_joints(
        ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
         "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
    )
    q  = robot.data.joint_pos[:, iiwa_joint_ids]   # (B, 7)
    dq = robot.data.joint_vel[:, iiwa_joint_ids]

    # --- read EE state in the body's local frame --------------------------
    # (replace with your task's ee body, e.g. body_names=["iiwa_link_ee"])
    ee_body_idx, _ = robot.find_bodies("iiwa_link_ee")
    ee_pos  = robot.data.body_pos_w[:, ee_body_idx[0], :]                # (B, 3)
    ee_quat = robot.data.body_quat_w[:, ee_body_idx[0], :]               # (B, 4)  [w,x,y,z]
    ee_vel  = robot.data.body_lin_vel_w[:, ee_body_idx[0], :]
    ee_avel = robot.data.body_ang_vel_w[:, ee_body_idx[0], :]

    # --- get the EE Jacobian from Isaac (split linear / angular) ---------
    # NOTE: confirm the row ordering against your Isaac Lab version.  Both
    # IsaacLab >=0.2 and Isaac Sim's underlying articulation typically return
    # the geometric Jacobian as (B, 6, ndofs) with rows 0:3 linear, 3:6 angular.
    J = robot.root_physx_view.get_jacobians()                            # (B, n_bodies, 6, n_dofs)
    # pull the row for the EE body and the columns for the iiwa dofs:
    J_ee = J[:, ee_body_idx[0], :, iiwa_joint_ids]                       # (B, 6, 7)
    J_pos = J_ee[..., 0:3, :]
    J_ang = J_ee[..., 3:6, :]

    # --- pack the policy action into core inputs --------------------------
    ee_des_pos  = action[..., 0:3]
    a = action[..., 3]
    b = action[..., 4]

    # --- step the controller ---------------------------------------------
    tau_iiwa = core.compute(
        q=q, dq=dq,
        ee_pos=ee_pos, ee_quat=ee_quat,
        ee_vel=ee_vel, ee_angVel=ee_avel,
        J_pos=J_pos, J_ang=J_ang,
        ee_des_pos=ee_des_pos,
        ee_des_quat=default_quat,
        ee_des_vel=None,         # outer DS will derive ee_des_vel from pos error
        a=a, b=b,
    )                                                                     # (B, 7)

    # scatter into Isaac Lab's full torque vector if you have other joints
    tau_full = torch.zeros_like(robot.data.joint_pos)
    tau_full[:, iiwa_joint_ids] = tau_iiwa
    return tau_full


# ---------------------------------------------------------------------------
def reset_example(core: PassiveDSCore, env_ids: torch.Tensor):
    """Call this from your task's _reset_idx if you re-enable init_phase."""
    if not core.enable_init_phase:
        return
    mask = torch.zeros(core._first.shape[0] if core._first is not None else 0,
                       dtype=torch.bool, device=env_ids.device)
    mask[env_ids] = True
    core.reset_init_phase(mask)
