"""Pure-python passive DS algorithm (no kinematics, no ROS, no sim).

Mirrors iiwa_toolkit/src/passive_control.cpp. All ops are torch and support a
leading batch dimension B (broadcasting over scalars where reasonable). Pass
tensors with shape:
  q, dq          : (B, 7)
  ee_pos         : (B, 3)
  ee_quat        : (B, 4)   [w, x, y, z]
  ee_vel         : (B, 3)   linear  EE velocity
  ee_angVel      : (B, 3)   angular EE velocity
  J_pos, J_ang   : (B, 3, 7)
  ee_des_pos     : (B, 3)
  ee_des_quat    : (B, 4)   [w, x, y, z]
  a, b           : (B,)     damping eigenvalue scale factors
                            lam0_eff = a * lam0_base, lam1_eff = b * lam1_base

Output:
  tau            : (B, 7) joint torque command

Set B=1 for single-arm Gazebo use; the math degenerates cleanly.
"""

from __future__ import annotations

import torch


# ---------------------------------------------------------------------------
# Inner passive DS (one branch: position OR orientation).
# ---------------------------------------------------------------------------
class PassiveDS:
    """Anisotropic damping in task space, mirrors C++ PassiveDS.

    The C++ version constructs an orthonormal basis with Gram-Schmidt and
    forms D = B * diag(lam0, lam1, lam1) * B^T.  Because the second and third
    eigenvalues are identical, the result has a clean closed form:
        D = lam1 * I + (lam0 - lam1) * (u u^T),  where u = ref_vel / |ref_vel|.
    This form is exactly equivalent, deterministic, batchable, and
    differentiable (good for Isaac-Lab RL).  When |ref_vel| is near zero, the
    C++ falls back to D = I; we mimic that.
    """

    def __init__(self, lam0_base: float, lam1_base: float, eps: float = 1e-6):
        self.lam0_base = float(lam0_base)
        self.lam1_base = float(lam1_base)
        self.eps = eps

    def damping_matrix(self, ref_vel: torch.Tensor, a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
        """D(ref_vel; a, b)  →  (B, 3, 3)."""
        lam0 = a * self.lam0_base   # (B,)
        lam1 = b * self.lam1_base   # (B,)
        norm = torch.linalg.vector_norm(ref_vel, dim=-1, keepdim=True)            # (B,1)
        u    = ref_vel / norm.clamp_min(self.eps)                                 # (B,3)
        I3   = torch.eye(3, dtype=ref_vel.dtype, device=ref_vel.device)           # (3,3)
        uuT  = u.unsqueeze(-1) * u.unsqueeze(-2)                                  # (B,3,3)
        D    = lam1[..., None, None] * I3 + (lam0 - lam1)[..., None, None] * uuT  # (B,3,3)
        # Match C++: when |ref_vel| <= eps, fall back to I (no preferred axis).
        mask = (norm.squeeze(-1) > self.eps)[..., None, None]                     # (B,1,1)
        return torch.where(mask, D, I3.expand_as(D))

    def update(self, vel: torch.Tensor, des_vel: torch.Tensor,
               a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
        """C++ PassiveDS::update + get_output:  f = -D vel + lam0_eff des_vel."""
        D    = self.damping_matrix(des_vel, a, b)                                  # (B,3,3)
        lam0 = (a * self.lam0_base)[..., None]                                     # (B,1)
        return -(D @ vel.unsqueeze(-1)).squeeze(-1) + lam0 * des_vel               # (B,3)


# ---------------------------------------------------------------------------
# Quaternion utilities (convention: [w, x, y, z], C++ Eigen::Vector4d order).
# ---------------------------------------------------------------------------
def _q_normalize(q: torch.Tensor, eps: float = 1e-9) -> torch.Tensor:
    return q / torch.linalg.vector_norm(q, dim=-1, keepdim=True).clamp_min(eps)


def _q_conjugate(q: torch.Tensor) -> torch.Tensor:
    out = q.clone()
    out[..., 1:] = -out[..., 1:]
    return out


def _q_product(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """Hamilton product, [w,x,y,z] order.  Matches Utils<double>::quaternionProduct."""
    w1, x1, y1, z1 = q1.unbind(-1)
    w2, x2, y2, z2 = q2.unbind(-1)
    return torch.stack((
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ), dim=-1)


def _q_slerp_half(q0: torch.Tensor, q1: torch.Tensor, eps: float = 1e-6) -> torch.Tensor:
    """Slerp(q0, q1, 0.5) — matches Utils<double>::slerpQuaternion(_, _, 0.5).

    Handles the q1 = -q0 sign-flip case so we always interpolate the short way.
    """
    q0 = _q_normalize(q0)
    q1 = _q_normalize(q1)
    dot = (q0 * q1).sum(dim=-1, keepdim=True)
    q1  = torch.where(dot < 0.0, -q1, q1)
    return _q_normalize(0.5 * (q0 + q1), eps=eps)


# ---------------------------------------------------------------------------
# High-level controller (no kinematics — provider injects EE state + Jacobians).
# ---------------------------------------------------------------------------
class PassiveDSCore:
    """Mirrors PassiveControl::computeTorqueCmd — but stateless w.r.t. FK.

    The two `is_just_velocity` modes from the C++ are exposed via the
    `compute()` arguments:
      - if `ee_des_vel` is None, the outer position DS computes it from
        `ee_des_pos - ee_pos` (C++ pose mode, is_just_velocity=False).
      - if `ee_des_vel` is given, the outer DS is skipped (C++ velocity mode,
        is_just_velocity=True).  `ee_des_pos` may be None in that case.

    The orientation outer DS always computes ee_des_angVel from quat error
    (the C++ has no separate ang-velocity-only mode).
    """

    def __init__(self,
                 lam0_pos: float = 100.0, lam1_pos: float = 100.0,
                 lam0_ori: float = 5.0,   lam1_ori: float = 5.0,
                 ds_gain_pos: float = 5.0, ds_gain_ori: float = 2.5,
                 max_dx: float = 0.1, max_dq: float = 0.2,
                 null_q: torch.Tensor | None = None,
                 null_gains: torch.Tensor | None = None,
                 null_damping: float = 1.0,
                 null_scale: float = 10.0,
                 init_phase_norm: float = 1.5,
                 enable_init_phase: bool = True,
                 load_added: float = 0.0,
                 gravity_z: float = 9.8):
        # passive DS branches
        self.ds_pos = PassiveDS(lam0_pos, lam1_pos)
        self.ds_ori = PassiveDS(lam0_ori, lam1_ori)
        # outer-DS gains (C++ dsGain_pos, dsGain_ori)
        self.ds_gain_pos = float(ds_gain_pos)
        self.ds_gain_ori = float(ds_gain_ori)
        # saturation
        self.max_dx = float(max_dx)
        self.max_dq = float(max_dq)
        # null space
        default_null_q = torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0])
        default_null_gains = torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0])
        self.null_q = default_null_q if null_q is None else null_q
        self.null_gains = default_null_gains if null_gains is None else null_gains
        self.null_damping = float(null_damping)
        self.null_scale = float(null_scale)
        self.init_phase_norm = float(init_phase_norm)
        self.enable_init_phase = bool(enable_init_phase)
        # gravity comp on EE
        self.load_added = float(load_added)
        self.gravity_z = float(gravity_z)
        # per-instance "first" flag (per-env in batched mode); shape (B,) bool, or None
        self._first: torch.Tensor | None = None

    # ---- helpers ---------------------------------------------------------
    def _clamp_norm(self, v: torch.Tensor, max_n: float, eps: float = 1e-9) -> torch.Tensor:
        n = torch.linalg.vector_norm(v, dim=-1, keepdim=True)
        scale = torch.where(n > max_n, max_n / n.clamp_min(eps), torch.ones_like(n))
        return v * scale

    def _ensure_first_state(self, batch_size: int, device, dtype):
        if not self.enable_init_phase:
            self._first = None
            return
        if self._first is None or self._first.shape[0] != batch_size or self._first.device != device:
            self._first = torch.ones(batch_size, dtype=torch.bool, device=device)

    def reset_init_phase(self, mask: torch.Tensor | None = None):
        """Force the controller back into 'first' (coarse-positioning) mode.
        For Isaac Lab, call on env reset with mask = the env-reset boolean tensor.
        """
        if self._first is None:
            return
        if mask is None:
            self._first[:] = True
        else:
            self._first[mask] = True

    # ---- main step -------------------------------------------------------
    def compute(self,
                # robot state
                q: torch.Tensor, dq: torch.Tensor,
                ee_pos: torch.Tensor, ee_quat: torch.Tensor,
                ee_vel: torch.Tensor, ee_angVel: torch.Tensor,
                J_pos: torch.Tensor, J_ang: torch.Tensor,
                # commands
                ee_des_pos: torch.Tensor | None,
                ee_des_quat: torch.Tensor,
                ee_des_vel:  torch.Tensor | None,
                a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
        """Returns tau: (B, 7).  See module docstring for tensor shapes."""

        B = q.shape[0]
        device, dtype = q.device, q.dtype

        # ------------ outer DS — position branch -------------------------
        if ee_des_vel is None:
            assert ee_des_pos is not None, "ee_des_pos required when ee_des_vel is None"
            deltaX = self._clamp_norm(ee_des_pos - ee_pos, self.max_dx)              # (B,3)
            theta_g = (-0.5 / (4 * self.max_dx**2)) * (deltaX * deltaX).sum(dim=-1)  # (B,)
            ee_des_vel = self.ds_gain_pos * (1.0 + torch.exp(theta_g))[..., None] * deltaX

        # ------------ outer DS — orientation branch ----------------------
        # slerp halfway, then angular vel via quaternion product (C++ block lines 258–271)
        dqd     = _q_slerp_half(ee_quat, ee_des_quat)
        deltaQ  = dqd - ee_quat
        qconj   = _q_conjugate(ee_quat)
        tmp_ang_q = _q_product(deltaQ, qconj)                 # (B,4)
        tmp_ang   = tmp_ang_q[..., 1:]                        # imaginary part, (B,3)
        tmp_ang   = self._clamp_norm(tmp_ang, self.max_dq)
        theta_gq  = (-0.5 / (4 * self.max_dq**2)) * (tmp_ang * tmp_ang).sum(dim=-1)
        ee_des_angVel = 2.0 * self.ds_gain_ori * (1.0 + torch.exp(theta_gq))[..., None] * tmp_ang

        # ------------ inner passive DS — position ------------------------
        wrenchPos = self.ds_pos.update(ee_vel, ee_des_vel, a, b)                # (B,3)
        if self.load_added != 0.0:
            wrenchPos = wrenchPos.clone()
            wrenchPos[..., 2] = wrenchPos[..., 2] + self.load_added * self.gravity_z
        # τ_pos = J_pos^T · F_pos
        trq_pos = (J_pos.transpose(-2, -1) @ wrenchPos.unsqueeze(-1)).squeeze(-1)  # (B,7)

        # ------------ inner passive DS — orientation ---------------------
        wrenchAng = self.ds_ori.update(ee_angVel, ee_des_angVel, a, b)             # (B,3)
        trq_ang   = (J_ang.transpose(-2, -1) @ wrenchAng.unsqueeze(-1)).squeeze(-1) # (B,7)

        trq_task = trq_pos + trq_ang                                                # (B,7)

        # ------------ null-space joint PD --------------------------------
        # Build full Jacobian J = [J_pos; J_ang] in (linear, angular) row order
        # (the order doesn't affect the null-space projector).
        J_full = torch.cat((J_pos, J_ang), dim=-2)                                  # (B,6,7)
        JJT    = J_full @ J_full.transpose(-2, -1)                                  # (B,6,6)
        JJT_pinv = torch.linalg.pinv(JJT)                                           # (B,6,6)
        I7 = torch.eye(7, dtype=dtype, device=device)
        N  = I7 - J_full.transpose(-2, -1) @ JJT_pinv @ J_full                       # (B,7,7)

        null_q  = self.null_q.to(device=device, dtype=dtype)
        null_g  = self.null_gains.to(device=device, dtype=dtype)
        er_null = self._clamp_norm(q - null_q, 0.2)                                  # (B,7)
        trq_null = -null_g * er_null - self.null_damping * dq                        # (B,7)

        trq_full = trq_task + self.null_scale * (N @ trq_null.unsqueeze(-1)).squeeze(-1)

        # ------------ initialization phase (C++ "first" flag) ------------
        if self.enable_init_phase:
            self._ensure_first_state(B, device, dtype)
            er_norm = torch.linalg.vector_norm(q - null_q, dim=-1)                  # (B,) un-clamped
            # latch False as soon as we get close enough (C++: er_null.norm() < 1.5)
            self._first = self._first & ~(er_norm < self.init_phase_norm)
            return torch.where(self._first[..., None], trq_null, trq_full)
        else:
            return trq_full
