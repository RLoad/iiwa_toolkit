"""Kinematics providers for PassiveDSCore.

Two reasonable backends are supported, but you can plug in your own — the
controller only needs an object exposing ``fk(q)`` and ``jacobian(q)`` with
the conventions documented below.

  TorchKinematicsPK   — pytorch_kinematics (works on cpu and cuda, batched).
                        Recommended for stand-alone Gazebo use and as a
                        fallback in Isaac Lab.
  IsaacLabKinematics  — thin adapter for Isaac Lab's articulation API; do NOT
                        instantiate at import time, see example_isaac_lab.py.

Conventions returned by ``fk(q)`` and ``jacobian(q)``:
  ee_pos    : (B, 3)
  ee_quat   : (B, 4)  [w, x, y, z]
  J_pos     : (B, 3, n_joints)  linear  Jacobian (∂x/∂q)
  J_ang     : (B, 3, n_joints)  angular Jacobian (∂ω/∂q)
"""

from __future__ import annotations

import torch


class TorchKinematicsPK:
    """pytorch_kinematics-based FK + Jacobian for a fixed serial chain (e.g. iiwa7).

    Imports ``pytorch_kinematics`` lazily so this module loads even without it
    installed (Isaac Lab side may use a different provider).
    """

    def __init__(self, urdf_string: str | bytes, ee_link: str, root_link: str | None = None,
                 device: str = "cpu", dtype: torch.dtype = torch.float32):
        # pytorch_kinematics imports matplotlib unconditionally (in ik.py),
        # which defaults to the tkagg backend and pulls in tkinter — not
        # installed in the project's docker container, and pointless for a
        # headless controller anyway. Force a non-GUI backend before the import.
        import os as _os
        _os.environ.setdefault("MPLBACKEND", "Agg")
        try:
            import pytorch_kinematics as pk
        except ImportError as e:
            raise ImportError(
                "pytorch_kinematics is required for TorchKinematicsPK.\n"
                "  pip install pytorch_kinematics"
            ) from e

        urdf = urdf_string.encode() if isinstance(urdf_string, str) else urdf_string
        self._chain = pk.build_serial_chain_from_urdf(urdf, ee_link, root_link or "")
        self._chain = self._chain.to(dtype=dtype, device=device)
        self.device = device
        self.dtype = dtype
        self.n_joints = len(self._chain.get_joint_parameter_names())

    # ---- FK --------------------------------------------------------------
    def fk(self, q: torch.Tensor):
        """q: (B, n).  Returns ee_pos (B,3), ee_quat (B,4 wxyz)."""
        ret = self._chain.forward_kinematics(q, end_only=True)
        # pytorch_kinematics returns a Transform3d — extract translation + quaternion
        T = ret.get_matrix()                 # (B, 4, 4)
        ee_pos = T[..., :3, 3]               # (B, 3)
        # rotation matrix → quaternion (wxyz).  Use a stable conversion that
        # matches Eigen::Quaternion(matrix3d) sign convention.
        ee_quat = _rotmat_to_quat_wxyz(T[..., :3, :3])
        return ee_pos, ee_quat

    # ---- Jacobian -------------------------------------------------------
    def jacobian(self, q: torch.Tensor):
        """q: (B, n).  Returns J_pos (B,3,n), J_ang (B,3,n).

        pytorch_kinematics returns the geometric Jacobian as (B, 6, n) with
        the *first three rows linear, last three angular*.  We split here so
        callers don't need to know.
        """
        J = self._chain.jacobian(q)          # (B, 6, n)
        return J[..., 0:3, :], J[..., 3:6, :]


# ---------------------------------------------------------------------------
def _rotmat_to_quat_wxyz(R: torch.Tensor) -> torch.Tensor:
    """Batched rotation-matrix → quaternion (w, x, y, z), Eigen-compatible sign.

    Uses Sheppard's stable method (largest diagonal trace path).  R: (B, 3, 3).
    """
    m = R
    t = m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2]      # trace, (B,)
    eps = 1e-12

    # Path 1: trace positive
    s1 = torch.sqrt((t + 1.0).clamp_min(eps)) * 2.0     # = 4w
    w1 = 0.25 * s1
    x1 = (m[..., 2, 1] - m[..., 1, 2]) / s1
    y1 = (m[..., 0, 2] - m[..., 2, 0]) / s1
    z1 = (m[..., 1, 0] - m[..., 0, 1]) / s1

    # Path 2: m00 largest
    s2 = torch.sqrt((1.0 + m[..., 0, 0] - m[..., 1, 1] - m[..., 2, 2]).clamp_min(eps)) * 2.0
    w2 = (m[..., 2, 1] - m[..., 1, 2]) / s2
    x2 = 0.25 * s2
    y2 = (m[..., 0, 1] + m[..., 1, 0]) / s2
    z2 = (m[..., 0, 2] + m[..., 2, 0]) / s2

    # Path 3: m11 largest
    s3 = torch.sqrt((1.0 + m[..., 1, 1] - m[..., 0, 0] - m[..., 2, 2]).clamp_min(eps)) * 2.0
    w3 = (m[..., 0, 2] - m[..., 2, 0]) / s3
    x3 = (m[..., 0, 1] + m[..., 1, 0]) / s3
    y3 = 0.25 * s3
    z3 = (m[..., 1, 2] + m[..., 2, 1]) / s3

    # Path 4: m22 largest
    s4 = torch.sqrt((1.0 + m[..., 2, 2] - m[..., 0, 0] - m[..., 1, 1]).clamp_min(eps)) * 2.0
    w4 = (m[..., 1, 0] - m[..., 0, 1]) / s4
    x4 = (m[..., 0, 2] + m[..., 2, 0]) / s4
    y4 = (m[..., 1, 2] + m[..., 2, 1]) / s4
    z4 = 0.25 * s4

    # Pick path per-element
    cond1 = t > 0
    cond2 = (m[..., 0, 0] >= m[..., 1, 1]) & (m[..., 0, 0] >= m[..., 2, 2])
    cond3 = m[..., 1, 1] >= m[..., 2, 2]

    w = torch.where(cond1, w1, torch.where(cond2, w2, torch.where(cond3, w3, w4)))
    x = torch.where(cond1, x1, torch.where(cond2, x2, torch.where(cond3, x3, x4)))
    y = torch.where(cond1, y1, torch.where(cond2, y2, torch.where(cond3, y3, y4)))
    z = torch.where(cond1, z1, torch.where(cond2, z2, torch.where(cond3, z3, z4)))

    q = torch.stack((w, x, y, z), dim=-1)
    # Make w >= 0 (Eigen convention picks the canonical hemisphere)
    q = torch.where(q[..., 0:1] < 0, -q, q)
    return q
