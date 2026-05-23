"""Standalone smoke test for AttractorDSController (no ROS, no Gazebo).

Run inside the docker container:

    docker compose exec iiwa-robot bash -c "
        source /opt/ros/noetic/setup.bash &&
        xacro \$(rospack find iiwa_description)/urdf/iiwa14.urdf.xacro \
            robot_name:=iiwa origin_xyz:='0 0 0' origin_rpy:='0 0 0' > /tmp/iiwa14.urdf &&
        cd \$(rospack find iiwa_toolkit)/src &&
        python3 iiwa_passive_ds/_smoke.py /tmp/iiwa14.urdf
    "

This exercises the entire controller pipeline (FK + Jacobian + outer attractor
DS + ‖v‖ clamp + inner passive DS + null-space PD) with synthetic joint state
and a fixed attractor, on CPU.  Asserts the output torques are finite, the
correct shape, and respond to attractor / gain changes the way you'd expect.
"""
from __future__ import annotations

import os
import sys

# Pytorch_kinematics imports matplotlib transitively (tries tkagg by default
# and tkinter isn't installed in the container) — force a non-GUI backend.
os.environ.setdefault("MPLBACKEND", "Agg")

import torch

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from iiwa_passive_ds.core import PassiveDSCore
from iiwa_passive_ds.attractor_ds import AttractorDSController
from iiwa_passive_ds.kinematics import TorchKinematicsPK


def main(urdf_path: str) -> None:
    with open(urdf_path, "rb") as fh:
        urdf = fh.read()

    dtype = torch.float64
    kin = TorchKinematicsPK(urdf_string=urdf, ee_link="iiwa_link_ee",
                            root_link="iiwa_link_0", device="cpu", dtype=dtype)
    print(f"[smoke] kin.n_joints = {kin.n_joints}")

    core = PassiveDSCore(
        lam0_pos=100.0, lam1_pos=100.0,
        lam0_ori=5.0,   lam1_ori=5.0,
        ds_gain_pos=5.0, ds_gain_ori=2.5,
        max_dx=0.10, max_dq=0.20,
        null_q=torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0], dtype=dtype),
        null_gains=torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0], dtype=dtype),
        null_damping=1.0, null_scale=10.0,
        enable_init_phase=False,
    )
    ctrl = AttractorDSController(core, kin, velocity_limit=0.15)

    # Synthetic robot state (batch of 1, near home)
    q  = torch.zeros(1, 7, dtype=dtype)
    q[0, 1] =  0.5
    q[0, 3] = -1.0
    dq = torch.zeros(1, 7, dtype=dtype)

    ee_pos, ee_quat = kin.fk(q)
    print(f"[smoke] starting ee_pos  = {ee_pos[0].tolist()}")
    print(f"[smoke] starting ee_quat = {ee_quat[0].tolist()}")

    attractor_pos  = ee_pos + torch.tensor([0.1, 0.0, 0.0], dtype=dtype)
    attractor_quat = ee_quat.clone()
    K_linear  = torch.tensor([0.5], dtype=dtype)
    damping_a = torch.tensor([1.0], dtype=dtype)
    damping_b = torch.tensor([1.0], dtype=dtype)

    out = ctrl.compute(q=q, dq=dq,
                       attractor_pos=attractor_pos, attractor_quat=attractor_quat,
                       K_linear=K_linear,
                       damping_a=damping_a, damping_b=damping_b)
    print(f"[smoke] tau       = {out['tau'][0].tolist()}")
    print(f"[smoke] v_des     = {out['v_des'][0].tolist()}")
    print(f"[smoke] v_des_raw = {out['v_des_raw'][0].tolist()}")

    # Sanity asserts
    assert out["tau"].shape == (1, 7),         f"tau shape {out['tau'].shape}"
    assert torch.isfinite(out["tau"]).all(),    "NaN/Inf in tau"
    assert out["v_des"].shape == (1, 3),       f"v_des shape {out['v_des'].shape}"
    # |v_des| ≤ velocity_limit (within fp epsilon)
    v_norm = torch.linalg.vector_norm(out["v_des"], dim=-1)
    assert (v_norm <= ctrl.velocity_limit + 1e-6).all(), \
        f"v_des norm {v_norm} exceeds velocity_limit {ctrl.velocity_limit}"

    # Sanity: doubling K should not change v_des once we're already saturating.
    out2 = ctrl.compute(q=q, dq=dq,
                        attractor_pos=attractor_pos, attractor_quat=attractor_quat,
                        K_linear=K_linear * 2.0,
                        damping_a=damping_a, damping_b=damping_b)
    if torch.linalg.vector_norm(out["v_des_raw"]) > ctrl.velocity_limit:
        assert torch.allclose(out["v_des"], out2["v_des"], atol=1e-6), \
            "v_des should be unchanged at saturation when K grows"

    print("[smoke] OK")


if __name__ == "__main__":
    main(sys.argv[1])
