# `iiwa_passive_ds` — Pure-Python passive DS + attractor DS controller

A pure-PyTorch implementation of the KUKA iiwa passive DS controller, with an
outer linear-attractor DS layer baked in.

The package has **zero ROS imports** — it's just `torch` (and optionally
`pytorch_kinematics` if you use the standalone FK provider).  It runs as
N parallel envs on GPU inside Isaac Lab, where the RL policy supplies the
attractor + gains directly each step.

This repo ships a **same-process, no-ROS demo** that runs the same code in
PyBullet, driven by a hand-written planner — so you can verify the
controller works end-to-end before wiring it into Isaac Lab.

---

## Three pieces

```
        ┌─────────────────────┐
        │ planner.py          │   attractor_pos, attractor_quat,
        │  (5-segment demo —  │   K_linear, damping_a, damping_b
        │   replaced by your  ├────────────────────────────────────────┐
        │   RL policy in      │                                        │
        │   Isaac Lab)        │                                        │
        └─────────────────────┘                                        │
                                                                       ▼
        ┌─────────────────────────────────────────────────────────────────┐
        │  AttractorDSController.compute_from_state(...)                  │
        │                                                                 │
        │   in : q, dq, ee_pos, ee_quat, J_pos, J_ang,                    │
        │        attractor_pos, attractor_quat, K_linear,                 │
        │        damping_a, damping_b                                     │
        │   out: tau (joint torques)                                      │
        │                                                                 │
        │   (no ROS, no FK inside, no force feedback)                     │
        └─────────────────────────────────────────────────────────────────┘
                                ▲          │
                                │          ▼
        ┌───────────────────────┴────┐  ┌─────────────────────────────┐
        │ test_pybullet_interface.py │  │ Isaac Lab task env          │
        │ (PyBullet sim — the demo   │  │ (Isaac Sim — your env       │
        │  runner; same call         │  │  uses the SAME call         │
        │  pattern as Isaac Lab)     │  │  pattern)                   │
        │                            │  │                             │
        │  q, dq <- getJointStates   │  │  q, dq <- articulation.data │
        │  FK    <- TorchKinematicsPK│  │  FK    <- articulation.data │
        │  tau   -> TORQUE_CONTROL   │  │  tau   -> set_joint_effort  │
        └────────────────────────────┘  └─────────────────────────────┘
```

1. **Controller library** — `iiwa_passive_ds/` package (this directory).  Zero
   ROS imports.  Same code runs in PyBullet here and in Isaac Lab there.
2. **Planner** — `examples/planner.py`.  Just a list of timed segments, each
   a constant attractor + gain triple.  In Isaac Lab this is what the RL
   policy's action tensor replaces.
3. **PyBullet runner** — `examples/test_pybullet_interface.py`.  Loads the
   iiwa URDF, advances physics, calls `compute_from_state` each step.  Same
   call pattern Isaac Lab will use.

---

## Algorithm in one picture

```
   ┌────────────────────────────────────────────────────────────────┐
   │  AttractorDSController                                          │
   │                                                                 │
   │   inputs (per-env, batched):                                    │
   │     q, dq            (robot state)                              │
   │     ee_pos, ee_quat  (FK — supplied externally)                 │
   │     J_pos, J_ang     (Jacobian — supplied externally)           │
   │     attractor_pos    ┐                                          │
   │     attractor_quat   │                                          │
   │     K_linear         │  ── from planner / RL policy ──          │
   │     damping_a        │                                          │
   │     damping_b        ┘                                          │
   │                                                                 │
   │   ┌──────────────────────────────────────────────────────────┐  │
   │   │ outer  : v_des = -K_linear * (ee_pos - attractor_pos)    │  │
   │   │           ‖v_des‖ ≤ velocity_limit  (norm saturation)    │  │
   │   ├──────────────────────────────────────────────────────────┤  │
   │   │ inner  : PassiveDSCore                                   │  │
   │   │           ├─ anisotropic passive DS (lam0/lam1 scaled    │  │
   │   │           │   per-env by damping_a / damping_b)          │  │
   │   │           ├─ orientation DS  (slerp + quat error)        │  │
   │   │           └─ null-space joint PD (toward null_q)         │  │
   │   └──────────────────────────────────────────────────────────┘  │
   │                                                                 │
   │   output:  tau (B, n_dof)  + intermediate state for RL obs/cost │
   └────────────────────────────────────────────────────────────────┘
```

Everything is `torch` ops with a leading batch dim — no Python loops over
envs.  Works on CPU and CUDA, `float32` or `float64`.

---

## Package layout

```
iiwa_passive_ds/
├── __init__.py               re-exports PassiveDS, PassiveDSCore, AttractorDSController
├── core.py                   PassiveDSCore — inner passive DS + null-space PD + init phase.
│                             Mirrors C++ iiwa_toolkit/src/passive_control.cpp.
│                             Pure algorithm: no FK, no ROS, no sim.
├── attractor_ds.py           AttractorDSController — outer attractor DS layer + ‖v‖ clamp,
│                             stacked onto PassiveDSCore.  This is the main entry point.
├── kinematics.py             TorchKinematicsPK — pytorch_kinematics-based FK + Jacobian.
│                             Optional: Isaac Lab can supply its own state directly.
├── _smoke.py                 Standalone unit test (no ROS / no Gazebo required).
├── examples/
│   ├── planner.py            The hand-written schedule used by the PyBullet demo.
│   │                         Substitute for an RL policy.  Delete in Isaac Lab.
│   ├── test_pybullet_interface.py   PyBullet runner — same-process, no-ROS demo.
│   └── isaac_lab_example.py  Documentation-grade Isaac Lab wiring template.
└── README.md                 (this file)
```

---

## Quick start — run the PyBullet demo

```bash
# Inside the docker container (which already has all deps installed):
docker compose exec iiwa-robot bash -c \
    "~/ros_ws/src/iiwa_toolkit/scripts/test_pybullet_interface.sh"
```

You'll see 5 segments — A → B → C → D → E — each holding an attractor for
12 seconds.  All five should PASS (best EE-attractor distance < 5 cm):

```
[pybullet_test] using URDF: /tmp/iiwa14_pybullet.urdf
[pybullet_test] controller built with kinematics=None  velocity_limit=0.150
[pybullet_test] PyBullet ready; iiwa loaded with 7 joints
  ...
  seg 0 (A — front high      (K=0.5)): best=0.0027 m  final=0.0027 m  [PASS]
  seg 1 (B — front-right low (K=0.5)): best=0.0212 m  final=0.0212 m  [PASS]
  seg 2 (C — back-left high  (K=0.8)): best=0.0379 m  final=0.0379 m  [PASS]
  seg 3 (D — center low      (soft damping)): best=0.0017 m  final=0.0017 m  [PASS]
  seg 4 (E — center          (stiff damping)): best=0.0004 m  final=0.0004 m  [PASS]
[pybullet_test] OVERALL: PASS
```

Want to **watch it move** in PyBullet's OpenGL window?  Add `--gui`:

```bash
docker compose exec -e DISPLAY=$DISPLAY iiwa-robot bash -c \
    "~/ros_ws/src/iiwa_toolkit/scripts/test_pybullet_interface.sh --gui"
```

With `--gui` the loop walks wall-clock alongside sim time (≈1× real-time)
so you can actually see the iiwa move through the schedule.

### Where artifacts go

Every run writes a timestamped directory under
`iiwa_toolkit/test_runs/pybullet_interface_test_<UTC>/`:

| file | what it shows |
|---|---|
| `test.log` | full stdout — per-step heartbeat + per-segment PASS / FAIL summary |
| `ee_trajectory_3d.png` | 3-D EE path from start (green dot) through the 5 attractors (red ★) |
| `position_error.png` | per-axis EE − attractor error and `‖error‖` vs time, with the 5 cm pass tolerance band |
| `torque_norm.png` | `‖tau‖` over the run — for spotting saturation against URDF effort limits |

Plots use matplotlib's `Agg` backend so they save fine in headless mode.

---

## Install

### As a pip package (Isaac Lab use)

```bash
cd <repo_root>/src/iiwa_toolkit/src
pip install -e .
```

After this, in any Python env on the same machine:

```python
from iiwa_passive_ds import PassiveDSCore, AttractorDSController
```

### Dependencies

| dep | required for | how to get |
|---|---|---|
| `torch >= 2.0` | everything | pip |
| `pytorch_kinematics` | `TorchKinematicsPK` only | `pip install -e .[fk]` |
| `pybullet`, `numpy==1.24.4` | the PyBullet demo only | already in the docker image |

Isaac Lab uses its own articulation API for FK + Jacobian, so it does NOT
need pytorch_kinematics.

---

## API reference

### `class PassiveDSCore`  (core.py)

Inner passive DS + null-space PD + (optional) coarse-positioning init phase.
Pure algorithm — takes FK + Jacobian as inputs, returns torque.

Constructor:

```python
PassiveDSCore(
    lam0_pos=100.0, lam1_pos=100.0,     # inner DS λ base values (position branch)
    lam0_ori=5.0,   lam1_ori=5.0,       # inner DS λ base values (orientation branch)
    ds_gain_pos=5.0,                    # used only by the optional outer-pos DS
    ds_gain_ori=2.5,                    # outer-orientation DS gain
    max_dx=0.1, max_dq=0.2,             # saturation on outer-DS error vectors
    null_q=None, null_gains=None,       # null-space joint PD target + gains
    null_damping=1.0,
    null_scale=10.0,                    # τ = τ_task + null_scale * N * τ_null
    init_phase_norm=1.5,                # ‖q - null_q‖ threshold to leave init phase
    enable_init_phase=True,             # set False in Isaac Lab (no latched state)
    load_added=0.0,                     # kg, added EE z-direction gravity comp
    gravity_z=9.8,
)
```

Init-phase latch (resets coarse positioning for a subset of envs):

```python
core.reset_init_phase(mask=None)        # mask=None → all envs
```

---

### `class AttractorDSController`  (attractor_ds.py)

**The main entry point.**  Wraps PassiveDSCore + outer attractor DS layer.

Constructor:

```python
AttractorDSController(
    core,                # PassiveDSCore instance
    kinematics=None,     # optional — needed only by .compute(...) below.
                         # Always pass None when using compute_from_state.
    velocity_limit=0.15, # m/s; ‖v_des‖ saturation
                         # (matches C++ test_ori_control::Velocity_limit_)
)
```

#### `compute_from_state(...)` — the only call you need

```python
out = ctrl.compute_from_state(
    q, dq,                    # (B, n_dof)
    ee_pos, ee_quat,          # (B, 3) and (B, 4) [w,x,y,z]
    J_pos, J_ang,             # (B, 3, n_dof) and (B, 3, n_dof)
    attractor_pos,            # (B, 3)   ← planner / policy knob
    attractor_quat,           # (B, 4)   ← planner / policy knob
    K_linear,                 # (B,)     ← planner / policy knob
    damping_a,                # (B,)     ← planner / policy knob
    damping_b,                # (B,)     ← planner / policy knob
)
```

Returns dict:

| key | shape | what it is |
|---|---|---|
| `tau` | (B, n_dof) | joint torques (primary output) |
| `v_des` | (B, 3) | velocity reference into passive DS (after ‖v‖ clamp) |
| `v_des_raw` | (B, 3) | velocity reference before ‖v‖ clamp (useful for RL obs) |
| `ee_pos` | (B, 3) | pass-through (so you can use one dict for control + obs) |
| `ee_quat` | (B, 4) | pass-through |
| `ee_vel` | (B, 3) | `J_pos @ dq` |
| `ee_angVel` | (B, 3) | `J_ang @ dq` |
| `first_flag` | (B,) bool or None | True iff this env is still in init phase |

`compute(q, dq, attractor_pos, attractor_quat, K_linear, damping_a, damping_b)`
is a thin wrapper that runs FK + Jacobian via the kinematics provider before
dispatching to `compute_from_state`.  Use it only for standalone scripts that
don't yet maintain FK state.  Isaac Lab always uses `compute_from_state`.

---

### `class TorchKinematicsPK`  (kinematics.py) — optional FK provider

Wraps `pytorch_kinematics` to compute batched FK + geometric Jacobian for a
fixed serial chain.  The PyBullet demo uses it; Isaac Lab does NOT.

```python
kin = TorchKinematicsPK(urdf_string=urdf_bytes,
                        ee_link="iiwa_link_ee",
                        root_link="iiwa_link_0",
                        device="cpu", dtype=torch.float64)

ee_pos, ee_quat = kin.fk(q)              # (B, 3), (B, 4) [w,x,y,z]
J_pos, J_ang    = kin.jacobian(q)        # (B, 3, n_dof), (B, 3, n_dof)
```

---

## Tensor conventions

| tensor | shape | notes |
|---|---|---|
| `q`, `dq` | (B, n_dof) | n_dof = 7 for iiwa |
| `ee_pos` | (B, 3) | world frame |
| `ee_quat`, `attractor_quat` | (B, 4) | `[w, x, y, z]` — same as Eigen `Quaternion(w,x,y,z)` and Isaac Lab `body_quat_w` |
| `ee_vel`, `ee_angVel`, `v_des`, `v_des_raw` | (B, 3) | world frame |
| `J_pos`, `J_ang` | (B, 3, n_dof) | geometric Jacobian, linear / angular halves |
| `K_linear`, `damping_a`, `damping_b` | (B,) | one scalar per env |
| `tau` | (B, n_dof) | joint torque command |

`dtype` and `device` are taken from `q`; the rest must match.  All operations
are pure `torch`, so `q.device = "cuda"` puts the whole compute on GPU.

---

## The planner (`examples/planner.py`)

A 50-line file that holds:

```python
@dataclass
class Segment:
    t_start:   float        # seconds at which this segment becomes active
    pos:       List[float]  # (3,) attractor position (world frame)
    quat:      List[float]  # (4,) attractor orientation [w, x, y, z]
    K_linear:  float        # outer DS gain
    damping_a: float        # inner DS λ0 multiplier
    damping_b: float        # inner DS λ1 multiplier
    label:     str          # for plots / logs

SCHEDULE = [
    Segment( 5.0, [0.7,  0.0, 0.6], TOOL_Z_ALONG_X, 0.5, 1.0, 1.0, "A — front high"),
    Segment(17.0, [0.7, -0.2, 0.4], TOOL_Z_ALONG_X, 0.5, 1.0, 1.0, "B — front-right low"),
    Segment(29.0, [0.4,  0.2, 0.6], TOOL_Z_ALONG_X, 0.8, 1.0, 1.0, "C — back-left high"),
    Segment(41.0, [0.5,  0.0, 0.4], TOOL_Z_ALONG_X, 0.5, 0.5, 0.5, "D — center low (soft damping)"),
    Segment(53.0, [0.5,  0.0, 0.6], TOOL_Z_ALONG_X, 0.5, 2.0, 2.0, "E — center (stiff damping)"),
]

def current_segment(t_elapsed: float) -> Optional[Segment]:
    """Latest segment whose t_start ≤ t_elapsed, else None during warmup."""
```

That's the whole planner.  It does ONE thing per timestep: take wall-clock
`t`, return the active `(attractor_pos, attractor_quat, K_linear, damping_a,
damping_b)`.

**This is the file you delete in Isaac Lab.**  Replace it with the policy
network's `action` tensor:

```python
# In Isaac Lab, INSTEAD of looking up the schedule:
seg = planner.current_segment(t)   # ← this whole call goes away
attractor_pos  = torch.tensor([seg.pos])
attractor_quat = torch.tensor([seg.quat])
...

# In Isaac Lab:
attractor_pos  = action[:, 0:3]    # ← policy output, batched (B, 3)
attractor_quat = self.fixed_quat   # or action[:, 3:7] if you want it learned
K_linear       = action[:, 3]      # (B,)
damping_a      = action[:, 4]
damping_b      = action[:, 5]
```

Nothing else about the call to `compute_from_state` changes.

---

## Isaac Lab integration walkthrough

### 1. Install the library in Isaac Lab's Python env

```bash
# Inside the Isaac Lab conda env / virtualenv
cd <repo_root>/src/iiwa_toolkit/src
pip install -e .
```

### 2. Build the controller once at env init

```python
import torch
from iiwa_passive_ds import PassiveDSCore, AttractorDSController

class MyIiwaTaskEnv(...):
    def __init__(self, num_envs, device, ...):
        ...
        dtype = torch.float32   # GPU-friendly
        null_q     = torch.tensor([0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0],
                                   dtype=dtype, device=device)
        null_gains = torch.tensor([5.0, 80.0, 10.0, 30.0, 5.0, 2.0, 2.0],
                                   dtype=dtype, device=device)
        core = PassiveDSCore(
            lam0_pos=100.0, lam1_pos=100.0,
            lam0_ori=5.0,   lam1_ori=5.0,
            ds_gain_ori=2.5, max_dx=0.10, max_dq=0.20,
            null_q=null_q, null_gains=null_gains,
            null_damping=1.0, null_scale=10.0,
            enable_init_phase=False,                  # OFF for RL
        )
        self.ctrl = AttractorDSController(core,
                                          kinematics=None,
                                          velocity_limit=0.15)
        self.fixed_quat = torch.tensor(
            [0.7071068, 0.0, 0.7071068, 0.0],         # tool z along world +x
            dtype=dtype, device=device,
        ).repeat(num_envs, 1)
```

### 3. Apply the policy action each step

```python
def _apply_action(self):
    q   = self.iiwa.data.joint_pos                          # (B, 7)
    dq  = self.iiwa.data.joint_vel                          # (B, 7)
    ee  = self.iiwa.data.body_pos_w[:, self.ee_idx]         # (B, 3)
    eeq = self.iiwa.data.body_quat_w[:, self.ee_idx]        # (B, 4) [w,x,y,z]
    J   = self.iiwa.root_physx_view.get_jacobians()[:, self.ee_idx]   # (B, 6, n_dof)
    Jp, Ja = J[:, 0:3, :], J[:, 3:6, :]

    out = self.ctrl.compute_from_state(
        q=q, dq=dq,
        ee_pos=ee, ee_quat=eeq,
        J_pos=Jp, J_ang=Ja,
        attractor_pos = self.action[:, 0:3],   # ← policy output (was planner)
        attractor_quat= self.fixed_quat,       # or self.action[:, 3:7]
        K_linear      = self.action[:, 3],
        damping_a     = self.action[:, 4],
        damping_b     = self.action[:, 5],
    )
    self.iiwa.set_joint_effort_target(out["tau"])
    self._last_ctrl_out = out                       # for reward / obs
```

### 4. Handle env resets

```python
def _reset_idx(self, env_ids):
    super()._reset_idx(env_ids)
    if self.ctrl.core._first is not None:
        mask = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        mask[env_ids] = True
        self.ctrl.reset_init_phase(mask)
```

### 5. Reward / cost — force lives HERE, not in the control loop

```python
def _get_rewards(self):
    F_world = self.contact_sensor.data.net_forces_w[:, self.ee_idx, :]  # (B, 3)
    F_mag   = torch.linalg.vector_norm(F_world, dim=-1)
    pos_err = torch.linalg.vector_norm(
        self._last_ctrl_out["ee_pos"] - self.attractor_pos, dim=-1)
    return -1.0 * pos_err - 0.1 * (F_mag - self.desired_force).pow(2)
```

The controller **never** receives force as input.  This is deliberate — it
keeps the control law a pure DS, and lets RL learn the right
`K_linear`/`damping_a`/`damping_b`/`attractor_pos` to produce the desired
contact behaviour.  Mixing force feedback into the inner loop fights this.

A full template lives at `iiwa_passive_ds/examples/isaac_lab_example.py`.

---

## PyBullet runner → Isaac Lab — what to swap

The PyBullet runner (`test_pybullet_interface.py`) and the Isaac Lab task env
differ ONLY in where state comes from and where torque goes.  The controller
call is identical, character-for-character.

| concern | PyBullet runner | Isaac Lab task env |
|---|---|---|
| simulator | PyBullet (`p.GUI` / `p.DIRECT`) | Isaac Sim |
| batching | `B = 1` (single env) | `B = num_envs` — same call, larger leading dim |
| device | `cpu`, `float64` | `cuda`, `float32` |
| URDF loading | `p.loadURDF(...)` after xacro-expand + `package://` resolve | Isaac Lab `ArticulationCfg` / USD |
| `q`, `dq` | `p.getJointStates(...)` | `articulation.data.joint_pos / joint_vel` |
| `ee_pos`, `ee_quat` | `TorchKinematicsPK.fk(q)` | `articulation.data.body_pos_w[:, ee_idx]` and `body_quat_w` |
| `J_pos`, `J_ang` | `TorchKinematicsPK.jacobian(q)` | `articulation.root_physx_view.get_jacobians()[:, ee_idx]` (split rows 0-2 / 3-5) |
| attractor / gains source | `planner.current_segment(t)` | `self.action` tensor from the policy |
| **controller call** | **`ctrl.compute_from_state(q, dq, ee_pos, ee_quat, J_pos, J_ang, attractor_pos, attractor_quat, K_linear, damping_a, damping_b)`** | **identical** |
| apply `out["tau"]` | `p.setJointMotorControlArray(..., p.TORQUE_CONTROL, forces=tau)` | `articulation.set_joint_effort_target(out["tau"])` |
| EE force (reward only) | n/a in this demo | `contact_sensor.data.net_forces_w[:, ee_idx, :]` (read, do **not** feed to controller) |
| reset hook | n/a (single 65 s run) | `ctrl.reset_init_phase(reset_mask)` from `_reset_idx` |

### Controller gain note

The PyBullet demo uses slightly stiffer gains than the suggested Isaac Lab
starting point (`lam0_pos=300` and `null_scale=2` instead of `100` and
`10`).  Reason: PyBullet's solver leaves small inertia/damping residuals
that the DS — with no integral action — manifests as a few-cm steady-state
position error.  Bumping `lam0_pos` and weakening the null-space pull-back
brings every segment under 5 cm.

Isaac Sim's Newton solver is closer to real-time physics, so start with the
gain values in the Isaac Lab walkthrough above (`lam0_pos=100`,
`null_scale=10`).  Only adjust if you see similar steady-state plateaus.

---

## Gotchas

* **Init phase**: `enable_init_phase=True` makes the controller output ONLY
  null-space joint PD while `‖q - null_q‖ > init_phase_norm`, ignoring the
  task command.  Mirrors C++.  Set `False` for Isaac Lab — RL training
  doesn't want a latched state-machine.

* **Velocity-norm saturation**: `velocity_limit=0.15` m/s mimics C++
  `Velocity_limit_`.  It's a constructor parameter on purpose — a safety
  bound, not an RL knob.  When ‖v_raw‖ exceeds it the *direction* is kept
  but the magnitude is clipped.

* **No force in the control loop**: by design.  See the reward example.

* **No low-pass filter on v_des**: looking at the C++ source, the variable
  `desired_velocity_filtered_` is misnamed — there's actually no temporal
  filter, just the same norm clamp.  We match that exactly.

* **Quaternion sign convention**: `[w, x, y, z]`, same as Eigen and
  Isaac Lab's `body_quat_w`.  `_rotmat_to_quat_wxyz` in `kinematics.py`
  forces `w ≥ 0` (Eigen-compatible hemisphere).

* **Joint order**: PassiveDSCore is agnostic; you just have to make sure
  `q`, `dq`, `J_pos`, `J_ang`, `null_q` and `null_gains` all share the same
  ordering.  Isaac Lab's articulation may not match URDF order — use
  `articulation.find_joints(...)` to remap if needed.

* **dtype / device**: everything follows the input `q`'s dtype/device.
  Mixing `q.cuda()` with a `null_q.cpu()` core will silently cast on GPU
  each step — not a bug, but slow.  Build `PassiveDSCore` with `null_q` and
  `null_gains` already on the target device.

* **PyBullet torque control**: PyBullet defaults to a velocity-servo motor
  per joint.  You MUST `setJointMotorControl2(VELOCITY_CONTROL, force=0)`
  once per joint before commanding torques, otherwise the default servo
  fights your `TORQUE_CONTROL` commands silently.  Setting `force=0` also
  disables the URDF's `<dynamics damping>` motor friction — the runner
  re-injects it via `changeDynamics(jointDamping=...)`.

* **Per-joint effort limits**: iiwa14's wrist (joints 6, 7) is capped at
  40 N·m in the URDF, while joints 1-5 go up to 320 N·m.  PyBullet's
  TORQUE_CONTROL doesn't enforce the URDF limits, so the runner clips
  per-joint with `TAU_ABS_LIMIT = [320, 320, 176, 176, 110, 40, 40]` —
  mirroring real-iiwa firmware saturation.

---

## C++ correspondence

| Python | C++ counterpart |
|---|---|
| `PassiveDSCore` | `iiwa_toolkit/src/passive_control.cpp` :: `PassiveControl` |
| `PassiveDS.update` | `passive_control.cpp` :: `PassiveDS::update` |
| `AttractorDSController` | `RL_controller/task_space_control.cpp` phase-2 logic, but stripped of multi-phase planning and force feedback |
| `velocity_limit=0.15` | `task_space_control.cpp` :: `Velocity_limit_(0.15)` |
| `null_q`, `null_gains` | C++ hard-coded constants in `passive_control.cpp` |
| `enable_init_phase`, `init_phase_norm=1.5` | C++ `_first` flag + `er_null.norm() < 1.5` check |
