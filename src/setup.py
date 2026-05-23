"""Pip-install metadata for the pure-Python iiwa_passive_ds library.

Installs the iiwa_passive_ds package — the algorithm core, kinematics
provider and AttractorDSController — and leaves the ROS-only node scripts
that live next to it (node_attractor_ds_gazebo.py, ds_planner.py) alone.

Typical use:

    # inside the Isaac Lab python env
    cd /path/to/iiwa_docker_RL/src/iiwa_toolkit/src
    pip install -e .

After that:

    from iiwa_passive_ds import PassiveDSCore, AttractorDSController
    from iiwa_passive_ds.kinematics import TorchKinematicsPK   # optional
"""
from setuptools import setup, find_packages

setup(
    name="iiwa_passive_ds",
    version="0.1.0",
    description="Pure-Python passive DS + attractor DS controller for the "
                "KUKA iiwa, batchable on CPU/CUDA, no ROS dependency.",
    author="Rui",
    packages=find_packages(include=["iiwa_passive_ds", "iiwa_passive_ds.*"]),
    install_requires=[
        "torch>=2.0",
        # pytorch_kinematics is optional — only needed by TorchKinematicsPK,
        # which Isaac Lab can skip entirely via AttractorDSController.compute_from_state().
    ],
    extras_require={
        # Standalone (non-Isaac-Lab) FK via pytorch_kinematics
        "fk": ["pytorch_kinematics"],
    },
    python_requires=">=3.8",
)
