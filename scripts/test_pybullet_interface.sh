#!/usr/bin/env bash
# ROS-free test runner — same controller, same API call, but driven via PyBullet
# instead of Gazebo.  No roslaunch, no rosmaster.  Just one Python process.
#
# We DO need ROS sourced once to find iiwa_description/urdf/iiwa14.urdf.xacro
# and expand it with xacro — that's just URDF generation, not control.

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/ros_ws/devel/setup.bash 2>/dev/null

export MPLBACKEND=Agg

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_ROOT="${OUT_ROOT:-$HOME/ros_ws/src/iiwa_toolkit/test_runs}"
mkdir -p "${OUT_ROOT}"
TEST_DIR="${OUT_ROOT}/pybullet_interface_test_${STAMP}"
mkdir -p "${TEST_DIR}"
LOG="${TEST_DIR}/test.log"

echo "[pybullet_test] stamp=${STAMP}"
echo "[pybullet_test] log    -> ${LOG}"
echo "[pybullet_test] plots  -> ${TEST_DIR}"

# Always write plots into the run dir.  Any extra CLI args (e.g. --gui or
# --duration) get forwarded verbatim.
python3 "$HOME/ros_ws/src/iiwa_toolkit/src/iiwa_passive_ds/examples/test_pybullet_interface.py" \
        --plot-dir "${TEST_DIR}" "$@" 2>&1 | tee "${LOG}"
RC="${PIPESTATUS[0]}"

if [[ "${RC}" -eq 0 ]]; then
    echo "[pybullet_test] OVERALL: PASS"
else
    echo "[pybullet_test] OVERALL: FAIL (exit code ${RC})"
fi
exit "${RC}"
