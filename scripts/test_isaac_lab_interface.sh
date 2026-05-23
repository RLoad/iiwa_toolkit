#!/usr/bin/env bash
# End-to-end test: drive Gazebo via the Isaac-Lab-style pure-Python interface.
#
#   1. start iiwa_gazebo.launch  (Gazebo + URDF only, NO Python controller)
#   2. wait for /clock + /iiwa/joint_states
#   3. run test_isaac_lab_interface.py — that script IS the controller now,
#      built with AttractorDSController(kinematics=None) and driven via
#      compute_from_state(...) with externally-computed FK + Jacobian.
#   4. on script exit, SIGINT roslaunch
#   5. report PASS / FAIL based on test_isaac_lab_interface.py's exit code.

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

export MPLBACKEND=Agg
DURATION="${DURATION:-42}"
GUI="${GUI:-false}"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_ROOT="${OUT_ROOT:-$HOME/ros_ws/src/iiwa_toolkit/test_runs}"
mkdir -p "${OUT_ROOT}"
TEST_DIR="${OUT_ROOT}/isaac_lab_interface_test_${STAMP}"
mkdir -p "${TEST_DIR}"
LOG_CSV="${TEST_DIR}/log_${STAMP}.csv"
GZ_LOG="${TEST_DIR}/gazebo.log"
TEST_LOG="${TEST_DIR}/test.log"

cleanup() {
    echo "[isaac_test] cleanup"
    [[ -n "${TEST_PID:-}" ]] && kill -INT "$TEST_PID" 2>/dev/null
    [[ -n "${GZ_PID:-}"   ]] && kill -INT "$GZ_PID"   2>/dev/null
    for _ in $(seq 1 20); do
        if ! kill -0 "$GZ_PID" 2>/dev/null; then break; fi
        sleep 0.5
    done
    pkill -9 -f test_isaac_lab_interface.py 2>/dev/null
    pkill -9 -f gzserver                    2>/dev/null
    pkill -9 -f gzclient                    2>/dev/null
    pkill -9 -f rosmaster                   2>/dev/null
    pkill -9 -f roslaunch                   2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[isaac_test] stamp=${STAMP}  duration=${DURATION}s  gui=${GUI}"
echo "[isaac_test] csv  -> ${LOG_CSV}"

# 1) Gazebo + URDF only (no Python controller node)
roslaunch iiwa_gazebo iiwa_gazebo.launch gui:="${GUI}" \
    > "${GZ_LOG}" 2>&1 &
GZ_PID=$!

# 2) wait for clock + joint_states
echo "[isaac_test] waiting for /clock and /iiwa/joint_states ..."
deadline=$((SECONDS + 60))
ready=0
while [[ $SECONDS -lt $deadline ]]; do
    if rostopic list 2>/dev/null | grep -q '^/iiwa/joint_states$' \
       && rostopic list 2>/dev/null | grep -q '^/clock$'; then
        if timeout 2 rostopic echo -n1 /iiwa/joint_states >/dev/null 2>&1; then
            ready=1; break
        fi
    fi
    sleep 0.5
done
if [[ $ready -ne 1 ]]; then
    echo "[isaac_test] FAIL: Gazebo never came up; gazebo log tail:"
    tail -120 "${GZ_LOG}"
    exit 2
fi
echo "[isaac_test] Gazebo up — starting pure-Python interface test"

# 3) Run the test script — this IS the controller via compute_from_state
python3 "$HOME/ros_ws/src/iiwa_toolkit/scripts/test_isaac_lab_interface.py" \
        --duration "${DURATION}" \
        --log-csv  "${LOG_CSV}" \
        > "${TEST_LOG}" 2>&1 &
TEST_PID=$!

# 4) Wait for the test to finish (it exits on its own when schedule done)
wait_deadline=$((SECONDS + DURATION + 10))
while [[ $SECONDS -lt $wait_deadline ]]; do
    if ! kill -0 "$TEST_PID" 2>/dev/null; then break; fi
    sleep 0.5
done
wait "$TEST_PID" 2>/dev/null
TEST_RC=$?

# 5) SIGINT Gazebo
kill -INT "$GZ_PID" 2>/dev/null
for _ in $(seq 1 20); do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then break; fi
    sleep 0.5
done

echo
echo "===== test script output (tail) ====="
tail -30 "${TEST_LOG}"
echo "======================================"

if [[ -f "${LOG_CSV}" ]]; then
    nrows=$(wc -l < "${LOG_CSV}")
    echo "[isaac_test] CSV present (${nrows} lines): ${LOG_CSV}"
else
    echo "[isaac_test] WARN: no CSV produced"
fi

if [[ "${TEST_RC}" -eq 0 ]]; then
    echo "[isaac_test] OVERALL: PASS"
    exit 0
else
    echo "[isaac_test] OVERALL: FAIL (test_isaac_lab_interface.py exit code ${TEST_RC})"
    exit 1
fi
