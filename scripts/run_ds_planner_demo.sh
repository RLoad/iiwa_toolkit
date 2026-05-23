#!/usr/bin/env bash
# DS planner demo runner.
#
# Flow:
#   1. start attractor_ds_gazebo.launch headless   — robot transitions to and
#      then holds at the init pose from attractor_ds_params.yaml
#   2. wait for /clock + /iiwa/joint_states        — Gazebo + controller ready
#   3. wait WARMUP seconds                          — let the EE settle at init
#   4. start ds_planner.py                          — publishes a 5-attractor
#                                                     schedule with gain changes
#   5. wait for ds_planner to exit                  — when the schedule is done
#   6. SIGINT roslaunch so the CSV flushes cleanly
#   7. auto-run plot_attractor_ds_log.py            — text summary + 5 PNGs
#
# Env knobs (with defaults):
#   N_ATTRACTORS=5      how many segments to play (max 5 with built-in schedule)
#   SEG_DURATION=12     seconds per segment
#   WARMUP=5            seconds the robot holds at init before seg 0 fires
#   DEVICE=cpu          "cpu" or "cuda"
#   GUI=false           "true" to bring up Gazebo GUI (needs X forwarding)

# Don't use `set -u` — ROS setup scripts touch unbound vars.

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

export MPLBACKEND=Agg
N_ATTRACTORS="${N_ATTRACTORS:-5}"
SEG_DURATION="${SEG_DURATION:-12}"
WARMUP="${WARMUP:-5}"
DEVICE="${DEVICE:-cpu}"
GUI="${GUI:-false}"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_ROOT_DEFAULT="$HOME/ros_ws/src/iiwa_toolkit/test_runs"
OUT_ROOT="${OUT_ROOT:-$OUT_ROOT_DEFAULT}"
mkdir -p "${OUT_ROOT}" || OUT_ROOT="/tmp"
TEST_LOG_DIR="${OUT_ROOT}/ds_planner_demo_${STAMP}"
mkdir -p "${TEST_LOG_DIR}"
LOG_CSV="${TEST_LOG_DIR}/attractor_ds_py_log_${STAMP}.csv"
CTRL_LOG="${TEST_LOG_DIR}/controller.log"
PLANNER_LOG="${TEST_LOG_DIR}/ds_planner.log"
SCHEDULE_FILE="${TEST_LOG_DIR}/attractor_schedule.txt"

cleanup() {
    echo "[demo] cleanup"
    [[ -n "${PLAN_PID:-}" ]] && kill -INT "$PLAN_PID" 2>/dev/null
    [[ -n "${CTRL_PID:-}" ]] && kill -INT "$CTRL_PID" 2>/dev/null
    for _ in $(seq 1 20); do
        if ! kill -0 "$CTRL_PID" 2>/dev/null; then break; fi
        sleep 0.5
    done
    pkill -9 -f node_attractor_ds_gazebo.py 2>/dev/null
    pkill -9 -f ds_planner.py               2>/dev/null
    pkill -9 -f gzserver                    2>/dev/null
    pkill -9 -f gzclient                    2>/dev/null
    pkill -9 -f rosmaster                   2>/dev/null
    pkill -9 -f roslaunch                   2>/dev/null
}
trap cleanup EXIT INT TERM

# Total wall time = warmup + N * seg + plot/shutdown margin
TOTAL_T=$(( WARMUP + N_ATTRACTORS * SEG_DURATION ))
echo "[demo] stamp=${STAMP}  N=${N_ATTRACTORS} segs × ${SEG_DURATION}s + ${WARMUP}s warmup"
echo "[demo] csv  -> ${LOG_CSV}"
echo "[demo] logs -> ${TEST_LOG_DIR}"

# 1) controller + Gazebo
roslaunch iiwa_toolkit attractor_ds_gazebo.launch \
    gui:="${GUI}" device:="${DEVICE}" log_csv:="${LOG_CSV}" \
    > "${CTRL_LOG}" 2>&1 &
CTRL_PID=$!

# 2) wait for /clock and /iiwa/joint_states
echo "[demo] waiting for /clock and /iiwa/joint_states ..."
deadline=$((SECONDS + 60))
ready=0
while [[ $SECONDS -lt $deadline ]]; do
    if rostopic list 2>/dev/null | grep -q '^/clock$' \
       && rostopic list 2>/dev/null | grep -q '^/iiwa/joint_states$'; then
        if timeout 2 rostopic echo -n1 /iiwa/joint_states >/dev/null 2>&1; then
            ready=1; break
        fi
    fi
    sleep 0.5
done
if [[ $ready -ne 1 ]]; then
    echo "[demo] FAIL: Gazebo / joint_states never came up; controller log:"
    tail -120 "${CTRL_LOG}"
    exit 2
fi
echo "[demo] /clock + joint_states up — robot now driving to init pose"

# 3) Start ds_planner.  It has its own --warmup so it holds publishing until the
# robot has had time to settle at the controller's init pose.
echo "[demo] starting ds_planner (warmup ${WARMUP}s then ${N_ATTRACTORS} segs)"
rosrun iiwa_toolkit ds_planner.py \
        --n-attractors  "${N_ATTRACTORS}" \
        --seg-duration  "${SEG_DURATION}" \
        --warmup        "${WARMUP}" \
        --schedule-out  "${SCHEDULE_FILE}" \
        > "${PLANNER_LOG}" 2>&1 &
PLAN_PID=$!

# 4) Wait for ds_planner to finish on its own (it exits when the schedule ends).
# Bound the wait so a hang doesn't ruin the whole run.
echo "[demo] waiting for ds_planner to finish (~${TOTAL_T}s) ..."
wait_deadline=$((SECONDS + TOTAL_T + 10))
while [[ $SECONDS -lt $wait_deadline ]]; do
    if ! kill -0 "$PLAN_PID" 2>/dev/null; then break; fi
    sleep 0.5
done
if kill -0 "$PLAN_PID" 2>/dev/null; then
    echo "[demo] WARN: ds_planner still running after ${TOTAL_T}s — killing it"
    kill -INT "$PLAN_PID" 2>/dev/null
fi

# 5) SIGINT controller so its on_shutdown CSV flush fires
echo "[demo] sending SIGINT to roslaunch (controller flushes CSV)"
kill -INT "$CTRL_PID" 2>/dev/null
for _ in $(seq 1 20); do
    if ! kill -0 "$CTRL_PID" 2>/dev/null; then break; fi
    sleep 0.5
done

# 6) Verify CSV
if [[ -f "${LOG_CSV}" ]]; then
    nrows=$(wc -l < "${LOG_CSV}")
    echo "[demo] CSV present: ${LOG_CSV} (${nrows} lines)"
else
    echo "[demo] FAIL: expected CSV ${LOG_CSV} not found"
    exit 3
fi

# 7) Auto plot + summary
PLOT_SCRIPT="$HOME/ros_ws/src/iiwa_toolkit/scripts/plot_attractor_ds_log.py"
if [[ -x "${PLOT_SCRIPT}" ]]; then
    echo "[demo] running plot/validation"
    python3 "${PLOT_SCRIPT}" "${LOG_CSV}" "${TEST_LOG_DIR}" "${SCHEDULE_FILE}" \
        | tee "${TEST_LOG_DIR}/summary_console.txt"
fi

echo "[demo] artifacts -> ${TEST_LOG_DIR}"
echo "[demo] DONE"
