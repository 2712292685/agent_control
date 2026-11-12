#!/usr/bin/env bash
# Start the loopback-only simulation stack (fake robot + driver + HTTP API).
# Run from the agent_control workspace after `colcon build`.
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export NO_PROXY=127.0.0.1,localhost
export no_proxy=127.0.0.1,localhost

FAKE_PORT="${FAKE_PORT:-18081}"
FAKE_REPLY_PORT="${FAKE_REPLY_PORT:-18080}"
BRIDGE_PORT="${BRIDGE_PORT:-8766}"
API_PORT="${API_PORT:-8100}"
FAKE_LOG="${FAKE_LOG:-/tmp/agent_fake.jsonl}"

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in $(ss -ltnp 2>/dev/null | grep -E ":($BRIDGE_PORT|$API_PORT)\b" | grep -oP 'pid=\K[0-9]+' | sort -u); do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[sim] stopped"
}
trap cleanup EXIT INT TERM

python3 -I tools/fake_robot.py \
    --listen "127.0.0.1:${FAKE_PORT}" \
    --reply "127.0.0.1:${FAKE_REPLY_PORT}" \
    --duration-sec 600 \
    --log-file "$FAKE_LOG" \
    --forward-speed 0.8 --strafe-speed 0.5 --yaw-rate 1.2 \
    > /tmp/agent_fake_stdout.log 2>&1 &
PIDS+=("$!")
sleep 0.5

install/agent_control_driver/lib/agent_control_driver/agent_control_driver \
    --ros-args \
    -p "robot_ip:=127.0.0.1" \
    -p "send_port:=${FAKE_PORT}" \
    -p "recv_port:=${FAKE_REPLY_PORT}" \
    -p "role:=sdk" \
    -p "bridge_port:=${BRIDGE_PORT}" \
    > /tmp/agent_driver_stdout.log 2>&1 &
PIDS+=("$!")
sleep 1.0

PYTHONPATH="$PWD/src/agent_control_api:${PYTHONPATH:-}" \
python3 -m agent_control_api.api_node \
    --ros-args \
    -p "port:=${API_PORT}" \
    -p "bind_address:=127.0.0.1" \
    -p "bridge_port:=${BRIDGE_PORT}" \
    > /tmp/agent_api_stdout.log 2>&1 &
PIDS+=("$!")

echo "[sim] fake log: $FAKE_LOG"
echo "[sim] API: http://127.0.0.1:${API_PORT}/  (Ctrl-C to stop)"
wait
