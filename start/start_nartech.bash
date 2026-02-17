#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="${1:-default}"
DEMO_FILE="${2:-demo_with_nars.metta}"
START_NARTECH_NODE=true

LIVOX_TOPIC="${LIVOX_TOPIC:-/livox/points}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/intel/D435i/depth}"

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if ros2 topic list | grep -Fxq "${topic}"; then
      echo "[start_nartech] Found topic: ${topic}"
      return 0
    fi
    if (( "$(date +%s)" - start_s > timeout_s )); then
      echo "[start_nartech] Warning: timeout waiting for ${topic}"
      return 1
    fi
    sleep 1
  done
}

if [[ "${MODE}" == "metta" ]]; then
  START_NARTECH_NODE=false
  if command -v geany >/dev/null 2>&1; then
    geany "${REPO_ROOT}/demos/${DEMO_FILE}" &
  fi
  gnome-terminal -- bash -c "sleep 4 && cd '${REPO_ROOT}' && python3 main.py ./demos/${DEMO_FILE}; exec bash" &
fi

echo "[start_nartech] Waiting for Unitree ROS2 topics..."
wait_for_topic "/tf" 30 || true
wait_for_topic "${LIVOX_TOPIC}" 30 || true
wait_for_topic "${DEPTH_TOPIC}" 30 || true

echo "[start_nartech] Launching G1 bringup stack..."
ros2 launch nartech_ros g1_nartech_bringup.launch.py \
  use_sim_time:=True \
  start_nartech_node:="${START_NARTECH_NODE}" \
  enable_arm_controller:=False
