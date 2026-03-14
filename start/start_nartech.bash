#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="${1:-default}"
DEMO_FILE="${2:-demo_with_nars.metta}"
DEMO_BASENAME="$(basename "${DEMO_FILE}")"
START_NARTECH_NODE=True
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
CONTRACT_FILE="${CONTRACT_FILE:-${REPO_ROOT}/config/g1_ros_contract.yaml}"
ENABLE_ARM_CONTROLLER=False

LIVOX_TOPIC="${LIVOX_TOPIC:-/livox/points}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/intel/D435i/depth}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/intel/D435i/camera_info}"
SLAM_SCAN_MODE="${SLAM_SCAN_MODE:-lidar_only}"

if [[ ! -f "${CONTRACT_FILE}" ]]; then
  echo "[start_nartech] Error: contract file not found: ${CONTRACT_FILE}"
  exit 1
fi

if [[ "${DEMO_BASENAME}" == "demo_with_nars_orangeball.metta" ]]; then
  ENABLE_ARM_CONTROLLER=True
fi

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local start_s
  local topic_list
  start_s="$(date +%s)"
  while true; do
    # Avoid piping `ros2 topic list` into `grep -q` to prevent BrokenPipeError
    # when grep exits early after finding a match.
    topic_list="$(ros2 topic list 2>/dev/null || true)"
    if [[ $'\n'"${topic_list}"$'\n' == *$'\n'"${topic}"$'\n'* ]]; then
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

wait_for_sensor_message() {
  local topic="$1"
  local timeout_s="${2:-15}"
  if timeout "${timeout_s}" ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort \
      --qos-durability volatile >/dev/null 2>&1; then
    echo "[start_nartech] Received first message on: ${topic}"
    return 0
  fi
  echo "[start_nartech] Warning: timeout waiting for first message on ${topic}"
  return 1
}

if [[ "${MODE}" == "metta" ]]; then
  START_NARTECH_NODE=False
  if command -v geany >/dev/null 2>&1; then
    geany "${REPO_ROOT}/demos/${DEMO_FILE}" &
  fi
  gnome-terminal -- bash -c "sleep 4 && cd '${REPO_ROOT}' && ${PYTHON_BIN} main.py ./demos/${DEMO_FILE} --ros-args --params-file '${CONTRACT_FILE}' -p enable_arm_controller:=${ENABLE_ARM_CONTROLLER} -p use_sim_time:=True; exec bash" &
fi

echo "[start_nartech] Waiting for Unitree ROS2 topics..."
wait_for_topic "/tf" 30 || true
wait_for_topic "${LIVOX_TOPIC}" 30 || true
wait_for_topic "${DEPTH_TOPIC}" 30 || true
wait_for_topic "${CAMERA_INFO_TOPIC}" 30 || true
wait_for_sensor_message "${LIVOX_TOPIC}" 15 || true
wait_for_sensor_message "${DEPTH_TOPIC}" 15 || true
wait_for_sensor_message "${CAMERA_INFO_TOPIC}" 15 || true

echo "[start_nartech] Launching G1 bringup stack..."
ros2 launch nartech_ros g1_nartech_bringup.launch.py \
  contract_file:="${CONTRACT_FILE}" \
  use_sim_time:=True \
  start_nartech_node:="${START_NARTECH_NODE}" \
  enable_arm_controller:="${ENABLE_ARM_CONTROLLER}" \
  slam:=True \
  slam_scan_mode:="${SLAM_SCAN_MODE}" \
  autostart:=True \
  use_composition:=True \
  use_respawn:=False \
  use_namespace:=False
