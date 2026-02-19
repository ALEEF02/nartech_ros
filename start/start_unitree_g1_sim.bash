#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UNITREE_ROOT="${UNITREE_ROOT:-${REPO_ROOT}/../../../unitree_rl_gym}"

if [[ ! -d "${UNITREE_ROOT}" ]]; then
  echo "[start_unitree_g1_sim] unitree_rl_gym not found at: ${UNITREE_ROOT}"
  echo "[start_unitree_g1_sim] Set UNITREE_ROOT to the unitree_rl_gym directory."
  exit 1
fi

cd "${UNITREE_ROOT}"
python3 deploy/deploy_mujoco/deploy_mujoco.py g1.yaml --headless --mapping-mode

