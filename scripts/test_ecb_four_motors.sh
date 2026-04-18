#!/usr/bin/env bash
set -euo pipefail

# ECB 四电机顺序联调脚本
#
# 默认按 2,3,4,5 顺序调用 scripts/test_ecb_motor_motion.sh，
# 方便在 can_driver_ecb.yaml 已配置四台 ECB 电机时做整包联调。
#
# 用法：
#   bash scripts/test_ecb_four_motors.sh [vel_rad_s] [vel_hold_s] [pos_rad] [pos_hold_s] [driver_ns]
# 示例：
#   bash scripts/test_ecb_four_motors.sh 0.8 2.0 1.2 2.0 /can_driver_node
#
# 可选环境变量：
#   ECB_TEST_MOTOR_IDS="2 3 4 5"   # 覆盖默认测试顺序
#   ECB_TEST_PAUSE_SEC=1.0         # 两台电机之间的停顿

VEL_CMD="${1:-0.8}"
VEL_HOLD_SEC="${2:-2.0}"
POS_CMD="${3:-1.2}"
POS_HOLD_SEC="${4:-2.0}"
DRIVER_NS="${5:-/can_driver_node}"
MOTOR_IDS_TEXT="${ECB_TEST_MOTOR_IDS:-2 3 4 5}"
INTER_MOTOR_PAUSE_SEC="${ECB_TEST_PAUSE_SEC:-1.0}"

log() {
  echo "[ECB-TEST-ALL] $*"
}

err() {
  echo "[ECB-TEST-ALL][ERR] $*" >&2
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    err "missing command: ${cmd}"
    exit 2
  fi
}

require_cmd bash
require_cmd rosservice

if ! rosservice list >/dev/null 2>&1; then
  err "ROS master unavailable. Please run roscore/bringup first."
  exit 2
fi

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SINGLE_TEST_SCRIPT="${SCRIPT_DIR}/test_ecb_motor_motion.sh"
if [[ ! -x "${SINGLE_TEST_SCRIPT}" ]]; then
  err "single motor test script not executable: ${SINGLE_TEST_SCRIPT}"
  exit 2
fi

IFS=' ' read -r -a MOTOR_IDS <<< "${MOTOR_IDS_TEXT}"
if [[ "${#MOTOR_IDS[@]}" -eq 0 ]]; then
  err "no motor ids configured. Set ECB_TEST_MOTOR_IDS or pass a non-empty default list."
  exit 2
fi

PASSED=()
FAILED=()

log "driver_ns=${DRIVER_NS}, motors=${MOTOR_IDS_TEXT}"
log "per-motor plan: vel=${VEL_CMD} rad/s for ${VEL_HOLD_SEC}s, pos_offset=${POS_CMD} rad for ${POS_HOLD_SEC}s"

for motor_id in "${MOTOR_IDS[@]}"; do
  log "==== start motor_id=${motor_id} ===="
  if bash "${SINGLE_TEST_SCRIPT}" "${motor_id}" "${VEL_CMD}" "${VEL_HOLD_SEC}" "${POS_CMD}" "${POS_HOLD_SEC}" "${DRIVER_NS}"; then
    PASSED+=("${motor_id}")
    log "motor_id=${motor_id} passed"
  else
    FAILED+=("${motor_id}")
    err "motor_id=${motor_id} failed"
  fi
  sleep "${INTER_MOTOR_PAUSE_SEC}"
done

log "summary: passed=${PASSED[*]:-none}, failed=${FAILED[*]:-none}"
if [[ "${#FAILED[@]}" -gt 0 ]]; then
  exit 1
fi
