#!/usr/bin/env bash
set -euo pipefail

# MT 运动测试：速度模式 + MIT 位置模式（增强版：MIT 往返位移更明显）
# 用法：
#   bash scripts/test_mt_motor_motion.sh [profile] [motor_id|auto] [vel] [vel_duration] [pos_rad] [mit_hold_sec]
# 示例：
#   bash scripts/test_mt_motor_motion.sh car_a 0x141 5.0 2.0 2.5 2.0
#   bash scripts/test_mt_motor_motion.sh car_a auto 5.0 2.0 2.5 2.0

PROFILE="${1:-car_a}"
MOTOR_ID_INPUT="${2:-auto}"
VEL="${3:-5.0}"
VEL_DURATION="${4:-2.0}"
POS_RAD="${5:-2.5}"
MIT_HOLD_SEC="${6:-2.0}"
MIT_STREAM_HZ="${MT_TEST_MIT_STREAM_HZ:-20}"
MIT_VERIFY_POS_RAD="${MT_TEST_MIT_VERIFY_POS_RAD:-2.5}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MT_IF="${SCRIPT_DIR}/mt_motor_interface.py"

if [[ ! -x "${MT_IF}" ]]; then
  echo "[ERR] 未找到可执行接口脚本: ${MT_IF}"
  echo "请先执行: chmod +x scripts/mt_motor_interface.py"
  exit 2
fi

run_if() {
  python3 "${MT_IF}" --profile "${PROFILE}" --motor-id "${MOTOR_ID}" "$@"
}

run_if_motor() {
  local motor_id="$1"
  shift
  python3 "${MT_IF}" --profile "${PROFILE}" --motor-id "${motor_id}" "$@"
}

send_mit_position_hold() {
  local target_rad="$1"
  local hold_sec="$2"

  local step_sec
  step_sec="$(python3 - <<PY
hz=float('${MIT_STREAM_HZ}')
print(1.0/max(1.0,hz))
PY
)"

  python3 - <<PY
import time, subprocess, shlex
target='${target_rad}'
hold=float('${hold_sec}')
step=float('${step_sec}')
cmd="python3 ${MT_IF} --profile ${PROFILE} --motor-id ${MOTOR_ID} --action position --value " + target
cmd += " --no-auto-mode"
end=time.time()+max(0.0, hold)
while time.time()<end:
    subprocess.run(shlex.split(cmd), check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(step)
# 保底再打一帧，避免边界时刻丢最后目标
subprocess.run(shlex.split(cmd), check=True)
PY
}

select_motor_id() {
  local req_id="${MOTOR_ID_INPUT}"
  local candidates=(0x141 0x14B)

  if [[ "${req_id}" != "auto" && "${req_id}" != "AUTO" ]]; then
    if run_if_motor "${req_id}" --action enable >/dev/null 2>&1; then
      run_if_motor "${req_id}" --action stop >/dev/null 2>&1 || true
      echo "${req_id}"
      return 0
    fi
    echo "[MT-TEST] 指定 motor_id=${req_id} 不可用，按优先级回退探测 0x141 -> 0x14B" >&2
  fi

  local id
  for id in "${candidates[@]}"; do
    if run_if_motor "${id}" --action enable >/dev/null 2>&1; then
      run_if_motor "${id}" --action stop >/dev/null 2>&1 || true
      echo "${id}"
      return 0
    fi
  done

  return 1
}

if ! MOTOR_ID="$(select_motor_id)"; then
  echo "[MT-TEST] 未找到可用 MT 电机ID（已优先探测 0x141，再探测 0x14B）" >&2
  echo "[MT-TEST] 可先执行: rosrun can_driver mt_motor_interface.py --profile ${PROFILE} --action list" >&2
  exit 3
fi

echo "[MT-TEST] profile=${PROFILE}, motor=${MOTOR_ID}"

echo "[1/9] Enable"
run_if --action enable

echo "[2/9] Velocity mode"
run_if --action mode --value 1

echo "[3/9] Velocity +${VEL}"
run_if --action velocity --value "${VEL}"
sleep "${VEL_DURATION}"

echo "[4/9] Velocity -${VEL}"
run_if --action velocity --value "-$(python3 - <<PY
v=float('${VEL}')
print(abs(v))
PY
)"
sleep "${VEL_DURATION}"

echo "[5/9] 速度归零（验证位置模式前必须停转）"
run_if --action mode --value 1
run_if --action velocity --value 0
sleep 1.0

echo "[6/9] Stop（保险）"
run_if --action stop
sleep 0.5

echo "[7/9] Position mode(MIT)"
run_if --action mode --value 0
sleep 0.2

echo "[8/9] MIT 位置 +${POS_RAD} rad"
send_mit_position_hold "${POS_RAD}" "${MIT_HOLD_SEC}"

echo "[9/9] MIT 位置 -${POS_RAD} rad"
NEG_POS="-$(python3 - <<PY
p=float('${POS_RAD}')
print(abs(p))
PY
)"
send_mit_position_hold "${NEG_POS}" "${MIT_HOLD_SEC}"

echo "[10/9] MIT 回零 (0 rad)"
send_mit_position_hold "0" "${MIT_HOLD_SEC}"

echo "[MIT-VERIFY] 强测: 重复置位模式 + 固定大位移 ${MIT_VERIFY_POS_RAD} rad"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "${MIT_VERIFY_POS_RAD}" "${MIT_HOLD_SEC}"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "-${MIT_VERIFY_POS_RAD}" "${MIT_HOLD_SEC}"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "0" "${MIT_HOLD_SEC}"

echo "[End] Final stop (保持可继续调试)"
run_if --action stop

if [[ "${MT_TEST_SEND_DISABLE:-0}" == "1" ]]; then
  echo "[End+] Disable (按需)"
  run_if --action disable
fi

echo "[MT-TEST] 完成"
