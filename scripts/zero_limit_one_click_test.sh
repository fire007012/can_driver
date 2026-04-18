#!/usr/bin/env bash

# 一次性联调脚本：零点+限位功能验证
# 默认按 docs/零点限位功能测试方案.md 覆盖核心场景（T1/T2/T3/T4/T5/T7）

set -u

NS="${NS:-/can_driver_node}"
MOTOR_ID="${MOTOR_ID:-22}"
CAN_DEV="${CAN_DEV:-can0}"
JOINT_NAME="${JOINT_NAME:-rotary_table}"
MOTOR_CAN_HEX="$(printf "%03X" "$MOTOR_ID")"

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0
TX_VISIBLE=0

TMP_DIR="$(mktemp -d /tmp/zero_limit_test.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT

log() { echo "[INFO] $*"; }
pass() { echo "[PASS] $*"; PASS_COUNT=$((PASS_COUNT+1)); }
fail() { echo "[FAIL] $*"; FAIL_COUNT=$((FAIL_COUNT+1)); }
skip() { echo "[SKIP] $*"; SKIP_COUNT=$((SKIP_COUNT+1)); }

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少命令: $1"
    exit 2
  fi
}

svc_call_set_zero_limit() {
  local req="$1"
  rosservice call "${NS}/set_zero_limit" "$req"
}

extract_success() {
  grep -E "^success:" | awk '{print $2}'
}

extract_value() {
  local key="$1"
  grep -E "^${key}:" | awk '{print $2}'
}

capture_can_around_call() {
  local req="$1"
  local out_file="$2"

  timeout 4s stdbuf -oL candump "$CAN_DEV" >"$out_file" 2>/dev/null &
  local cap_pid=$!
  # 给 candump 启动留一点时间，避免首帧丢失
  sleep 0.35

  svc_call_set_zero_limit "$req" >"${out_file}.svc" 2>&1 || true

  sleep 1.2
  kill "$cap_pid" >/dev/null 2>&1 || true
  wait "$cap_pid" >/dev/null 2>&1 || true
}

has_limit_frames_for_motor() {
  local can_log="$1"
  grep -Eiq "${MOTOR_CAN_HEX}.* 01 (38|39|3A|3B) " "$can_log"
}

probe_tx_visibility() {
  local out_file="$1"
  timeout 3s stdbuf -oL candump "$CAN_DEV" >"$out_file" 2>/dev/null &
  local cap_pid=$!
  sleep 0.35

  # 触发主站发命令：切模式 + 位置命令
  rosservice call "${NS}/motor_command" "motor_id: ${MOTOR_ID}
command: 3
value: 0.0" >/dev/null 2>&1 || true
  rostopic pub -1 "${NS}/motor/${JOINT_NAME}/cmd_position" std_msgs/Float64 "data: 0.0" >/dev/null 2>&1 || true

  sleep 1.0
  kill "$cap_pid" >/dev/null 2>&1 || true
  wait "$cap_pid" >/dev/null 2>&1 || true

  # 只要看到了主站命令帧（01/03/05），就认为当前抓包环境可见TX
  if grep -Eiq "${MOTOR_CAN_HEX}.* 0(1|3|5) " "$out_file"; then
    return 0
  fi
  return 1
}

find_velocity_motor_id() {
  python3 - <<'PY'
import rospy
import sys
rospy.init_node('find_vel_joint_tmp', anonymous=True, disable_signals=True)
joints = rospy.get_param('/can_driver_node/joints', [])
for j in joints:
    if str(j.get('control_mode','')) == 'velocity':
        print(int(j.get('motor_id', 0)))
        sys.exit(0)
sys.exit(1)
PY
}

main() {
  need_cmd rosservice
  need_cmd rosparam
  need_cmd rostopic
  need_cmd candump
  need_cmd python3

  if ! rosservice list >/dev/null 2>&1; then
    echo "无法连接 ROS Master，请先启动系统。"
    exit 2
  fi

  # T1: 接口可用性
  if rosservice list | grep -q "${NS}/set_zero_limit"; then
    if rossrv show can_driver/SetZeroLimit >/dev/null 2>&1; then
      pass "T1 服务存在且类型可解析"
    else
      fail "T1 服务类型 can_driver/SetZeroLimit 不可解析"
    fi
  else
    fail "T1 服务 ${NS}/set_zero_limit 不存在"
  fi

  # 预探测：当前抓包是否可见主站发包
  PROBE_LOG="${TMP_DIR}/tx_probe.log"
  if probe_tx_visibility "$PROBE_LOG"; then
    TX_VISIBLE=1
    log "抓包环境可见主站发包（TX）"
  else
    TX_VISIBLE=0
    log "抓包环境未检测到主站发包（TX），帧级断言将以 SKIP 处理"
  fi

  # T2: 手工限位 + 物理下发（PP）
  REQ_T2="motor_id: ${MOTOR_ID}
zero_offset_rad: 0.0
min_position_rad: -1.0
max_position_rad: 1.0
use_urdf_limits: false
apply_to_motor: true"
  CAN_T2="${TMP_DIR}/t2_can.log"
  capture_can_around_call "$REQ_T2" "$CAN_T2"
  if grep -q "success: True" "${CAN_T2}.svc"; then
    if has_limit_frames_for_motor "$CAN_T2"; then
      pass "T2 成功并检测到物理限位相关寄存器写帧"
    else
      # 重试一次，降低总线繁忙导致的偶发漏检
      CAN_T2_R="${TMP_DIR}/t2_can_retry.log"
      capture_can_around_call "$REQ_T2" "$CAN_T2_R"
      if has_limit_frames_for_motor "$CAN_T2_R"; then
        pass "T2 重试后检测到物理限位相关寄存器写帧"
      elif [[ "$TX_VISIBLE" -eq 0 ]]; then
        skip "T2 服务成功，但当前抓包环境不可见TX，无法做寄存器帧级断言"
      else
        fail "T2 成功但未抓到 motor_id=${MOTOR_ID}(0x${MOTOR_CAN_HEX}) 的 0x38/0x39/0x3A/0x3B 写帧"
        echo "----- T2 svc 响应 -----"
        cat "${CAN_T2}.svc"
        echo "----- T2 抓包尾部 -----"
        tail -n 30 "$CAN_T2" || true
        echo "----- T2 重试抓包尾部 -----"
        tail -n 30 "$CAN_T2_R" || true
      fi
    fi
  else
    fail "T2 set_zero_limit 调用失败：$(tr '\n' ' ' < "${CAN_T2}.svc")"
  fi

  # T3: 仅软件生效（不下发电机）
  REQ_T3="motor_id: ${MOTOR_ID}
zero_offset_rad: 0.1
min_position_rad: -1.0
max_position_rad: 1.0
use_urdf_limits: false
apply_to_motor: false"
  CAN_T3="${TMP_DIR}/t3_can.log"
  capture_can_around_call "$REQ_T3" "$CAN_T3"
  if grep -q "success: True" "${CAN_T3}.svc"; then
    if has_limit_frames_for_motor "$CAN_T3"; then
      fail "T3 apply_to_motor=false 但仍抓到物理限位寄存器写帧"
    elif [[ "$TX_VISIBLE" -eq 0 ]]; then
      skip "T3 服务成功，但当前抓包环境不可见TX，无法做帧级断言"
    else
      pass "T3 成功且未下发物理限位寄存器"
    fi
  else
    fail "T3 set_zero_limit 调用失败：$(tr '\n' ' ' < "${CAN_T3}.svc")"
  fi

  # T4: URDF限位读取（若无robot_description则跳过）
  if rosparam get /robot_description >/dev/null 2>&1; then
    REQ_T4="motor_id: ${MOTOR_ID}
zero_offset_rad: 0.2
min_position_rad: 0.0
max_position_rad: 0.0
use_urdf_limits: true
apply_to_motor: false"
    OUT_T4="${TMP_DIR}/t4.out"
    svc_call_set_zero_limit "$REQ_T4" >"$OUT_T4" 2>&1 || true
    if grep -q "success: True" "$OUT_T4"; then
      pass "T4 URDF限位读取成功"
    else
      fail "T4 URDF限位读取失败：$(tr '\n' ' ' < "$OUT_T4")"
    fi
  else
    skip "T4 未检测到 /robot_description，跳过 URDF 限位测试"
  fi

  # T5: 当前位置匹配拦截（构造不可能包含当前位姿的范围）
  # 先读取当前位姿
  OUT_BASE="${TMP_DIR}/t5_base.out"
  REQ_BASE="motor_id: ${MOTOR_ID}
zero_offset_rad: 0.0
min_position_rad: -1000.0
max_position_rad: 1000.0
use_urdf_limits: false
apply_to_motor: false"
  svc_call_set_zero_limit "$REQ_BASE" >"$OUT_BASE" 2>&1 || true
  if grep -q "success: True" "$OUT_BASE"; then
    CUR_POS="$(extract_value current_position_rad < "$OUT_BASE")"
    if [[ -n "$CUR_POS" ]]; then
      LOW="$(python3 - <<PY
p=float('$CUR_POS')
print(p+0.5)
PY
)"
      HIGH="$(python3 - <<PY
p=float('$CUR_POS')
print(p+1.0)
PY
)"
      OUT_T5="${TMP_DIR}/t5.out"
      REQ_T5="motor_id: ${MOTOR_ID}
zero_offset_rad: 0.0
min_position_rad: ${LOW}
max_position_rad: ${HIGH}
use_urdf_limits: false
apply_to_motor: true"
      svc_call_set_zero_limit "$REQ_T5" >"$OUT_T5" 2>&1 || true
      if grep -q "success: False" "$OUT_T5" && grep -qi "outside" "$OUT_T5"; then
        pass "T5 当前位置不在限位内时已正确拒绝"
      else
        fail "T5 未触发当前位置匹配拦截：$(tr '\n' ' ' < "$OUT_T5")"
      fi
    else
      fail "T5 无法解析 current_position_rad"
    fi
  else
    fail "T5 基线读取失败：$(tr '\n' ' ' < "$OUT_BASE")"
  fi

  # T7: 非位置关节拒绝（若存在velocity关节）
  VEL_ID=""
  if VEL_ID=$(find_velocity_motor_id 2>/dev/null); then
    OUT_T7="${TMP_DIR}/t7.out"
    REQ_T7="motor_id: ${VEL_ID}
zero_offset_rad: 0.0
min_position_rad: -1.0
max_position_rad: 1.0
use_urdf_limits: false
apply_to_motor: false"
    svc_call_set_zero_limit "$REQ_T7" >"$OUT_T7" 2>&1 || true
    if grep -q "success: False" "$OUT_T7" && grep -qi "position-control" "$OUT_T7"; then
      pass "T7 velocity关节被正确拒绝"
    else
      fail "T7 velocity关节拒绝逻辑异常：$(tr '\n' ' ' < "$OUT_T7")"
    fi
  else
    skip "T7 未发现 velocity 关节，跳过"
  fi

  echo
  echo "========== 测试汇总 =========="
  echo "PASS: ${PASS_COUNT}"
  echo "FAIL: ${FAIL_COUNT}"
  echo "SKIP: ${SKIP_COUNT}"

  if [[ "$FAIL_COUNT" -gt 0 ]]; then
    exit 1
  fi
}

main "$@"
