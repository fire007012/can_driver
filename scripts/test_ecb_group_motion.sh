#!/usr/bin/env bash
set -euo pipefail

# ECB 分组同时联调脚本
#
# 默认按两组电机顺序测试：
#   group A: 2,3
#   group B: 4,5
# 每组内部电机会同时执行：
#   Enable -> 速度模式 -> 同步正反速度 -> Stop -> 位置模式 -> 同步相对位置往返 -> Final stop
#
# 用法：
#   bash scripts/test_ecb_group_motion.sh [vel_rad_s] [vel_hold_s] [pos_rad] [pos_hold_s] [driver_ns]
#
# 示例：
#   bash scripts/test_ecb_group_motion.sh 8.0 2.0 12.0 2.0 /can_driver_node
#
# 可选环境变量：
#   ECB_TEST_GROUPS="2,3 4,5"      # 组定义，空格分组、逗号分隔组内 motor_id
#   ECB_TEST_GROUP_PAUSE_SEC=1.0   # 组与组之间的停顿
#   ECB_TEST_STREAM_HZ=20          # 组内同步下发频率

VEL_CMD="${1:-8.0}"
VEL_HOLD_SEC="${2:-2.0}"
POS_CMD="${3:-12.0}"
POS_HOLD_SEC="${4:-2.0}"
DRIVER_NS="${5:-/can_driver_node}"
GROUPS_TEXT="${ECB_TEST_GROUPS:-2,3 4,5}"
GROUP_PAUSE_SEC="${ECB_TEST_GROUP_PAUSE_SEC:-1.0}"
GROUP_STREAM_HZ="${ECB_TEST_STREAM_HZ:-20}"
MOTOR_SRV="${DRIVER_NS}/motor_command"
MOTOR_STATE_TOPIC="${DRIVER_NS}/motor_states"

log() {
  echo "[ECB-GROUP-TEST] $*"
}

err() {
  echo "[ECB-GROUP-TEST][ERR] $*" >&2
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    err "missing command: ${cmd}"
    exit 2
  fi
}

require_cmd rosservice
require_cmd rosparam
require_cmd python3

if ! rosservice list >/dev/null 2>&1; then
  err "ROS master unavailable. Please run roscore/bringup first."
  exit 2
fi

resolve_group_joints() {
  local group_spec="$1"
  python3 - "$group_spec" "$DRIVER_NS" <<'PY'
import subprocess
import sys
import yaml

group_spec = sys.argv[1]
ns = sys.argv[2].rstrip('/')
param_name = f"{ns}/joints" if ns else "/joints"
ids = [int(x, 0) for x in group_spec.split(',') if x.strip()]

raw = subprocess.check_output(["rosparam", "get", param_name], text=True)
joints = yaml.safe_load(raw)
if not isinstance(joints, list):
    raise SystemExit("invalid joints parameter")

mapping = {}
for item in joints:
    if not isinstance(item, dict):
        continue
    if str(item.get("protocol", "")).upper() != "ECB":
        continue
    motor_id = item.get("motor_id")
    if isinstance(motor_id, str):
        motor_id = int(motor_id, 0)
    elif isinstance(motor_id, int):
        motor_id = int(motor_id)
    else:
        continue
    mapping[motor_id] = (
        str(item.get("name", "")),
        float(item.get("position_scale", 1.0)),
    )

for motor_id in ids:
    if motor_id not in mapping:
        raise SystemExit(f"missing ECB joint for motor_id={motor_id}")
    name, pos_scale = mapping[motor_id]
    print(f"{motor_id} {name} {pos_scale}")
PY
}

call_motor_cmd() {
  local motor_id="$1"
  local command="$2"
  local value="$3"
  rosservice call "${MOTOR_SRV}" "{motor_id: ${motor_id}, command: ${command}, value: ${value}}" >/dev/null
}

wait_motor_state_equals() {
  local motor_id="$1"
  local field_name="$2"
  local expected="$3"
  local timeout_sec="${4:-3.0}"
  python3 - "$motor_id" "$field_name" "$expected" "$timeout_sec" "$MOTOR_STATE_TOPIC" <<'PY'
import subprocess
import sys
import time

motor_id = sys.argv[1]
field = sys.argv[2]
expected = sys.argv[3]
timeout_sec = float(sys.argv[4])
topic = sys.argv[5]
deadline = time.time() + timeout_sec

while time.time() < deadline:
    proc = subprocess.run(
        ["timeout", "2s", "rostopic", "echo", topic],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    current_id = ""
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "motor_id:":
            current_id = parts[1]
            continue
        if current_id == motor_id and len(parts) >= 2 and parts[0] == f"{field}:":
            if parts[1] == expected:
                sys.exit(0)
            break
    time.sleep(0.1)

sys.exit(1)
PY
}

fetch_group_positions() {
  local group_info="$1"
  python3 - "$group_info" "$MOTOR_STATE_TOPIC" <<'PY'
import subprocess
import sys
import time

group_info = sys.argv[1].strip().splitlines()
topic = sys.argv[2]
targets = {}
for line in group_info:
    motor_id, joint_name, pos_scale = line.split()
    targets[motor_id] = {"joint": joint_name, "scale": float(pos_scale), "position": None}

deadline = time.time() + 5.0
while time.time() < deadline:
    proc = subprocess.run(
        ["timeout", "2s", "rostopic", "echo", topic],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    current_id = ""
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "motor_id:":
            current_id = parts[1]
            continue
        if current_id in targets and len(parts) >= 2 and parts[0] == "position:":
            targets[current_id]["position"] = int(parts[1], 0)
    if all(v["position"] is not None for v in targets.values()):
        break
    time.sleep(0.1)

for motor_id in sorted(targets, key=lambda x: int(x)):
    entry = targets[motor_id]
    if entry["position"] is None:
        raise SystemExit(f"missing position for motor_id={motor_id}")
    print(f"{motor_id} {entry['joint']} {entry['scale']} {entry['position']}")
PY
}

publish_group_targets() {
  local mode="$1"
  local hold_sec="$2"
  local payload="$3"
  python3 - "$mode" "$hold_sec" "$payload" "$DRIVER_NS" "$GROUP_STREAM_HZ" <<'PY'
import sys
import time

import rospy
from std_msgs.msg import Float64

mode = sys.argv[1]
hold_sec = max(0.0, float(sys.argv[2]))
payload_lines = [line.strip() for line in sys.argv[3].splitlines() if line.strip()]
driver_ns = sys.argv[4].rstrip('/')
hz = max(1.0, float(sys.argv[5]))

anonymous = True
if not rospy.core.is_initialized():
    rospy.init_node("ecb_group_test_pub", anonymous=anonymous, disable_signals=True)

publishers = []
messages = []
for line in payload_lines:
    _, joint_name, value = line.split()
    suffix = "cmd_velocity" if mode == "velocity" else "cmd_position"
    topic = f"{driver_ns}/motor/{joint_name}/{suffix}" if driver_ns else f"/motor/{joint_name}/{suffix}"
    pub = rospy.Publisher(topic, Float64, queue_size=1)
    publishers.append(pub)
    msg = Float64()
    msg.data = float(value)
    messages.append(msg)

start_wait = time.time()
while time.time() - start_wait < 1.0 and not rospy.is_shutdown():
    if all(pub.get_num_connections() > 0 for pub in publishers):
        break
    rospy.sleep(0.05)

rate = rospy.Rate(max(1.0, hz))
deadline = time.time() + hold_sec
while time.time() < deadline and not rospy.is_shutdown():
    for pub, msg in zip(publishers, messages):
        pub.publish(msg)
    rate.sleep()

for pub, msg in zip(publishers, messages):
    pub.publish(msg)
PY
}

run_group_test() {
  local group_spec="$1"
  local group_info
  group_info="$(resolve_group_joints "${group_spec}")"
  log "group=${group_spec}"
  printf '%s\n' "${group_info}" | sed 's/^/[ECB-GROUP-TEST] member /'

  while read -r motor_id _joint_name _pos_scale; do
    call_motor_cmd "${motor_id}" 0 0.0
    if wait_motor_state_equals "${motor_id}" "enabled" "True" 3.0; then
      log "motor_id=${motor_id} enable confirmed"
    else
      log "motor_id=${motor_id} enable not confirmed within timeout"
    fi
  done <<< "${group_info}"

  while read -r motor_id _joint_name _pos_scale; do
    call_motor_cmd "${motor_id}" 3 1.0
    if wait_motor_state_equals "${motor_id}" "mode" "2" 3.0; then
      log "motor_id=${motor_id} velocity mode confirmed"
    else
      log "motor_id=${motor_id} velocity mode not confirmed within timeout"
    fi
  done <<< "${group_info}"

  local velocity_payload=""
  while read -r motor_id joint_name _pos_scale; do
    velocity_payload+="${motor_id} ${joint_name} ${VEL_CMD}"$'\n'
  done <<< "${group_info}"
  log "group=${group_spec} velocity +${VEL_CMD} rad/s"
  publish_group_targets "velocity" "${VEL_HOLD_SEC}" "${velocity_payload}"

  local neg_velocity_payload=""
  while read -r motor_id joint_name _pos_scale; do
    neg_velocity_payload+="${motor_id} ${joint_name} -${VEL_CMD#-}"$'\n'
  done <<< "${group_info}"
  log "group=${group_spec} velocity -${VEL_CMD#-} rad/s"
  publish_group_targets "velocity" "${VEL_HOLD_SEC}" "${neg_velocity_payload}"

  local zero_velocity_payload=""
  while read -r motor_id joint_name _pos_scale; do
    zero_velocity_payload+="${motor_id} ${joint_name} 0.0"$'\n'
  done <<< "${group_info}"
  log "group=${group_spec} velocity zero"
  publish_group_targets "velocity" "0.2" "${zero_velocity_payload}"

  while read -r motor_id _joint_name _pos_scale; do
    call_motor_cmd "${motor_id}" 2 0.0
  done <<< "${group_info}"
  sleep 0.5

  while read -r motor_id _joint_name _pos_scale; do
    call_motor_cmd "${motor_id}" 3 0.0
    if wait_motor_state_equals "${motor_id}" "mode" "1" 3.0; then
      log "motor_id=${motor_id} position mode confirmed"
    else
      log "motor_id=${motor_id} position mode not confirmed within timeout"
    fi
  done <<< "${group_info}"

  local group_positions
  group_positions="$(fetch_group_positions "${group_info}")"

  local pos_plus_payload=""
  local pos_minus_payload=""
  local pos_home_payload=""
  while read -r motor_id joint_name pos_scale position_raw; do
    local current_rad target_plus target_minus
    current_rad="$(python3 - "$position_raw" "$pos_scale" <<'PY'
import sys
print(float(sys.argv[1]) * float(sys.argv[2]))
PY
)"
    target_plus="$(python3 - "$current_rad" "$POS_CMD" <<'PY'
import sys
print(float(sys.argv[1]) + abs(float(sys.argv[2])))
PY
)"
    target_minus="$(python3 - "$current_rad" "$POS_CMD" <<'PY'
import sys
print(float(sys.argv[1]) - abs(float(sys.argv[2])))
PY
)"
    pos_plus_payload+="${motor_id} ${joint_name} ${target_plus}"$'\n'
    pos_minus_payload+="${motor_id} ${joint_name} ${target_minus}"$'\n'
    pos_home_payload+="${motor_id} ${joint_name} ${current_rad}"$'\n'
  done <<< "${group_positions}"

  log "group=${group_spec} position +${POS_CMD} rad relative"
  publish_group_targets "position" "${POS_HOLD_SEC}" "${pos_plus_payload}"

  log "group=${group_spec} position -${POS_CMD} rad relative"
  publish_group_targets "position" "${POS_HOLD_SEC}" "${pos_minus_payload}"

  log "group=${group_spec} position back to start"
  publish_group_targets "position" "${POS_HOLD_SEC}" "${pos_home_payload}"

  while read -r motor_id _joint_name _pos_scale; do
    call_motor_cmd "${motor_id}" 2 0.0
  done <<< "${group_info}"

  while read -r motor_id _joint_name _pos_scale; do
    log "group=${group_spec} final state motor_id=${motor_id}"
    timeout 3s rostopic echo "${MOTOR_STATE_TOPIC}" 2>/dev/null | awk -v id="${motor_id}" '
      /^motor_id:/ { current_id=$2; block=$0 ORS; next }
      current_id != "" {
        block=block $0 ORS
        if ($0 == "---") {
          if (current_id == id) {
            printf "%s", block
            exit
          }
          current_id=""
          block=""
        }
      }
    ' || true
  done <<< "${group_info}"
}

PASSED=()
FAILED=()
TEST_GROUPS=()
IFS=' ' read -r -a TEST_GROUPS <<< "${GROUPS_TEXT}"

log "driver_ns=${DRIVER_NS}, groups=${GROUPS_TEXT}"
log "group plan: vel=${VEL_CMD} rad/s for ${VEL_HOLD_SEC}s, pos_offset=${POS_CMD} rad for ${POS_HOLD_SEC}s, stream_hz=${GROUP_STREAM_HZ}"

for group_spec in "${TEST_GROUPS[@]}"; do
  log "==== start group=${group_spec} ===="
  if run_group_test "${group_spec}"; then
    PASSED+=("${group_spec}")
    log "group=${group_spec} passed"
  else
    FAILED+=("${group_spec}")
    err "group=${group_spec} failed"
  fi
  sleep "${GROUP_PAUSE_SEC}"
done

log "summary: passed=${PASSED[*]:-none}, failed=${FAILED[*]:-none}"
if [[ "${#FAILED[@]}" -gt 0 ]]; then
  exit 1
fi
