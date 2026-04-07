#!/bin/bash

set -euo pipefail

# 电机明显移动演示脚本（PP 快写模式）
# 假设can_driver节点已启动，电机ID 22 (0x16)

MOTOR_ID=22
JOINT_NAME="rotary_table"
CAN_DEV="can0"
NS="/can_driver_node"
MOTOR_CAN_HEX="$(printf '%03X' "$MOTOR_ID")"
APPLY_LIMIT_PRESET="${APPLY_LIMIT_PRESET:-0}"
HARD_RESET_PHYSICAL_LIMITS="${HARD_RESET_PHYSICAL_LIMITS:-1}"

echo "检查 pp_fast_write_enabled ..."
PP_FAST_WRITE="$(rosparam get /can_driver_node/pp_fast_write_enabled 2>/dev/null || echo false)"
echo "pp_fast_write_enabled=${PP_FAST_WRITE}"
if [[ "${PP_FAST_WRITE}" != "true" ]]; then
	echo "[WARN] 当前未开启快写模式。请在 can_driver.yaml 设置 pp_fast_write_enabled: true 并重启节点。"
fi

publish_position_and_check_fast_write() {
	local target_pos="$1"
	local dump_file
	local parsed_file
	dump_file="$(mktemp /tmp/pp_fast_write.XXXXXX.log)"
	parsed_file="$(mktemp /tmp/pp_fast_write_parsed.XXXXXX.log)"

	echo "发送位置命令: ${target_pos} rad"
	timeout 4s stdbuf -oL candump "${CAN_DEV}" > "${dump_file}" 2>/dev/null &
	local cap_pid=$!

	sleep 0.35
	# 快写无应答，重复下发几次以提高总线繁忙时的成功率。
	for _ in 1 2 3; do
		rostopic pub -1 "${NS}/motor/${JOINT_NAME}/cmd_position" std_msgs/Float64 "data: ${target_pos}" >/dev/null
		sleep 0.08
	done
	sleep 1.2

	kill "${cap_pid}" >/dev/null 2>&1 || true
	wait "${cap_pid}" >/dev/null 2>&1 || true

	awk -v id="${MOTOR_CAN_HEX}" '$2==id && $4=="05" && ($5=="09" || $5=="0A") {print NR, $5}' "${dump_file}" > "${parsed_file}" || true
	local first09 first0A
	first09="$(awk '$2=="09" {print $1; exit}' "${parsed_file}" || true)"
	first0A="$(awk '$2=="0A" {print $1; exit}' "${parsed_file}" || true)"

	# 兼容判据：某些抓包环境看不到主站TX(05/01)，但能看到从站对 0x01 的写返回 0x02。
	local ack_file ack09 ack0A
	ack_file="$(mktemp /tmp/pp_write_ack_parsed.XXXXXX.log)"
	awk -v id="${MOTOR_CAN_HEX}" '$2==id && $4=="02" && ($5=="09" || $5=="0A") {print NR, $5}' "${dump_file}" > "${ack_file}" || true
	ack09="$(awk '$2=="09" {print $1; exit}' "${ack_file}" || true)"
	ack0A="$(awk '$2=="0A" {print $1; exit}' "${ack_file}" || true)"

	if [[ -n "${first09}" && -n "${first0A}" && "${first09}" -lt "${first0A}" ]]; then
		echo "[OK] 快写顺序正确（先 05 09，再 05 0A）"
		awk -v id="${MOTOR_CAN_HEX}" '$2==id && $4=="05" && ($5=="09" || $5=="0A") {print}' "${dump_file}" | head -n 6
	elif [[ -n "${ack09}" && -n "${ack0A}" && "${ack09}" -lt "${ack0A}" ]]; then
		echo "[OK] 检测到写返回顺序（先 02 09，再 02 0A），命令链路有效。"
		awk -v id="${MOTOR_CAN_HEX}" '$2==id && $4=="02" && ($5=="09" || $5=="0A") {print}' "${dump_file}" | head -n 8
	else
		echo "[WARN] 未检测到正确的 05 09->05 0A 顺序（可能抓包环境不可见TX）"
		tail -n 20 "${dump_file}" || true
	fi
	rm -f "${dump_file}"
	rm -f "${parsed_file}"
	rm -f "${ack_file}"
}

get_motor_position_raw() {
	timeout 5 rostopic echo "${NS}/motor_states" 2>/dev/null | awk -v n="${JOINT_NAME}" '
		$1=="name:" {
			name=$2;
			gsub(/"/, "", name);
		}
		$1=="position:" && name==n {
			print $2;
			exit;
		}
	'
}

show_position_delta() {
	local before after label
	label="$1"
	before="${2:-}"
	after="${3:-}"
	if [[ -z "${before}" || -z "${after}" ]]; then
		echo "[WARN] ${label}: 未能读取完整位置数据，跳过位移计算。"
		return 0
	fi
	local delta=$(( after - before ))
	echo "[INFO] ${label}: 位置变化(raw)=${delta} (before=${before}, after=${after})"
}

get_pp_write_counters() {
	local fast normal
	fast="$(rosparam get "${NS}/pp_fast_write_sent_count" 2>/dev/null || echo 0)"
	normal="$(rosparam get "${NS}/pp_normal_write_sent_count" 2>/dev/null || echo 0)"
	# rosparam 可能返回浮点字符串（如 7778.0），先归一化为整数，避免 bash 算术报错。
	fast="$(awk -v v="${fast}" 'BEGIN{printf "%d", (v+0)}')"
	normal="$(awk -v v="${normal}" 'BEGIN{printf "%d", (v+0)}')"
	echo "${fast} ${normal}"
}

show_pp_write_counter_delta() {
	local label="$1"
	local bf_fast="$2"
	local bf_normal="$3"
	local af_fast="$4"
	local af_normal="$5"
	local dfast=$(( af_fast - bf_fast ))
	local dnormal=$(( af_normal - bf_normal ))
	echo "[INFO] ${label}: 快写计数增量=${dfast}, 普通写计数增量=${dnormal}"
	if [[ "${dfast}" -gt 0 ]]; then
		echo "[OK] 已确认快写路径实际执行（无需依赖抓包可见TX）。"
	fi
}

prepare_position_mode_motion() {
	echo "[INFO] 预驱动：先切速度模式并给小速度，再回位置模式。"
	rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: 3, value: 1.0}" >/dev/null || true
	for _ in 1 2 3; do
		rostopic pub -1 "${NS}/motor/${JOINT_NAME}/cmd_velocity" std_msgs/Float64 "data: 1.0" >/dev/null || true
		sleep 0.12
	done
	sleep 0.25
	rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: 3, value: 0.0}" >/dev/null || true
}

reset_physical_limits_raw_pp() {
	if [[ "${HARD_RESET_PHYSICAL_LIMITS}" != "1" ]]; then
		echo "[INFO] 跳过物理限位硬复位（HARD_RESET_PHYSICAL_LIMITS=${HARD_RESET_PHYSICAL_LIMITS}）。"
		return 0
	fi
	if ! command -v cansend >/dev/null 2>&1; then
		echo "[WARN] 未找到 cansend，无法执行物理限位硬复位。"
		return 0
	fi

	echo "[INFO] 执行PP物理限位硬复位（0x38/0x39/0x3A/0x3B）："
	# 0x38: 限位使能=0（关闭）
	cansend "${CAN_DEV}" "${MOTOR_CAN_HEX}#0138000000000000" || true
	# 0x39: 上限=0x7FFFFFFF
	cansend "${CAN_DEV}" "${MOTOR_CAN_HEX}#01397FFFFFFF0000" || true
	# 0x3A: 下限=0x80000000
	cansend "${CAN_DEV}" "${MOTOR_CAN_HEX}#013A800000000000" || true
	# 0x3B: 零偏=0
	cansend "${CAN_DEV}" "${MOTOR_CAN_HEX}#013B000000000000" || true
	sleep 0.2
}

widen_limits_if_available() {
	if [[ "${APPLY_LIMIT_PRESET}" != "1" ]]; then
		echo "[INFO] 跳过限位预设（APPLY_LIMIT_PRESET=${APPLY_LIMIT_PRESET}）。"
		return 0
	fi
	if rosservice list | grep -q "${NS}/set_zero_limit"; then
		echo "[INFO] 预设宽限位(-6.28~6.28 rad)，避免历史限位影响演示。"
		rosservice call "${NS}/set_zero_limit" "motor_id: ${MOTOR_ID}
zero_offset_rad: 0.0
use_current_position_as_zero: false
min_position_rad: -6.28
max_position_rad: 6.28
use_urdf_limits: false
apply_to_motor: true" >/dev/null || true
	else
		echo "[INFO] 未发现 set_zero_limit 服务，跳过限位预设。"
	fi
}

echo "使能电机..."
echo "先执行恢复（若无故障则无影响）..."
rosservice call "${NS}/recover" "{motor_id: ${MOTOR_ID}}" >/dev/null 2>&1 || true
rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: 0, value: 0.0}"

reset_physical_limits_raw_pp
widen_limits_if_available
prepare_position_mode_motion

echo "设置位置模式..."
rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: 3, value: 0.0}"

echo "读取初始位置..."
POS0="$(get_motor_position_raw 2>/dev/null || true)"
if [[ -z "${POS0}" ]]; then
	echo "[WARN] 无法读取初始位置（不影响演示继续）。"
else
	echo "[INFO] 初始位置(raw)=${POS0}"
fi

echo "移动到 π 弧度 (约180度)..."
read -r FW0 NW0 < <(get_pp_write_counters)
publish_position_and_check_fast_write 3.14159
read -r FW1 NW1 < <(get_pp_write_counters)
show_pp_write_counter_delta "第一段命令" "${FW0}" "${NW0}" "${FW1}" "${NW1}"

echo "等待5秒观察移动..."
sleep 5

echo "检查当前位置..."
POS1="$(get_motor_position_raw 2>/dev/null || true)"
show_position_delta "第一段运动" "${POS0:-}" "${POS1:-}"
timeout 3 rostopic echo "${NS}/motor_states" | grep -A5 "name: \"${JOINT_NAME}\"" || true

echo "回到 0 弧度..."
read -r FW2 NW2 < <(get_pp_write_counters)
publish_position_and_check_fast_write 0.0
read -r FW3 NW3 < <(get_pp_write_counters)
show_pp_write_counter_delta "回零段命令" "${FW2}" "${NW2}" "${FW3}" "${NW3}"

echo "等待5秒观察返回..."
sleep 5

echo "最终位置检查..."
POS2="$(get_motor_position_raw 2>/dev/null || true)"
show_position_delta "回零段运动" "${POS1:-}" "${POS2:-}"
timeout 3 rostopic echo "${NS}/motor_states" | grep -A5 "name: \"${JOINT_NAME}\"" || true

echo "失能电机..."
rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: 1, value: 0.0}"

echo "演示完成。"
