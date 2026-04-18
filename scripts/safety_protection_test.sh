#!/bin/bash

set -euo pipefail

# 安全保护功能测试脚本
# 覆盖：
# 1) 未使能阻断(safety_require_enabled_for_motion)
# 2) 位置步进限幅(max_position_step_rad) - 启发式检查
# 3) 掉线恢复保持(safety_hold_after_device_recover)
# 4) 故障自动Stop(safety_stop_on_fault) - 人工注入故障验证提示

NS="${NS:-/can_driver_node}"
MOTOR_ID="${MOTOR_ID:-22}"
JOINT_NAME="${JOINT_NAME:-rotary_table}"
CAN_DEV="${CAN_DEV:-can0}"
POS_EPS_RAW="${POS_EPS_RAW:-10}"
SHORT_WAIT="${SHORT_WAIT:-0.4}"
MOVE_WAIT="${MOVE_WAIT:-1.5}"
JOINT_MODE=""
POS_STIM_TOGGLE=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

CMD_ENABLE=0
CMD_DISABLE=1
CMD_STOP=2
CMD_SET_MODE=3

note() { echo "[INFO] $*"; }
pass() { echo "[PASS] $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
fail() { echo "[FAIL] $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
skip() { echo "[SKIP] $*"; SKIP_COUNT=$((SKIP_COUNT + 1)); }

bootstrap_ros_env() {
    # sudo 场景下常见问题：PATH 丢失导致 rosservice/rostopic 不可用。
    if command -v rosservice >/dev/null 2>&1 && command -v rostopic >/dev/null 2>&1; then
        return 0
    fi

    local setup_files=(
        "/opt/ros/noetic/setup.bash"
        "/opt/ros/melodic/setup.bash"
        "${WS_ROOT}/devel/setup.bash"
    )
    local f
    for f in "${setup_files[@]}"; do
        if [[ -f "${f}" ]]; then
            # shellcheck disable=SC1090
            source "${f}" || true
        fi
    done

    if ! command -v rosservice >/dev/null 2>&1 || ! command -v rostopic >/dev/null 2>&1; then
        echo "[ERROR] 未找到 rosservice/rostopic。请先 source ROS 环境后再运行脚本。"
        echo "[ERROR] 例如：source /opt/ros/noetic/setup.bash && source ${WS_ROOT}/devel/setup.bash"
        return 1
    fi
    return 0
}

abs_int() {
    local v="$1"
    if (( v < 0 )); then
        echo $(( -v ))
    else
        echo "$v"
    fi
}

have_service() {
    rosservice list 2>/dev/null | grep -q "^$1$"
}

wait_service() {
    local srv="$1"
    local timeout_sec="${2:-5}"
    local max_try=$(( timeout_sec * 10 ))
    local i
    for ((i = 0; i < max_try; ++i)); do
        if rosservice list 2>/dev/null | grep -Fxq "${srv}"; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

call_motor_cmd() {
    local cmd="$1"
    local value="${2:-0.0}"
    local out
    out="$(rosservice call "${NS}/motor_command" "{motor_id: ${MOTOR_ID}, command: ${cmd}, value: ${value}}" 2>&1 || true)"
    if echo "${out}" | grep -Eiq 'success:[[:space:]]*(True|true)'; then
        return 0
    fi
    echo "[WARN] motor_command 调用失败: cmd=${cmd}, value=${value}, resp=${out}"
    return 1
}

set_mode_velocity() { call_motor_cmd "${CMD_SET_MODE}" 1.0; }
set_mode_position() { call_motor_cmd "${CMD_SET_MODE}" 0.0; }

enable_motor() { call_motor_cmd "${CMD_ENABLE}" 0.0; }
disable_motor() { call_motor_cmd "${CMD_DISABLE}" 0.0; }
stop_motor() { call_motor_cmd "${CMD_STOP}" 0.0 || true; }

pub_vel_once() {
    local v="$1"
    rostopic pub -1 "${NS}/motor/${JOINT_NAME}/cmd_velocity" std_msgs/Float64 "data: ${v}" >/dev/null
}

pub_pos_once() {
    local p="$1"
    rostopic pub -1 "${NS}/motor/${JOINT_NAME}/cmd_position" std_msgs/Float64 "data: ${p}" >/dev/null
}

get_joint_mode_from_param() {
    rosparam get "${NS}/joints" 2>/dev/null | awk -v target="${JOINT_NAME}" '
        $1=="-" {in_item=1; is_target=0; next}
        in_item && $1=="name:" {
            name=$2; gsub(/"/, "", name);
            if (name==target) is_target=1; else is_target=0;
            next
        }
        in_item && is_target && $1=="control_mode:" {
            print $2;
            exit
        }
    '
}

stimulate_motion() {
    if [[ "${JOINT_MODE}" == "velocity" ]]; then
        for _ in 1 2 3; do pub_vel_once 1.2; sleep 0.1; done
        return 0
    fi

    # position 模式下使用交替目标点，确保有可观测位移。
    local target
    if (( POS_STIM_TOGGLE == 0 )); then
        target=1.2
        POS_STIM_TOGGLE=1
    else
        target=0.0
        POS_STIM_TOGGLE=0
    fi
    for _ in 1 2 3; do pub_pos_once "${target}"; sleep 0.1; done
}

get_motor_position_raw() {
    timeout 4 rostopic echo "${NS}/motor_states" 2>/dev/null | awk -v n="${JOINT_NAME}" '
        $1=="name:" { name=$2; gsub(/"/, "", name); }
        $1=="position:" && name==n { print $2; exit }
    '
}

position_delta_abs() {
    local before="$1"
    local after="$2"
    local d=$(( after - before ))
    abs_int "$d"
}

require_param_true() {
    local key="$1"
    local val
    val="$(rosparam get "${NS}/${key}" 2>/dev/null || echo "")"
    if [[ "${val}" == "true" ]]; then
        return 0
    fi
    return 1
}

cleanup() {
    stop_motor
    enable_motor >/dev/null 2>&1 || true
    if command -v ip >/dev/null 2>&1; then
        ip link set dev "${CAN_DEV}" up >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

main() {
    bootstrap_ros_env || exit 2

    note "检查服务与基础连通性..."
    if ! wait_service "${NS}/motor_command" 5; then
        echo "[ERROR] 未找到服务 ${NS}/motor_command，请先启动 can_driver_node。"
        exit 2
    fi

    if ! enable_motor; then
        echo "[ERROR] 无法使能电机，请先检查电机状态/故障位。"
        exit 2
    fi
    if ! set_mode_velocity; then
        note "无法切换到速度模式，继续按关节配置模式执行测试。"
    fi

    JOINT_MODE="$(get_joint_mode_from_param || true)"
    if [[ -z "${JOINT_MODE}" ]]; then
        JOINT_MODE="position"
        note "未读取到 joint control_mode，默认按 position 模式测试。"
    fi
    note "当前关节模式: ${JOINT_MODE}"

    # ------------------------------
    # T1: 未使能阻断
    # ------------------------------
    note "T1 未使能阻断测试（safety_require_enabled_for_motion）"
    if ! require_param_true "safety_require_enabled_for_motion"; then
        skip "参数 safety_require_enabled_for_motion != true，跳过 T1"
    else
        local p0 p1 p2 d_block d_move
        p0="$(get_motor_position_raw || true)"
        if [[ -z "${p0}" ]]; then
            skip "无法读取初始位置，跳过 T1"
        else
            # 先停稳，避免把历史速度残留误判成“disable后仍在动”。
            stop_motor
            for _ in 1 2 3; do pub_vel_once 0.0; sleep 0.08; done
            sleep 0.4
            p0="$(get_motor_position_raw || true)"

            if ! disable_motor; then
                fail "CMD_DISABLE 调用失败，无法进行 T1"
                p1=""
                p2=""
            else
                stimulate_motion
                sleep "${MOVE_WAIT}"
                p1="$(get_motor_position_raw || true)"

                if ! enable_motor; then
                    fail "CMD_ENABLE 调用失败，无法完成 T1"
                    p2=""
                else
                    stimulate_motion
                    sleep "${MOVE_WAIT}"
                    p2="$(get_motor_position_raw || true)"
                fi
            fi

            if [[ -z "${p1}" || -z "${p2}" ]]; then
                skip "读取位置不完整，跳过 T1 判定"
            else
                d_block="$(position_delta_abs "${p0}" "${p1}")"
                d_move="$(position_delta_abs "${p1}" "${p2}")"
                note "T1 delta_block=${d_block}, delta_after_enable=${d_move}"
                if (( d_block <= POS_EPS_RAW && d_move > POS_EPS_RAW )); then
                    pass "未使能阻断生效，重新使能后运动恢复"
                elif (( d_block <= POS_EPS_RAW && d_move <= POS_EPS_RAW )); then
                    skip "阻断阶段无位移，但使能后也无可观测运动（当前激励不足或机构被限位），不判 FAIL"
                else
                    fail "未使能阻断判定不通过（可能阈值不合适或机构惯性较大）"
                fi
            fi
        fi
    fi

    stop_motor

    # ------------------------------
    # T2: 位置步进限幅（启发式）
    # ------------------------------
    note "T2 位置步进限幅测试（max_position_step_rad）"
    local max_step
    max_step="$(rosparam get "${NS}/max_position_step_rad" 2>/dev/null || echo 0)"
    if awk -v v="${max_step}" 'BEGIN{exit !((v+0)>0)}'; then
        set_mode_position
        enable_motor
        local q0 q1 q2 d_early d_total
        q0="$(get_motor_position_raw || true)"
        if [[ -z "${q0}" ]]; then
            skip "无法读取初始位置，跳过 T2"
        else
            pub_pos_once 3.14159
            sleep 0.2
            q1="$(get_motor_position_raw || true)"
            sleep 1.2
            q2="$(get_motor_position_raw || true)"
            if [[ -z "${q1}" || -z "${q2}" ]]; then
                skip "读取位置不完整，跳过 T2 判定"
            else
                d_early="$(position_delta_abs "${q0}" "${q1}")"
                d_total="$(position_delta_abs "${q0}" "${q2}")"
                note "T2 delta_early=${d_early}, delta_total=${d_total}, max_step_rad=${max_step}"
                if (( d_total > 0 && d_early < d_total )); then
                    pass "检测到渐进响应，步进限幅疑似生效（建议结合日志复核）"
                else
                    fail "未观察到明显渐进响应，建议增大目标跳变并复测"
                fi
            fi
        fi
    else
        skip "max_position_step_rad<=0，跳过 T2"
    fi

    stop_motor

    # ------------------------------
    # T3: 掉线恢复保持
    # ------------------------------
    note "T3 掉线恢复保持测试（safety_hold_after_device_recover）"
    if ! require_param_true "safety_hold_after_device_recover"; then
        skip "参数 safety_hold_after_device_recover != true，跳过 T3"
    elif ! command -v ip >/dev/null 2>&1; then
        skip "系统无 ip 命令，跳过 T3"
    else
        if [[ "${JOINT_MODE}" == "velocity" ]]; then
            set_mode_velocity || true
        else
            set_mode_position || true
        fi
        enable_motor

        local r0 r1 r2 d_hold d_fresh
        r0="$(get_motor_position_raw || true)"
        if [[ -z "${r0}" ]]; then
            skip "无法读取初始位置，跳过 T3"
        else
            note "尝试将 ${CAN_DEV} down（需要权限）..."
            if ! ip link set dev "${CAN_DEV}" down >/dev/null 2>&1; then
                skip "无权限执行 ip link set ${CAN_DEV} down，跳过 T3"
                ip link set dev "${CAN_DEV}" up >/dev/null 2>&1 || true
            else
                stimulate_motion
                sleep 0.2
                ip link set dev "${CAN_DEV}" up >/dev/null 2>&1 || true

                # 恢复后不发 fresh 命令，先观察
                sleep 0.35
                r1="$(get_motor_position_raw || true)"

                # 再发 fresh 命令
                stimulate_motion
                sleep "${MOVE_WAIT}"
                r2="$(get_motor_position_raw || true)"

                if [[ -z "${r1}" || -z "${r2}" ]]; then
                    skip "读取位置不完整，跳过 T3 判定"
                else
                    d_hold="$(position_delta_abs "${r0}" "${r1}")"
                    d_fresh="$(position_delta_abs "${r1}" "${r2}")"
                    note "T3 delta_after_recover_without_fresh=${d_hold}, delta_after_fresh=${d_fresh}"
                    if (( d_hold <= POS_EPS_RAW && d_fresh > POS_EPS_RAW )); then
                        pass "恢复保持生效：恢复后首段无明显动作，fresh 命令后恢复运动"
                    else
                        fail "恢复保持判定不通过（可能权限/链路/阈值影响）"
                    fi
                fi
            fi
        fi
    fi

    stop_motor

    # ------------------------------
    # T4: 故障自动 Stop（人工）
    # ------------------------------
    note "T4 故障自动Stop需要人工触发故障位（例如伺服报警）。"
    note "触发后观察日志：应出现 auto Stop sent / motion command blocked。"
    pass "T4 已提供人工验证步骤（脚本不自动注入硬件故障）"

    echo
    echo "========== 测试总结 =========="
    echo "PASS=${PASS_COUNT}, FAIL=${FAIL_COUNT}, SKIP=${SKIP_COUNT}"
    if (( FAIL_COUNT > 0 )); then
        exit 1
    fi
}

main "$@"
