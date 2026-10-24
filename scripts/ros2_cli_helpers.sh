#!/usr/bin/env bash

# Small command adapters shared by the hardware test scripts. They keep the
# test logic readable while making all ROS graph access use ROS2 CLI commands.

ros2_service_type() {
  case "${1##*/}" in
    motor_command) echo "can_driver/srv/MotorCommand" ;;
    recover) echo "can_driver/srv/Recover" ;;
    set_zero) echo "can_driver/srv/SetZero" ;;
    set_zero_limit) echo "can_driver/srv/SetZeroLimit" ;;
    apply_limits) echo "can_driver/srv/ApplyLimits" ;;
    init) echo "can_driver/srv/Init" ;;
    shutdown) echo "can_driver/srv/Shutdown" ;;
    enable|disable|halt|resume) echo "std_srvs/srv/Trigger" ;;
    *) return 1 ;;
  esac
}

ros2_service_call() {
  local service="$1"
  local payload="$2"
  local type
  type="$(ros2_service_type "${service}")" || {
    echo "unknown ROS2 service type: ${service}" >&2
    return 2
  }
  ros2 service call "${service}" "${type}" "${payload}"
}

ros2_topic_pub_once() {
  local topic="$1"
  local type="$2"
  local payload="$3"
  type="${type//\//\/msg\/}"
  ros2 topic pub --once "${topic}" "${type}" "${payload}"
}

ros2_param_get() {
  local full="$1"
  local node="${full%/*}"
  local key="${full##*/}"
  [[ -z "${node}" ]] && node="/"

  local output
  if output="$(ros2 param get "${node}" "${key}" 2>/dev/null)"; then
    printf '%s\n' "${output#*: }"
    return 0
  fi

  local config_file="${CAN_DRIVER_CONFIG:-}"
  if [[ -z "${config_file}" ]]; then
    config_file="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/can_driver.yaml"
  fi
  python3 - "${config_file}" "${key}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    root = yaml.safe_load(stream) or {}
node = root.get("can_driver_node", root)
value = node.get(sys.argv[2]) if isinstance(node, dict) else None
if value is None:
    raise SystemExit(1)
if isinstance(value, bool):
    print("true" if value else "false")
elif isinstance(value, (dict, list)):
    print(yaml.safe_dump(value, default_flow_style=False, sort_keys=False), end="")
else:
    print(value)
PY
}

