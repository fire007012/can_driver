#!/usr/bin/env bash
set -euo pipefail

# 自动发现 ECB 电机并回填标准 can_driver.yaml 的 ECB_AUTO_PROBE 标记块。
# 用法：
#   scripts/discover_ecb_and_enable_yaml.sh [index] [--setup-logtool]
# 说明：
# - index 从 1 开始，默认 1，表示选择第几台被发现的电机。
# - 脚本会构建并运行 SDK 示例 01_lookupActuators。

SELECT_INDEX="1"
SETUP_LOGTOOL="0"
YAML_PATHS=("config/can_driver.yaml")
SDK_EXAMPLE_DIR="lib/innfos-cpp-sdk/example"
SDK_BUILD_DIR="${SDK_EXAMPLE_DIR}/build"
SDK_LIB_DIR="lib/innfos-cpp-sdk/sdk/lib/linux_x86_64"
LOOKUP_BIN="${SDK_BUILD_DIR}/bin/01_lookupActuators"
LOGTOOL_DIR="lib/innfos-cpp-sdk/tools/logtool/linux_x86_64"
LOGTOOL_SET_PATH="${LOGTOOL_DIR}/setpath.sh"
LOGTOOL_BIN="${LOGTOOL_DIR}/ActuatorLogTool"

log() {
  echo "[ECB-DISCOVER] $*"
}

err() {
  echo "[ECB-DISCOVER][ERR] $*" >&2
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    err "missing command: ${cmd}"
    exit 2
  fi
}

print_usage() {
  cat <<'EOF'
Usage:
  scripts/discover_ecb_and_enable_yaml.sh [index] [--setup-logtool]

Options:
  index            选择第几台被发现的电机（从 1 开始，默认 1）
  --setup-logtool  自动执行 SDK 要求的 setpath.sh（需要 sudo）
  -h, --help       显示帮助
EOF
}

parse_args() {
  for arg in "$@"; do
    case "${arg}" in
      --setup-logtool)
        SETUP_LOGTOOL="1"
        ;;
      -h|--help)
        print_usage
        exit 0
        ;;
      *)
        if [[ "${arg}" =~ ^[0-9]+$ ]]; then
          SELECT_INDEX="${arg}"
        else
          err "unknown argument: ${arg}"
          print_usage
          exit 2
        fi
        ;;
    esac
  done
}

maybe_setup_logtool() {
  if [[ "${SETUP_LOGTOOL}" != "1" ]]; then
    return
  fi
  if [[ ! -f "${LOGTOOL_BIN}" ]]; then
    err "logtool binary not found: ${LOGTOOL_BIN}"
    exit 2
  fi
  log "run robust logtool setup with sudo (group/capability)"
  sudo groupadd -f pcap
  sudo usermod -a -G pcap "${USER}"
  sudo chgrp pcap "${LOGTOOL_BIN}"
  sudo chmod 750 "${LOGTOOL_BIN}"
  sudo setcap cap_net_raw,cap_net_admin=eip "${LOGTOOL_BIN}"
  log "logtool setup done; if this is first setup, run 'newgrp pcap' or re-login before retry"
}

prepare_logtool_runtime() {
  if [[ ! -d "${LOGTOOL_DIR}" ]]; then
    err "logtool dir not found: ${LOGTOOL_DIR}"
    exit 2
  fi
  if [[ ! -f "${LOGTOOL_BIN}" ]]; then
    err "logtool binary not found: ${LOGTOOL_BIN}"
    exit 2
  fi

  # SDK 依赖该变量定位 logtool。
  export ACTUATOR_TOOL_PATH="$(cd "${LOGTOOL_DIR}" && pwd)"
  chmod +x "${LOGTOOL_BIN}" >/dev/null 2>&1 || true

  if command -v getcap >/dev/null 2>&1; then
    local caps
    caps="$(getcap "${LOGTOOL_BIN}" 2>/dev/null || true)"
    if [[ "${caps}" != *"cap_net_raw"* ]] || [[ "${caps}" != *"cap_net_admin"* ]]; then
      log "warning: ${LOGTOOL_BIN} missing cap_net_raw/cap_net_admin; discovery may fail."
      log "hint: sudo sh ${LOGTOOL_SET_PATH} && newgrp pcap"
    fi
  fi
}

clean_sdk_build_dir() {
  if [[ ! -d "${SDK_BUILD_DIR}" ]]; then
    return
  fi
  log "reset stale SDK build dir: ${SDK_BUILD_DIR}"
  rm -rf "${SDK_BUILD_DIR}"
}

ensure_sdk_build_dir_matches_workspace() {
  local cache_file="${SDK_BUILD_DIR}/CMakeCache.txt"
  if [[ ! -f "${cache_file}" ]]; then
    return
  fi

  local expected_source expected_build cached_source cached_build
  expected_source="$(cd "${SDK_EXAMPLE_DIR}" && pwd)"
  expected_build="${expected_source}/build"
  cached_source="$(sed -n 's#^CMAKE_HOME_DIRECTORY:INTERNAL=##p' "${cache_file}" | head -n1)"
  cached_build="$(sed -n 's#^CMAKE_CACHEFILE_DIR:INTERNAL=##p' "${cache_file}" | head -n1)"

  if [[ -n "${cached_source}" && "${cached_source}" != "${expected_source}" ]]; then
    log "detected foreign CMake source cache: ${cached_source}"
    clean_sdk_build_dir
    return
  fi

  if [[ -n "${cached_build}" && "${cached_build}" != "${expected_build}" ]]; then
    log "detected foreign CMake build cache: ${cached_build}"
    clean_sdk_build_dir
  fi
}

build_lookup_tool() {
  ensure_sdk_build_dir_matches_workspace

  if cmake -S "${SDK_EXAMPLE_DIR}" -B "${SDK_BUILD_DIR}" >/dev/null; then
    :
  else
    err "cmake configure failed for ${SDK_BUILD_DIR}; retry with a clean build dir."
    clean_sdk_build_dir
    cmake -S "${SDK_EXAMPLE_DIR}" -B "${SDK_BUILD_DIR}" >/dev/null
  fi

  cmake --build "${SDK_BUILD_DIR}" --target 01_lookupActuators -j >/dev/null
}

handle_lookup_failure() {
  local raw="$1"
  if printf '%s' "${raw}" | grep -qi "please run logtool setting script"; then
    err "SDK requires logtool setup before discovery."
    err "run: sudo sh ${LOGTOOL_SET_PATH}"
    err "then re-login or run: newgrp pcap"
    err "retry: scripts/discover_ecb_and_enable_yaml.sh ${SELECT_INDEX}"
    exit 4
  fi
}

print_network_hint() {
  local raw="${1:-}"
  local iface_ip
  iface_ip="$(ip -4 -brief addr 2>/dev/null | awk '$1 != "lo" && $2 == "UP" && $3 != "" {print $1" "$3; exit}')"
  if [[ -n "${iface_ip}" ]]; then
    local iface ip_cidr
    iface="$(printf '%s' "${iface_ip}" | awk '{print $1}')"
    ip_cidr="$(printf '%s' "${iface_ip}" | awk '{print $2}')"
    err "network hint: active iface with IPv4=${iface}, ip=${ip_cidr}."
  else
    local up_no_ip
    up_no_ip="$(ip -brief link 2>/dev/null | awk '$1 != "lo" && $2 == "UP" {print $1; exit}')"
    if [[ -n "${up_no_ip}" ]]; then
      err "network hint: active iface=${up_no_ip}, ipv4=none."
    fi
  fi

  if command -v systemd-detect-virt >/dev/null 2>&1; then
    local virt
    virt="$(systemd-detect-virt 2>/dev/null || true)"
    if [[ -n "${virt}" && "${virt}" != "none" ]]; then
      err "virtualization hint: running inside '${virt}'."
      err "if VM network is NAT, ECB L2 broadcast discovery may be blocked."
      err "switch VM NIC to bridged mode or use host machine for discovery."
    fi
  fi

  local suggested_nic
  if [[ -n "${raw}" ]]; then
    suggested_nic="$(printf '%s\n' "${raw}" | awk '/^try open device / {print $4}' | while read -r nic; do
      [[ -z "${nic}" || "${nic}" == "lo" || "${nic}" == "any" ]] && continue
      if [[ -z "$(ip -4 -brief addr show dev "${nic}" 2>/dev/null | awk '{print $3}')" ]]; then
        echo "${nic}"
        break
      fi
    done)"
  fi

  if [[ -z "${suggested_nic}" ]]; then
    suggested_nic="$(ip -brief link 2>/dev/null | awk '
      $1 != "lo" && $1 ~ /^ens|^enp|^eth/ {print $1}
    ' | while read -r nic; do
      if [[ -z "$(ip -4 -brief addr show dev "${nic}" 2>/dev/null | awk '{print $3}')" ]]; then
        echo "${nic}"
        break
      fi
    done)"
  fi

  if [[ -z "${suggested_nic}" ]]; then
    suggested_nic="$(ip route 2>/dev/null | awk '/^default / {print $5; exit}')"
  fi

  if [[ -n "${suggested_nic}" ]]; then
    err "recommended nic for ECB subnet alias: ${suggested_nic}"
  fi

  err "if ECB uses 192.168.1.x default subnet, add a temporary alias then retry:"
  if [[ -n "${suggested_nic}" ]]; then
    err "  sudo ip addr add 192.168.1.100/24 dev ${suggested_nic}"
  else
    err "  sudo ip addr add 192.168.1.100/24 dev <your_nic>"
  fi
  err "  scripts/discover_ecb_and_enable_yaml.sh ${SELECT_INDEX}"
  err "cleanup alias after test:"
  if [[ -n "${suggested_nic}" ]]; then
    err "  sudo ip addr del 192.168.1.100/24 dev ${suggested_nic}"
  else
    err "  sudo ip addr del 192.168.1.100/24 dev <your_nic>"
  fi
}

print_permission_hint() {
  if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx "pcap"; then
    return
  fi
  err "permission hint: current shell is not in 'pcap' group."
  err "run: newgrp pcap   (or re-login)"
}

require_cmd cmake
require_cmd python3

parse_args "$@"

for yaml_path in "${YAML_PATHS[@]}"; do
  if [[ ! -f "${yaml_path}" ]]; then
    err "yaml not found: ${yaml_path}"
    exit 2
  fi
done

if [[ ! -d "${SDK_EXAMPLE_DIR}" ]]; then
  err "sdk example dir not found: ${SDK_EXAMPLE_DIR}"
  exit 2
fi

if ! [[ "${SELECT_INDEX}" =~ ^[0-9]+$ ]] || [[ "${SELECT_INDEX}" -lt 1 ]]; then
  err "index must be a positive integer, got: ${SELECT_INDEX}"
  exit 2
fi

maybe_setup_logtool
prepare_logtool_runtime

log "build lookup tool"
build_lookup_tool

if [[ ! -x "${LOOKUP_BIN}" ]]; then
  err "lookup binary not found: ${LOOKUP_BIN}"
  exit 2
fi

if [[ ! -d "${SDK_LIB_DIR}" ]]; then
  err "sdk lib dir not found: ${SDK_LIB_DIR}"
  exit 2
fi

log "scan online ECB actuators"
LOOKUP_OUT="$(LD_LIBRARY_PATH="${SDK_LIB_DIR}:${LD_LIBRARY_PATH:-}" ACTUATOR_TOOL_PATH="${ACTUATOR_TOOL_PATH}" "${LOOKUP_BIN}" || true)"

if [[ -z "${LOOKUP_OUT}" ]]; then
  err "lookup returned empty output. Please check network and power."
  exit 3
fi

handle_lookup_failure "${LOOKUP_OUT}"

PARSED="$(printf '%s\n' "${LOOKUP_OUT}" | awk '/Actuator ID:/ {
  id=""; ip="";
  for (i=1; i<=NF; ++i) {
    if ($i == "ID:") { id=$(i+1); }
    if ($i == "address:") { ip=$(i+1); }
  }
  if (id != "" && ip != "") { print id" "ip; }
}')"

if [[ -z "${PARSED}" ]]; then
  handle_lookup_failure "${LOOKUP_OUT}"
  print_permission_hint
    if printf '%s' "${LOOKUP_OUT}" | grep -q "No available device"; then
      print_network_hint "${LOOKUP_OUT}"
  fi
  err "no actuator found by SDK lookup. raw output: ${LOOKUP_OUT}"
  exit 3
fi

COUNT="$(printf '%s\n' "${PARSED}" | wc -l | awk '{print $1}')"
if [[ "${SELECT_INDEX}" -gt "${COUNT}" ]]; then
  err "index ${SELECT_INDEX} out of range, discovered ${COUNT} actuator(s)."
  printf '%s\n' "${PARSED}" | nl -ba | sed 's/^/[ECB-DISCOVER] /'
  exit 3
fi

SELECTED="$(printf '%s\n' "${PARSED}" | sed -n "${SELECT_INDEX}p")"
MOTOR_ID="$(printf '%s' "${SELECTED}" | awk '{print $1}')"
IP_ADDR="$(printf '%s' "${SELECTED}" | awk '{print $2}')"

log "selected actuator: id=${MOTOR_ID}, ip=${IP_ADDR}"

printf '%s\n' "${PARSED}" | sort -n | sed 's/^/[ECB-DISCOVER] discovered /'

python3 - "${PARSED}" "${YAML_PATHS[@]}" <<'PY'
import pathlib
import re
import sys

parsed_lines = [line.strip() for line in sys.argv[1].splitlines() if line.strip()]
yaml_paths = [pathlib.Path(p) for p in sys.argv[2:]]

entries = []
for line in parsed_lines:
    motor_id_text, ip_addr = line.split()
    entries.append((int(motor_id_text, 10), ip_addr))

entries = sorted(dict(entries).items())

begin = "# ECB_AUTO_PROBE_BEGIN"
end = "# ECB_AUTO_PROBE_END"
pattern = re.compile(rf"{re.escape(begin)}.*?{re.escape(end)}", re.S)

block_lines = [begin]
for motor_id, ip_addr in entries:
    block_lines.extend(
        [
            f"    - name: ecb_joint_{motor_id:02d}",
            "      # 由 scripts/discover_ecb_and_enable_yaml.sh 自动回填（固定 IP）",
            f"      motor_id: 0x{motor_id:02X}",
            "      protocol: ECB",
            f"      can_device: ecb://{ip_addr}",
            f"      ecb_ip: {ip_addr}",
            "      ecb_discovery: fixed",
            "      ecb_refresh_ms: 20",
            "      control_mode: position",
            "      position_scale: 6.283185307179586e-4",
            "      velocity_scale: 1.0471975511965978e-2",
        ]
    )
block_lines.append(f"    {end}")
block = "\n".join(block_lines)

for yaml_path in yaml_paths:
    text = yaml_path.read_text(encoding="utf-8")
    if not pattern.search(text):
        raise SystemExit(f"ECB auto-probe marker block not found in yaml: {yaml_path}")
    updated = pattern.sub(block, text, count=1)
    yaml_path.write_text(updated, encoding="utf-8")
PY

for yaml_path in "${YAML_PATHS[@]}"; do
  log "yaml updated: ${yaml_path}"
done
log "next: keep only the required joints in config/can_driver.yaml for this machine"
log "next: roslaunch can_driver can_driver.launch"
log "then: scripts/test_ecb_motor_motion.sh 2 0.8 2.0 1.2 2.0 /can_driver_node"
log "repeat for more motors: scripts/test_ecb_motor_motion.sh 3|4|5 0.8 2.0 1.2 2.0 /can_driver_node"
