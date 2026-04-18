#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TOOL_BIN="${SCRIPT_DIR}/ActuatorLogTool"

if [ ! -f "${TOOL_BIN}" ]; then
  echo "ActuatorLogTool not found: ${TOOL_BIN}" >&2
  exit 1
fi

echo "ACTUATOR_TOOL_PATH=${SCRIPT_DIR}"

if ! grep -q 'ACTUATOR_TOOL_PATH=' /etc/profile 2>/dev/null; then
  echo "export ACTUATOR_TOOL_PATH=${SCRIPT_DIR}" >> /etc/profile
fi

if ! getent group pcap >/dev/null 2>&1; then
  groupadd -f pcap
fi

TARGET_USER="${SUDO_USER:-${USER:-}}"
if [ -n "${TARGET_USER}" ]; then
  usermod -a -G pcap "${TARGET_USER}"
fi

chgrp pcap "${TOOL_BIN}"
chmod 750 "${TOOL_BIN}"
setcap cap_net_raw,cap_net_admin=eip "${TOOL_BIN}"

echo "logtool permissions updated for ${TOOL_BIN}"
echo "run 'newgrp pcap' or re-login before retrying discovery"
