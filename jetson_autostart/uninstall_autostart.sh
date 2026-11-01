#!/usr/bin/env bash
set -Eeuo pipefail

SERVICE_NAME="rm_brainco_orbbec_demo.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"
LAUNCHER="/usr/local/bin/rm_brainco_orbbec_demo"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo: sudo ./uninstall_autostart.sh" >&2
  exit 1
fi

systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
rm -f "${SERVICE_FILE}"
rm -f "${LAUNCHER}"
systemctl daemon-reload

echo "Removed ${SERVICE_NAME}."
echo "The env file is kept at /etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env."
