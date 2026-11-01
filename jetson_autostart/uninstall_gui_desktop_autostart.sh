#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  TARGET_USER="${SUDO_USER:-jetson}"
else
  TARGET_USER="${USER}"
fi

TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
DESKTOP_FILE="${TARGET_HOME}/.config/autostart/rm_brainco_orbbec_demo.desktop"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo so the installer can remove another user's autostart file if needed." >&2
  echo "Run: sudo ./uninstall_gui_desktop_autostart.sh" >&2
  exit 1
fi

rm -f "${DESKTOP_FILE}"
echo "Removed GUI desktop autostart file:"
echo "  ${DESKTOP_FILE}"
