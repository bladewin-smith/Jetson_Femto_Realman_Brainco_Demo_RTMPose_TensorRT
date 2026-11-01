#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="rm_brainco_orbbec_demo.service"
ENV_DIR="/etc/rm_brainco_orbbec_demo"
ENV_FILE="${ENV_DIR}/rm_brainco_orbbec_demo.env"
LAUNCHER="/usr/local/bin/rm_brainco_orbbec_demo"

if [[ "${EUID}" -eq 0 ]]; then
  TARGET_USER="${SUDO_USER:-jetson}"
else
  TARGET_USER="${USER}"
fi

TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
if [[ -z "${TARGET_HOME}" || ! -d "${TARGET_HOME}" ]]; then
  echo "Could not resolve home directory for user ${TARGET_USER}" >&2
  exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "This installer needs sudo to update /usr/local/bin and /etc." >&2
  echo "Run: sudo ./install_gui_desktop_autostart.sh" >&2
  exit 1
fi

install -d -m 0755 "${ENV_DIR}"
if [[ ! -f "${ENV_FILE}" ]]; then
  install -m 0644 "${SCRIPT_DIR}/rm_brainco_orbbec_demo.env" "${ENV_FILE}"
fi

sed -i 's/^SHOW_WINDOW=.*/SHOW_WINDOW=true/' "${ENV_FILE}"
sed -i 's/^AUTO_START_ON_READY=.*/AUTO_START_ON_READY=false/' "${ENV_FILE}"

install -m 0755 "${SCRIPT_DIR}/run_rm_brainco_orbbec_demo.sh" "${LAUNCHER}"

systemctl disable --now "${SERVICE_NAME}" 2>/dev/null || true
systemctl reset-failed "${SERVICE_NAME}" 2>/dev/null || true

AUTOSTART_DIR="${TARGET_HOME}/.config/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/rm_brainco_orbbec_demo.desktop"
install -d -m 0755 -o "${TARGET_USER}" -g "${TARGET_USER}" "${AUTOSTART_DIR}"
install -m 0644 -o "${TARGET_USER}" -g "${TARGET_USER}" \
  "${SCRIPT_DIR}/rm_brainco_orbbec_demo.desktop.in" "${DESKTOP_FILE}"

echo "Installed GUI desktop autostart for user ${TARGET_USER}:"
echo "  ${DESKTOP_FILE}"
echo
echo "The background system service has been disabled to avoid duplicate robot control."
echo "The GUI will start after the ${TARGET_USER} desktop session logs in."
echo
echo "Current important env values:"
grep -n "SHOW_WINDOW\|AUTO_START_ON_READY\|APP_ROOT\|CONTROL_SCRIPT" "${ENV_FILE}" || true
