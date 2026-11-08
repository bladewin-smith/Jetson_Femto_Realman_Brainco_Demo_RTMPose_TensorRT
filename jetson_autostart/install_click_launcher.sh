#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="rm_brainco_orbbec_demo.service"
ENV_DIR="/etc/rm_brainco_orbbec_demo"
ENV_FILE="${ENV_DIR}/rm_brainco_orbbec_demo.env"
LAUNCHER="/usr/local/bin/rm_brainco_orbbec_demo"
DESKTOP_TEMPLATE="${SCRIPT_DIR}/rm65_vision_control.desktop.in"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "This installer must be run with sudo." >&2
  echo "Run: sudo ./jetson_autostart/install_click_launcher.sh" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-jetson}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
TARGET_GROUP="$(id -gn "${TARGET_USER}")"
if [[ -z "${TARGET_HOME}" || ! -d "${TARGET_HOME}" ]]; then
  echo "Could not resolve the home directory for ${TARGET_USER}." >&2
  exit 1
fi

TARGET_DESKTOP="$(runuser -u "${TARGET_USER}" -- xdg-user-dir DESKTOP 2>/dev/null || true)"
if [[ -z "${TARGET_DESKTOP}" ]]; then
  TARGET_DESKTOP="${TARGET_HOME}/Desktop"
fi
APPLICATIONS_DIR="${TARGET_HOME}/.local/share/applications"
APPLICATION_FILE="${APPLICATIONS_DIR}/rm65-vision-control.desktop"
DESKTOP_FILE="${TARGET_DESKTOP}/rm65-vision-control.desktop"
OLD_AUTOSTART="${TARGET_HOME}/.config/autostart/rm_brainco_orbbec_demo.desktop"

install -d -m 0755 "${ENV_DIR}"
if [[ -f "${ENV_FILE}" ]]; then
  cp -a "${ENV_FILE}" "${ENV_FILE}.bak.${STAMP}"
  echo "Backed up the previous environment file to ${ENV_FILE}.bak.${STAMP}"
fi

# These two installs intentionally replace the previous quick-launch runtime.
install -m 0644 "${SCRIPT_DIR}/rm_brainco_orbbec_demo.env" "${ENV_FILE}"
install -m 0755 "${SCRIPT_DIR}/run_rm_brainco_orbbec_demo.sh" "${LAUNCHER}"

# Avoid a second controller being started by an old service or login autostart.
systemctl disable --now "${SERVICE_NAME}" 2>/dev/null || true
systemctl reset-failed "${SERVICE_NAME}" 2>/dev/null || true
if [[ -f "${OLD_AUTOSTART}" ]]; then
  mv "${OLD_AUTOSTART}" "${OLD_AUTOSTART}.disabled.${STAMP}"
  chown "${TARGET_USER}:${TARGET_GROUP}" "${OLD_AUTOSTART}.disabled.${STAMP}"
  echo "Disabled the previous login autostart shortcut."
fi

install -d -m 0755 -o "${TARGET_USER}" -g "${TARGET_GROUP}" \
  "${APPLICATIONS_DIR}" "${TARGET_DESKTOP}"
install -m 0755 -o "${TARGET_USER}" -g "${TARGET_GROUP}" \
  "${DESKTOP_TEMPLATE}" "${APPLICATION_FILE}"
install -m 0755 -o "${TARGET_USER}" -g "${TARGET_GROUP}" \
  "${DESKTOP_TEMPLATE}" "${DESKTOP_FILE}"

# GNOME may otherwise show the desktop file as an untrusted text file.
runuser -u "${TARGET_USER}" -- gio set "${DESKTOP_FILE}" metadata::trusted true \
  2>/dev/null || true
if command -v update-desktop-database >/dev/null 2>&1; then
  runuser -u "${TARGET_USER}" -- update-desktop-database "${APPLICATIONS_DIR}" \
    >/dev/null 2>&1 || true
fi

echo
echo "Installed the current click-to-run controller:"
echo "  launcher: ${LAUNCHER}"
echo "  settings: ${ENV_FILE}"
echo "  desktop:  ${DESKTOP_FILE}"
echo
echo "Double-click 'RM65 Vision Control'. In the camera window:"
echo "  S = start/resume"
echo "  H = return Home and pause until the next S"
echo "  Q or Esc = quit"
