#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="rm_brainco_orbbec_demo.service"
ENV_DIR="/etc/rm_brainco_orbbec_demo"
ENV_FILE="${ENV_DIR}/rm_brainco_orbbec_demo.env"
LAUNCHER="/usr/local/bin/rm_brainco_orbbec_demo"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo: sudo ./install_autostart.sh" >&2
  exit 1
fi

SERVICE_USER="${SUDO_USER:-jetson}"
APP_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

install -d -m 0755 "${ENV_DIR}"
if [[ ! -f "${ENV_FILE}" ]]; then
  install -m 0644 "${SCRIPT_DIR}/rm_brainco_orbbec_demo.env" "${ENV_FILE}"
  echo "Created ${ENV_FILE}"
else
  echo "Keeping existing ${ENV_FILE}"
fi

install -m 0755 "${SCRIPT_DIR}/run_rm_brainco_orbbec_demo.sh" "${LAUNCHER}"

sed \
  -e "s|__SERVICE_USER__|${SERVICE_USER}|g" \
  -e "s|__APP_ROOT__|${APP_ROOT}|g" \
  "${SCRIPT_DIR}/rm_brainco_orbbec_demo.service.in" > "${SERVICE_FILE}"
chmod 0644 "${SERVICE_FILE}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "Installed ${SERVICE_NAME}"
echo
echo "Edit parameters:"
echo "  sudo nano ${ENV_FILE}"
echo
echo "Start now:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo
echo "Watch logs:"
echo "  journalctl -u ${SERVICE_NAME} -f"
