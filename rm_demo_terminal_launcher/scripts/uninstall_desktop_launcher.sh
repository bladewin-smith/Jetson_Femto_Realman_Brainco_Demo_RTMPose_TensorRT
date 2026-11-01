#!/usr/bin/env bash
set -euo pipefail

LAUNCHER_BIN="/usr/local/bin/rm_brainco_orbbec_demo_terminal"
DESKTOP_FILE="$HOME/.local/share/applications/rm_brainco_orbbec_demo.desktop"

rm -f "$DESKTOP_FILE"
if [[ -f "$LAUNCHER_BIN" ]]; then
  sudo rm -f "$LAUNCHER_BIN"
fi

echo "[INFO] Uninstalled launcher and desktop shortcut."
