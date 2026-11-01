#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
INSTALL_BIN_DIR="/usr/local/bin"
DESKTOP_NAME="rm_brainco_orbbec_demo.desktop"
USER_DESKTOP_DIR="$HOME/.local/share/applications"
LAUNCHER_BIN="$INSTALL_BIN_DIR/rm_brainco_orbbec_demo_terminal"

if [[ $EUID -eq 0 ]]; then
  echo "[ERROR] Please run this script as normal user (not root)."
  exit 1
fi

mkdir -p "$BUILD_DIR"
cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" -j"$(nproc)"

if [[ ! -f "$BUILD_DIR/rm_brainco_orbbec_demo_terminal" ]]; then
  echo "[ERROR] Build failed: binary not found."
  exit 1
fi

sudo cp "$BUILD_DIR/rm_brainco_orbbec_demo_terminal" "$LAUNCHER_BIN"
sudo chmod +x "$LAUNCHER_BIN"

mkdir -p "$USER_DESKTOP_DIR"
cat > "$USER_DESKTOP_DIR/$DESKTOP_NAME" <<EOF
[Desktop Entry]
Type=Application
Name=RM BrainCo Orbbec Demo (Terminal)
Comment=Open a new terminal and run RM BrainCo + Orbbec demo
Exec=$LAUNCHER_BIN
Icon=utilities-terminal
Terminal=false
Categories=Utility;
StartupNotify=false
X-GNOME-Autostart-enabled=false
EOF

chmod +x "$USER_DESKTOP_DIR/$DESKTOP_NAME"
update-desktop-database "$USER_DESKTOP_DIR" 2>/dev/null || true

echo "[INFO] Installed binary: $LAUNCHER_BIN"
echo "[INFO] Desktop shortcut: $USER_DESKTOP_DIR/$DESKTOP_NAME"
echo "[INFO] Done."
