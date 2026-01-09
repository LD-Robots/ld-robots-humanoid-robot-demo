#!/usr/bin/env bash
set -euo pipefail

# Base = folderul unde este scriptul
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"

TARGET_HOME="${HOME}"
if [ "${SUDO_USER-}" != "" ] && [ "${HOME}" = "/root" ]; then
  TARGET_HOME="/home/${SUDO_USER}"
fi

link_file() {
  local src_rel="$1"
  local dst_rel="$2"
  local src="$BASE_DIR/$src_rel"
  local dst="$dst_rel"

  if [ ! -f "$src" ]; then
    echo "Missing source file: $src"
    exit 1
  fi

  mkdir -p "$(dirname "$dst")"

  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    local ts
    ts="$(date +%Y%m%d-%H%M%S)"
    cp "$dst" "${dst}.bak.${ts}"
    echo "Backup: ${dst}.bak.${ts}"
  fi

  ln -sf "$src" "$dst"
  echo "Linked $dst -> $src"
}

link_file "../config/mc_rtc.yaml" "$TARGET_HOME/.config/mc_rtc/mc_rtc.yaml"
link_file "../config/mc_mujoco.yaml" "$TARGET_HOME/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml"
link_file "../config/G1CoMPosture.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/G1CoMPosture.yaml"
link_file "../config/G1.yaml" "/usr/local/share/mc_mujoco/G1.yaml"
link_file "../config/g1.json" "$TARGET_HOME/.config/mc_rtc/robots/g1.json"
link_file "../config/g1.rsdf" "$TARGET_HOME/.config/mc_rtc/robots/rsdf/G1/g1.rsdf"
