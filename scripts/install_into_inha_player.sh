#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
inha_soccer_root="${1:-/home/booster/Workspace/INHA-Soccer}"
player_root="$inha_soccer_root/INHA-Player"
patch_file="$repo_root/patches/inha-player-custom-motion.patch"

if [ ! -d "$player_root/src/brain" ]; then
  echo "[ERROR] INHA-Player brain package not found: $player_root/src/brain" >&2
  echo "usage: $0 /home/booster/Workspace/INHA-Soccer" >&2
  exit 1
fi

if ! git -C "$inha_soccer_root" rev-parse --show-toplevel >/dev/null 2>&1; then
  echo "[ERROR] Not a git repository: $inha_soccer_root" >&2
  exit 1
fi

if git -C "$inha_soccer_root" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  echo "[PATCH] custom motion patch already applied"
else
  echo "[PATCH] applying custom motion patch"
  git -C "$inha_soccer_root" apply --check "$patch_file"
  git -C "$inha_soccer_root" apply "$patch_file"
fi

echo "[MOTION] copying jy_walk_001 runtime folder"
mkdir -p "$player_root/motions/jy_walk_001"
cp -R "$repo_root/motions/jy_walk_001/." "$player_root/motions/jy_walk_001/"
chmod +x "$player_root/motions/jy_walk_001/start_bt_custom_motion.sh"
chmod +x "$player_root/motions/jy_walk_001/booster_deploy_overlay/run_bt_cmd_vel.py"

echo "[DONE] Build INHA-Player with: cd $player_root && colcon build --symlink-install"
