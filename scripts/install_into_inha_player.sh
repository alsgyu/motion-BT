#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
inha_soccer_root="${1:-/home/booster/Workspace/INHA-Soccer}"
player_root="$inha_soccer_root/INHA-Player"
patch_file="$repo_root/patches/inha-player-custom-motion.patch"
upgrade_patch_files=(
  "$repo_root/patches/inha-player-head-motion-upgrade.patch"
  "$repo_root/patches/inha-player-kick-mode-upgrade.patch"
  "$repo_root/patches/inha-player-kick-precommand-grace-upgrade.patch"
  "$repo_root/patches/inha-player-soccer-head-reclaim-upgrade.patch"
  "$repo_root/patches/inha-player-soccer-head-reacquire-upgrade.patch"
)

if [ ! -d "$player_root/src/brain" ]; then
  echo "[ERROR] INHA-Player brain package not found: $player_root/src/brain" >&2
  echo "usage: $0 /home/booster/Workspace/INHA-Soccer" >&2
  exit 1
fi

if ! git -C "$inha_soccer_root" rev-parse --show-toplevel >/dev/null 2>&1; then
  echo "[ERROR] Not a git repository: $inha_soccer_root" >&2
  exit 1
fi

apply_upgrade_patch() {
  local upgrade_patch_file="$1"
  local upgrade_patch_name
  upgrade_patch_name="$(basename "$upgrade_patch_file")"

  if git -C "$inha_soccer_root" apply --reverse --check "$upgrade_patch_file" >/dev/null 2>&1; then
    echo "[PATCH] $upgrade_patch_name already applied"
  elif git -C "$inha_soccer_root" apply --check "$upgrade_patch_file" >/dev/null 2>&1; then
    echo "[PATCH] applying $upgrade_patch_name"
    git -C "$inha_soccer_root" apply "$upgrade_patch_file"
  else
    if upgrade_patch_already_integrated "$upgrade_patch_name"; then
      echo "[PATCH] $upgrade_patch_name already integrated"
    else
      echo "[ERROR] Could not apply $upgrade_patch_name automatically." >&2
      echo "        Your INHA-Soccer tree may have local changes in the same files." >&2
      echo "        Check with: git -C $inha_soccer_root status --short" >&2
      exit 1
    fi
  fi
}

upgrade_patch_already_integrated() {
  local upgrade_patch_name="$1"

  case "$upgrade_patch_name" in
    inha-player-head-motion-upgrade.patch)
      grep -q "customMotionHeadTopic" "$player_root/src/brain/include/brain_config.h" &&
        grep -q "publishCustomMotionHead" "$player_root/src/brain/src/robot_client.cpp"
      ;;
    inha-player-kick-mode-upgrade.patch)
      grep -q "RLKick waiting for kSoccer before VisualKick" "$player_root/src/brain/src/kick.cpp" &&
        grep -q 'robocup_mode_target", "soccer"' "$player_root/src/brain/src/walk.cpp"
      ;;
    inha-player-kick-precommand-grace-upgrade.patch)
      grep -q "visualKickPreCommandGrace" "$player_root/src/brain/src/striker_decision.cpp" &&
        grep -q "visualKickPreCommandGrace" "$player_root/src/brain/src/setpiece.cpp" &&
        grep -q "moveHead(0.7, 0.0)" "$player_root/src/brain/src/kick.cpp"
      ;;
    inha-player-soccer-head-reclaim-upgrade.patch)
      grep -q "soccer_head_reclaim_msec" "$player_root/src/brain/include/kick.h" &&
        (grep -q "RLKick kSoccer confirmed; reclaiming head before VisualKick" "$player_root/src/brain/src/kick.cpp" ||
          grep -q "RLKick kSoccer confirmed; head-only ball reacquire before VisualKick" "$player_root/src/brain/src/kick.cpp") &&
        grep -q "customMotionPostKickGrace" "$player_root/src/brain/src/kick.cpp"
      ;;
    inha-player-soccer-head-reacquire-upgrade.patch)
      grep -q "soccer_head_reacquire_msec" "$player_root/src/brain/include/kick.h" &&
        grep -q "stepSoccerHeadReacquire" "$player_root/src/brain/src/kick.cpp" &&
        grep -q "head-only ball reacquire before VisualKick" "$player_root/src/brain/src/kick.cpp"
      ;;
    *)
      return 1
      ;;
  esac
}

if git -C "$inha_soccer_root" apply --check "$patch_file" >/dev/null 2>&1; then
  echo "[PATCH] applying custom motion patch"
  git -C "$inha_soccer_root" apply "$patch_file"
else
  echo "[PATCH] custom motion base patch already present or partially present"
fi

for upgrade_patch_file in "${upgrade_patch_files[@]}"; do
  apply_upgrade_patch "$upgrade_patch_file"
done

echo "[MOTION] copying jy_walk_001 runtime folder"
mkdir -p "$player_root/motions/jy_walk_001"
cp -R "$repo_root/motions/jy_walk_001/." "$player_root/motions/jy_walk_001/"
chmod +x "$player_root/motions/jy_walk_001/start_bt_custom_motion.sh"
chmod +x "$player_root/motions/jy_walk_001/booster_deploy_overlay/run_bt_cmd_vel.py"

echo "[DONE] Build INHA-Player with: cd $player_root && colcon build --symlink-install"
