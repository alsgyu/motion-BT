#!/usr/bin/env bash
# ==========================================================================
# verify_patches.sh – Check which patches are applied to INHA-Player source
# ==========================================================================
# Usage: ./scripts/verify_patches.sh /home/booster/Workspace/INHA-Soccer
# ==========================================================================
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
inha_soccer_root="${1:-/home/booster/Workspace/INHA-Soccer}"
player_root="$inha_soccer_root/INHA-Player"

if [ ! -d "$player_root/src/brain" ]; then
  echo "[ERROR] INHA-Player brain package not found: $player_root/src/brain" >&2
  echo "usage: $0 /home/booster/Workspace/INHA-Soccer" >&2
  exit 1
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass=0
fail=0

check() {
  local name="$1"; shift
  if "$@"; then
    echo -e "  ${GREEN}[OK]${NC}   $name"
    ((pass++))
  else
    echo -e "  ${RED}[MISS]${NC} $name"
    ((fail++))
  fi
}

echo "=============================================="
echo " Patch Verification: $player_root"
echo "=============================================="
echo ""

# -- custom-motion (base patch) --
echo "--- custom-motion (base) ---"
check "customMotionEnable in brain_config.h" \
  grep -q "customMotionEnable" "$player_root/src/brain/include/brain_config.h"
check "publishCustomMotionVelocity in robot_client.h" \
  grep -q "publishCustomMotionVelocity" "$player_root/src/brain/include/robot_client.h"
check "geometry_msgs in CMakeLists.txt" \
  grep -q "geometry_msgs" "$player_root/src/brain/CMakeLists.txt"
check "custom_motion section in config.yaml" \
  grep -q "custom_motion:" "$player_root/src/brain/config/config.yaml"

# -- head-motion-upgrade --
echo ""
echo "--- head-motion-upgrade ---"
check "customMotionHeadTopic in brain_config.h" \
  grep -q "customMotionHeadTopic" "$player_root/src/brain/include/brain_config.h"
check "publishCustomMotionHead in robot_client.cpp" \
  grep -q "publishCustomMotionHead" "$player_root/src/brain/src/robot_client.cpp"

# -- kick-mode-upgrade --
echo ""
echo "--- kick-mode-upgrade ---"
check "RLKick waiting for kSoccer" \
  grep -q "waiting for kSoccer before VisualKick" "$player_root/src/brain/src/kick.cpp"
check "robocup_mode_target soccer" \
  grep -q 'robocup_mode_target", "soccer"' "$player_root/src/brain/src/walk.cpp"

# -- kick-precommand-grace-upgrade --
echo ""
echo "--- kick-precommand-grace-upgrade ---"
check "visualKickPreCommandGrace in striker_decision.cpp" \
  grep -q "visualKickPreCommandGrace" "$player_root/src/brain/src/striker_decision.cpp"
check "visualKickPreCommandGrace in setpiece.cpp" \
  grep -q "visualKickPreCommandGrace" "$player_root/src/brain/src/setpiece.cpp"
check "moveHead(0.7, 0.0) in kick.cpp" \
  grep -q "moveHead(0.7, 0.0)" "$player_root/src/brain/src/kick.cpp"

# -- soccer-head-reclaim-upgrade --
echo ""
echo "--- soccer-head-reclaim-upgrade ---"
check "soccer_head_reclaim_msec in kick.h" \
  grep -q "soccer_head_reclaim_msec" "$player_root/src/brain/include/kick.h"
check "kSoccer confirmed head reclaim" \
  grep -q "head-only ball reacquire before VisualKick" "$player_root/src/brain/src/kick.cpp"
check "customMotionPostKickGrace" \
  grep -q "customMotionPostKickGrace" "$player_root/src/brain/src/kick.cpp"

# -- soccer-head-reacquire-upgrade --
echo ""
echo "--- soccer-head-reacquire-upgrade ---"
check "soccer_head_reacquire_msec in kick.h" \
  grep -q "soccer_head_reacquire_msec" "$player_root/src/brain/include/kick.h"
check "stepSoccerHeadReacquire in kick.cpp" \
  grep -q "stepSoccerHeadReacquire" "$player_root/src/brain/src/kick.cpp"

# -- mode-stop-and-last-known-upgrade --
echo ""
echo "--- mode-stop-and-last-known-upgrade ---"
check "stopMotionForModeChange in robot_client.cpp" \
  grep -q "stopMotionForModeChange" "$player_root/src/brain/src/robot_client.cpp"
check "_headReacquireUsedLastKnown in kick.h" \
  grep -q "_headReacquireUsedLastKnown" "$player_root/src/brain/include/kick.h"
check "filterball.pitchToRobot in kick.cpp" \
  grep -q "filterball.pitchToRobot" "$player_root/src/brain/src/kick.cpp"

# -- smooth-stop-tight-kick-align-upgrade --
echo ""
echo "--- smooth-stop-tight-kick-align-upgrade ---"
check "70ms sleep in robot_client.cpp" \
  grep -q "std::chrono::milliseconds(70)" "$player_root/src/brain/src/robot_client.cpp"
check 'kick_dir_back_margin="0.55" in lead_striker.xml' \
  grep -q 'kick_dir_back_margin="0.55"' "$player_root/src/brain/behavior_trees/subtrees/lead_striker.xml"
check "kickTargetLateralTolerance = 0.20 in striker_decision" \
  grep -q "kickTargetLateralTolerance = 0.20" "$player_root/src/brain/src/striker_decision.cpp"
check "kickTargetLateralTolerance = 0.20 in setpiece" \
  grep -q "kickTargetLateralTolerance = 0.20" "$player_root/src/brain/src/setpiece.cpp"

# NOTE: throttle-fast-custom-motion-upgrade.patch is deprecated.
# Its features are now included in the base custom-motion.patch.
# Verify with these checks instead:

# -- custom-kick --
echo ""
echo "--- custom-kick ---"
check "customKickEnable in brain_config.h" \
  grep -q "customKickEnable" "$player_root/src/brain/include/brain_config.h"
check "publishCustomKickGoal in robot_client.cpp" \
  grep -q "publishCustomKickGoal" "$player_root/src/brain/src/robot_client.cpp"
check "custom kick goal published in kick.cpp" \
  grep -q "custom kick goal published" "$player_root/src/brain/src/kick.cpp"

# -- velocity_scale check (should be 1.0, not 1.35) --
echo ""
echo "--- velocity_scale (latest) ---"
if grep -q 'velocity_scale_x: 1.35' "$player_root/src/brain/config/config.yaml" 2>/dev/null; then
  echo -e "  ${YELLOW}[WARN]${NC} config.yaml still has velocity_scale=1.35 (old boost)"
  echo "         → start script overrides to 1.0 at runtime, but rebuild to fix source."
  ((fail++))
elif grep -q 'velocity_scale_x: 1.0' "$player_root/src/brain/config/config.yaml" 2>/dev/null; then
  echo -e "  ${GREEN}[OK]${NC}   config.yaml velocity_scale=1.0 (no boost)"
  ((pass++))
else
  echo -e "  ${YELLOW}[??]${NC}  velocity_scale not found in config.yaml"
fi

echo ""
echo "=============================================="
echo -e " Result: ${GREEN}$pass OK${NC} / ${RED}$fail MISS${NC}"
echo "=============================================="

if [ "$fail" -gt 0 ]; then
  echo ""
  echo "Some patches are missing. Run:"
  echo "  ./scripts/install_into_inha_player.sh $inha_soccer_root"
  echo "  cd $inha_soccer_root && ./build.sh"
  exit 1
fi

echo ""
echo "All patches applied. Don't forget to rebuild if C++ source changed:"
echo "  cd $inha_soccer_root && ./build.sh"
