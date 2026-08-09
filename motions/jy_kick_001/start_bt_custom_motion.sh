#!/usr/bin/env bash
set -euo pipefail

field_name="${1:-default}"
if [ "$#" -gt 0 ]; then
  shift
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
player_root="$(cd "$script_dir/../.." && pwd)"

cd "$player_root"
exec ./scripts/start.sh "$field_name" \
  custom_motion:=true \
  custom_motion_topic:=/inha/custom_motion/cmd_vel \
  custom_motion_head_topic:=/inha/custom_motion/head \
  custom_motion_require_subscriber:=true \
  custom_kick:=true \
  custom_kick_goal_topic:=/inha/custom_motion/kick/goal \
  custom_kick_active_topic:=/inha/custom_motion/kick/active \
  "$@"
