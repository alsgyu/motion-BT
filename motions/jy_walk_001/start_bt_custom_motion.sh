#!/usr/bin/env bash
set -euo pipefail

# ==========================================================================
# start_bt_custom_motion.sh – Launch INHA brain in custom motion mode.
#
# Brain-side tunables (ROS param overrides, no rebuild needed):
#   custom_motion_velocity_scale_x:=1.0       속도 증폭 배율 (1.0=원본 그대로)
#   custom_motion_cmd_vel_update_interval_msec:=500.0  같은 명령 유지 최소 시간(ms)
#   custom_motion_soccer_mode_stop_settle_msec:=350.0   모드 전환 전 정지 유지 시간(ms)
#
# Deploy-runner-side tunables (edit smoothing_config.yaml, restart runner):
#   smoothing/alpha, max_delta_v*, deadband, stop_decay_alpha
# ==========================================================================

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
  custom_motion_velocity_scale_x:=1.0 \
  custom_motion_velocity_scale_y:=1.0 \
  custom_motion_velocity_scale_theta:=1.0 \
  custom_motion_cmd_vel_update_interval_msec:=250.0 \
  custom_motion_soccer_mode_stop_settle_msec:=350.0 \
  "$@"
