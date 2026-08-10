# JY Walk 001 BT Bridge

This folder keeps the INHA brain-side custom motion profile and the small
`booster_deploy` overlay needed to drive the policy from BT velocity commands.

## Robot-side model

The current real-robot checkpoint is expected at:

```text
/home/booster/Workspace/deploy/tasks/scratch/models/JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt
```

The task settings are expected to live in:

```text
/home/booster/Workspace/deploy/tasks/scratch/walk_001.py
```

## Intended flow

1. Start the deploy runner first so it owns `/joint_ctrl` and can enter
   `RobotMode.kCustom`.
2. Start INHA brain with `custom_motion:=true`.
3. BT `SetVelocity` and local planner commands publish `geometry_msgs/Twist` to
   `/inha/custom_motion/cmd_vel` instead of sending SDK `CreateMoveMsg`.
4. `RLVisionKick` switches back to `kSoccer` before sending VisualKick.

## Commands

From the robot's `booster_deploy` root:

```bash
cd /home/booster/Workspace/deploy
source /opt/booster/BoosterRos2Interface/install/setup.bash
python3 /home/booster/Workspace/INHA-Soccer/INHA-Player/motions/jy_walk_001/booster_deploy_overlay/run_bt_cmd_vel.py \
  --task k1_scratch_walk_001 \
  --checkpoint /home/booster/Workspace/deploy/tasks/scratch/models/JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt \
  --topic /inha/custom_motion/cmd_vel \
  --head-topic /inha/custom_motion/head
```

Then start brain:

```bash
cd /home/booster/Workspace/INHA-Soccer/INHA-Player
./motions/jy_walk_001/start_bt_custom_motion.sh default
```

For kick-and-return experiments, start the deploy runner first, confirm it is
publishing `/joint_ctrl`, then add this launch argument to let brain request
`kCustom` again after a soccer kick:

```bash
custom_motion_brain_controls_mode:=true
```

## Configuration (no rebuild needed)

설정은 **두 군데**로 나뉘어 있습니다:

| 위치 | 담당 | 수정 방법 |
|------|------|----------|
| `start_bt_custom_motion.sh`의 `:=` 인자들 | Brain (C++) | 스크립트 수정 or CLI 인자 추가 |
| `smoothing_config.yaml` | Deploy runner (Python) | YAML 수정 후 runner 재시작 |

### Brain-side tunables (in `start_bt_custom_motion.sh`)

```bash
# velocity_scale: BT가 계산한 속도에 곱하는 배율. 1.0=원본 그대로.
custom_motion_velocity_scale_x:=1.0
custom_motion_velocity_scale_y:=1.0
custom_motion_velocity_scale_theta:=1.0

# cmd_vel_update_interval_msec: 동일한 non-zero 명령을 재발행하기까지의 최소 간격(ms).
# BT가 매 틱마다 SetVelocity를 호출해도, 이 간격 동안은 같은 명령이 유지됨.
# 0으로 설정하면 매 틱마다 발행 (throttle 없음).
# 기본 500ms → 빠른 반응이 필요하면 150~250ms, 떨림이 있으면 500~800ms.
custom_motion_cmd_vel_update_interval_msec:=250.0

# soccer_mode_stop_settle_msec: custom → soccer 모드 전환 시 zero velocity 유지 시간(ms).
custom_motion_soccer_mode_stop_settle_msec:=350.0
```

### Deploy-runner-side tunables (`smoothing_config.yaml`)

Runner가 자동으로 읽습니다. 수정 후 runner만 재시작.

```bash
# YAML 무시하고 CLI로만 테스트:
python3 run_bt_cmd_vel.py --smoothing-alpha 0.15 --deadband 0.12
```

세부 파라미터 설명은 `smoothing_config.yaml` 주석 참고.

### Config 우선순위

```
CLI args (--smoothing-alpha 0.2)  >  smoothing_config.yaml  >  코드 기본값
   (brain)                               (deploy runner)
```

## Deadband explained

Deadband는 **normalized [-1, 1]** 공간에서 동작합니다:

| max_vx | deadband | 무시되는 raw cmd |
|--------|----------|-----------------|
| 0.5 m/s | 0.02 | < 0.01 m/s |
| 0.5 m/s | 0.08 | < 0.04 m/s |
| 0.5 m/s | 0.15 | < 0.075 m/s |
| 1.0 m/s | 0.08 | < 0.08 m/s |

Deadband는 EMA 필터 **이전**에 적용되므로, 작은 떨림이 smoothing state에
누적되는 것을 원천 차단합니다.

- 떨림/oscillation을 잡으려면 **0.08~0.12**가 적당합니다.
- 0.02는 단순 전기적 노이즈 제거 수준이라 BT 떨림을 못 잡습니다.
- 너무 높이면(0.2+) 느린 Chase가 멈춰 보일 수 있습니다.
