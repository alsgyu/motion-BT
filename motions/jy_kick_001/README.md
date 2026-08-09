# JY Kick 001 – Custom Mode Kick

이 폴더는 INHA brain에서 **custom mode 내에서 직접 킥**을 실행하기 위한
프로파일과 `booster_deploy` 킥 러너를 포함합니다.

모드 전환 없이 custom mode에서 바로 킥 정책이 실행됩니다.

## 로봇 측 모델

현재 실제 로봇 체크포인트:

```text
/home/booster/Workspace/deploy/tasks/kick_isaaclab/models/k1_kick_001_2026-08-05_01-05-52.pt
```

## 아키텍처

- **Walk Runner** (`run_bt_cmd_vel.py`): 평상시 BT cmd_vel 명령을 받아 보행 정책 실행
- **Kick Runner** (`run_bt_kick.py`): 킥 트리거를 받으면 킥 정책 실행, 완료 후 idle 복귀
- Walk Runner는 `/inha/custom_motion/kick/active` 토픽을 구독하여 킥 중 자동 정지

## 실행 방법

Terminal 1: walk runner 시작

```bash
cd /home/booster/Workspace/deploy
source /opt/booster/BoosterRos2Interface/install/setup.bash
python3 /home/booster/Workspace/INHA-Soccer/INHA-Player/motions/jy_kick_001/booster_deploy_overlay/run_bt_cmd_vel.py \
  --task k1_scratch_walk_001 \
  --checkpoint /home/booster/Workspace/deploy/tasks/scratch/models/JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt \
  --topic /inha/custom_motion/cmd_vel \
  --head-topic /inha/custom_motion/head
```

Terminal 2: kick runner 시작 (idle 상태로 대기)

```bash
cd /home/booster/Workspace/deploy
source /opt/booster/BoosterRos2Interface/install/setup.bash
python3 /home/booster/Workspace/INHA-Soccer/INHA-Player/motions/jy_kick_001/booster_deploy_overlay/run_bt_kick.py \
  --checkpoint /home/booster/Workspace/deploy/tasks/kick_isaaclab/models/k1_kick_001_2026-08-05_01-05-52.pt \
  --goal-topic /inha/custom_motion/kick/goal \
  --active-topic /inha/custom_motion/kick/active
```

Terminal 3: brain 시작

```bash
cd /home/booster/Workspace/INHA-Soccer/INHA-Player
./motions/jy_kick_001/start_bt_custom_motion.sh default
```

## 동작 흐름

1. BT가 `RLVisionKick`에 진입 → `kick/goal` 토픽에 볼 위치 + 목표 방향 publish
2. Kick Runner가 goal을 받으면 `/kick/active = true` publish → 킥 정책 실행
3. Walk Runner가 `kick/active` 감지 → velocity 명령을 0으로 멈춤
4. 킥 정책 완료 (약 2초) → `/kick/active = false`
5. Walk Runner 재개, BT는 `RLVisionKick` SUCCESS 반환
