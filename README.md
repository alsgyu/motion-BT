# motion-BT

INHA behavior tree velocity commands can drive a custom `booster_deploy` RL
walking policy through a small `cmd_vel` bridge.

This repository does not replace the whole `INHA-Soccer` tree. It ships:

- a patch for `INHA-Soccer/INHA-Player/src/brain`
- the `motions/jy_walk_001` runtime folder
- an install script that applies both pieces on the robot

## Robot Model

Expected checkpoint on the real robot:

```text
/home/booster/Workspace/deploy/tasks/scratch/models/JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt
```

Expected `booster_deploy` task config:

```text
/home/booster/Workspace/deploy/tasks/scratch/walk_001.py
```

## Install On Robot

Clone this repository on the robot:

```bash
cd /home/booster/Workspace
git clone https://github.com/alsgyu/motion-BT.git
```

If it is already cloned, update it instead:

```bash
cd /home/booster/Workspace/motion-BT
git pull
```

Apply the INHA brain patch and copy the motion runtime folder:

```bash
cd /home/booster/Workspace/motion-BT
./scripts/install_into_inha_player.sh /home/booster/Workspace/INHA-Soccer
```

Rebuild INHA-Player:

```bash
cd /home/booster/Workspace/INHA-Soccer/INHA-Player
colcon build --symlink-install
```

## Run

Terminal 1: start the custom policy runner from the `booster_deploy` root.

```bash
cd /home/booster/Workspace/deploy
source /opt/booster/BoosterRos2Interface/install/setup.bash
python3 /home/booster/Workspace/INHA-Soccer/INHA-Player/motions/jy_walk_001/booster_deploy_overlay/run_bt_cmd_vel.py \
  --task k1_scratch_walk_001 \
  --checkpoint /home/booster/Workspace/deploy/tasks/scratch/models/JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt \
  --topic /inha/custom_motion/cmd_vel \
  --head-topic /inha/custom_motion/head
```

Terminal 2: start INHA brain in custom motion mode.

```bash
cd /home/booster/Workspace/INHA-Soccer/INHA-Player
./motions/jy_walk_001/start_bt_custom_motion.sh default
```

For kick-and-return experiments, start the deploy runner first, confirm it is
publishing `/joint_ctrl`, then add:

```bash
custom_motion_brain_controls_mode:=true
```

Example:

```bash
./motions/jy_walk_001/start_bt_custom_motion.sh default custom_motion_brain_controls_mode:=true
```

## Behavior

- Default mode is unchanged unless `custom_motion:=true` is passed.
- With custom motion enabled, `RobotClient::setVelocity()` publishes
  `geometry_msgs/Twist` to `/inha/custom_motion/cmd_vel`.
- With custom motion enabled, `RobotClient::moveHead()` also publishes
  `sensor_msgs/JointState` to `/inha/custom_motion/head`.
- Every robot mode change sends zero velocity to both the custom motion
  `cmd_vel` bridge and the Booster SDK move API before and after the mode
  request.
- The `booster_deploy` overlay converts that `Twist` into normalized policy
  velocity commands.
- The overlay keeps the RL gait output for the body and overrides only the head
  yaw/pitch joint targets from the latest head command.
- `RLVisionKick` switches back to soccer mode before starting VisualKick.
- In custom motion mode, `RLVisionKick` waits until the robot state confirms
  soccer mode before sending `VisualKick(true)`, then the root mode gate returns
  to custom mode after the visual-kick lock clears.
- After soccer mode is confirmed, `RLVisionKick` runs a head-only ball
  reacquire scan before sending `VisualKick(true)`. It uses a CamFindBall-like
  low/high pitch and left/center/right yaw sweep while keeping the visual-kick
  decision active.
- Before that sweep, it first points the head at the last known filtered ball
  direction when `ball_location_known` is still available.
- `VisualKick(true)` is sent only after the head reclaim window has elapsed and
  a fresh raw ball detection is available again.
- If the ball is not reacquired within `soccer_head_reacquire_msec`,
  `RLVisionKick` exits without a blind kick so the BT can fall back to
  custom-motion find/chase.
- After the kick command, `RLVisionKick` keeps the head down briefly so the
  soccer-mode entry reset does not immediately lose the ball again.
- Before `VisualKick(true)` is sent, the BT holds the visual-kick decision
  briefly so a momentary ball loss during the soccer-mode handoff does not fall
  back to chase/find.
