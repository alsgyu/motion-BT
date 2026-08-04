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
  --topic /inha/custom_motion/cmd_vel
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
