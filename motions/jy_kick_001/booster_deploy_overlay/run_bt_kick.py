#!/usr/bin/env python3
"""
Custom-mode kick runner for the K1 kick policy.

Waits for kick goal messages from the BT (ball position + target direction),
then executes the TorchScript kick policy directly.  Publishes kick/active so
the walk runner can pause itself during the kick.

Usage:
  python3 run_bt_kick.py \\
    --checkpoint /path/to/k1_kick_001.pt \\
    --goal-topic /inha/custom_motion/kick/goal \\
    --active-topic /inha/custom_motion/kick/active
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass

import torch


# --- Isaac Lab helper (same as kick_k1.py) ----------------------------------

def _yaw_from_quat(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.unbind(-1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_apply_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """q is [w, x, y, z]; v is [x, y, z]. Returns v rotated by q^{-1}."""
    q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
    # v * q
    t = 2.0 * (q_x * v[0] + q_y * v[1] + q_z * v[2])
    u = 2.0 * q_w
    return torch.tensor([
        v[0] * (1.0 - 2.0 * (q_y * q_y + q_z * q_z)) + t * q_x + u * (q_y * v[2] - q_z * v[1]),
        v[1] * (1.0 - 2.0 * (q_x * q_x + q_z * q_z)) + t * q_y + u * (q_z * v[0] - q_x * v[2]),
        v[2] * (1.0 - 2.0 * (q_x * q_x + q_y * q_y)) + t * q_z + u * (q_x * v[1] - q_y * v[0]),
    ])


# --- Policy constants (mirrors kick_k1.py) ----------------------------------

POLICY_JOINT_NAMES = [
    "Left_Hip_Pitch", "Right_Hip_Pitch",
    "Left_Hip_Roll", "Right_Hip_Roll",
    "Left_Hip_Yaw", "Right_Hip_Yaw",
    "Left_Knee_Pitch", "Right_Knee_Pitch",
    "Left_Ankle_Pitch", "Right_Ankle_Pitch",
    "Left_Ankle_Roll", "Right_Ankle_Roll",
]

# Full 22-dof robot joint order (K1).
FULL_JOINT_NAMES = [
    "Left_Hip_Pitch", "Right_Hip_Pitch",
    "Left_Hip_Roll", "Right_Hip_Roll",
    "Left_Hip_Yaw", "Right_Hip_Yaw",
    "Left_Knee_Pitch", "Right_Knee_Pitch",
    "Left_Ankle_Pitch", "Right_Ankle_Pitch",
    "Left_Shoulder_Pitch", "Right_Shoulder_Pitch",
    "Left_Shoulder_Roll", "Right_Shoulder_Roll",
    "Left_Elbow", "Right_Elbow",
    "Left_Ankle_Roll", "Right_Ankle_Roll",
    "Left_Shoulder_Yaw", "Right_Shoulder_Yaw",
    "Left_Wrist", "Right_Wrist",
]

DEFAULT_JOINT_POS = torch.tensor([
    0.0, 0.0,
    0.0, -1.35, 1.57, 0.0,
    0.0, 1.35, 1.57, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
], dtype=torch.float32)

ACTION_SCALE = torch.tensor([
    0.2125, 0.2125,
    0.2375, 0.2375,
    0.1196875, 0.1196875,
    0.35, 0.35,
    0.3191666667, 0.3191666667,
    0.3191666667, 0.3191666667,
], dtype=torch.float32)

OBS_DIM = 49
ACTION_DIM = 12
POLICY_DT = 0.02  # 50 Hz
KICK_DURATION_SEC = 3.0  # maximum kick duration
STARTUP_HOLD_SEC = 1.0  # hold default pose before policy control (braking phase)
MAX_JOINT_VEL_FOR_KICK = 0.5  # rad/s — all leg joint vels must be below this to start kick


# --- Kick state machine -----------------------------------------------------

@dataclass
class KickGoal:
    ball_x: float  # ball x in robot frame (m)
    ball_y: float  # ball y in robot frame (m)
    target_angle_deg: float  # kick direction in robot frame (degrees)
    target_distance: float  # kick target distance (m)


class KickRunner:
    """Loads the kick policy and runs it on demand."""

    def __init__(self, checkpoint_path: str, goal_topic: str, active_topic: str):
        self.checkpoint_path = checkpoint_path
        self.goal_topic = goal_topic
        self.active_topic = active_topic
        self.device = torch.device("cpu")

        # ROS state
        self._node = None
        self._goal_sub = None
        self._active_pub = None
        self._ball_sub = None
        self._lock = threading.Lock()
        self._latest_goal: KickGoal | None = None
        self._ball_x: float = 0.275  # default: ball ~27.5cm in front
        self._ball_y: float = 0.0
        self._ball_valid: bool = False
        self._ball_time: float = 0.0  # last ball update monotonic time

        # Kick state
        self._kick_active = False
        self._kick_start_time = 0.0
        self._hold_until = 0.0
        self._last_action = torch.zeros(ACTION_DIM, dtype=torch.float32)
        self._target_w = torch.zeros(2, dtype=torch.float32)  # world-frame target

        # Robot state (updated from SDK)
        self._joint_pos = DEFAULT_JOINT_POS.clone()
        self._joint_vel = torch.zeros(22, dtype=torch.float32)
        self._root_pos = torch.zeros(3, dtype=torch.float32)
        self._root_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32)
        self._root_ang_vel = torch.zeros(3, dtype=torch.float32)

        # Model
        self._model: torch.jit.ScriptModule | None = None
        self._policy_joint_idx: torch.Tensor | None = None

        # SDK
        self._channel = None

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def load_model(self) -> None:
        checkpoint = os.path.expanduser(self.checkpoint_path)
        if not os.path.isfile(checkpoint):
            raise FileNotFoundError(f"Kick checkpoint not found: {checkpoint}")
        self._model = torch.jit.load(checkpoint, map_location=self.device)
        self._model.to(self.device).eval()
        print(f"[kick] loaded model: {checkpoint}", flush=True)

        # Build joint index mapping: policy joints (12) → full robot joints (22)
        full_idx_map = {name: i for i, name in enumerate(FULL_JOINT_NAMES)}
        idx_list = [full_idx_map[name] for name in POLICY_JOINT_NAMES]
        self._policy_joint_idx = torch.tensor(idx_list, dtype=torch.long)

        print(f"[kick] policy joint indices: {idx_list}", flush=True)

    def init_ros(self) -> None:
        import rclpy
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import Bool

        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self._node = rclpy.create_node("bt_kick_runner")

        self._goal_sub = self._node.create_subscription(
            Pose2D,
            self.goal_topic,
            self._goal_callback,
            10,
        )
        print(f"[kick] subscribed to goal: {self.goal_topic}", flush=True)

        self._active_pub = self._node.create_publisher(
            Bool,
            self.active_topic,
            10,
        )
        print(f"[kick] publishing active state to: {self.active_topic}", flush=True)

    def init_sdk(self) -> None:
        try:
            from booster_robotics_sdk_python import ChannelFactory
        except ImportError:
            print("[kick] booster_robotics_sdk_python not available; running in dry-run mode", flush=True)
            return

        self._channel = ChannelFactory.Instance()
        self._channel.Init(0, "")
        print("[kick] SDK channel initialised", flush=True)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, msg) -> None:
        """Receive kick goal from BT. Pose2D: x=ball_x, y=ball_y, theta=target_angle_deg."""
        goal = KickGoal(
            ball_x=float(msg.x),
            ball_y=float(msg.y),
            target_angle_deg=float(msg.theta),
            target_distance=4.0,  # default, can be extended
        )
        with self._lock:
            self._latest_goal = goal
        print(f"[kick] received goal: ball=({goal.ball_x:.3f}, {goal.ball_y:.3f}) "
              f"target_angle={goal.target_angle_deg:.1f}deg", flush=True)

    # ------------------------------------------------------------------
    # Robot state update (called from main loop)
    # ------------------------------------------------------------------

    def update_robot_state(self) -> None:
        """Read robot state from SDK channel."""
        if self._channel is None:
            return
        try:
            state = self._channel.GetRobotState()
            if state is None:
                return
            # Joint positions and velocities
            for i in range(min(22, len(state.joint_pos))):
                self._joint_pos[i] = float(state.joint_pos[i])
            for i in range(min(22, len(state.joint_vel))):
                self._joint_vel[i] = float(state.joint_vel[i])
            # Base pose
            self._root_pos[0] = float(state.root_pos[0])
            self._root_pos[1] = float(state.root_pos[1])
            self._root_pos[2] = float(state.root_pos[2])
            self._root_quat[0] = float(state.root_quat[0])
            self._root_quat[1] = float(state.root_quat[1])
            self._root_quat[2] = float(state.root_quat[2])
            self._root_quat[3] = float(state.root_quat[3])
            self._root_ang_vel[0] = float(state.root_ang_vel[0])
            self._root_ang_vel[1] = float(state.root_ang_vel[1])
            self._root_ang_vel[2] = float(state.root_ang_vel[2])
        except Exception:
            pass  # state not available yet

    # ------------------------------------------------------------------
    # Kick lifecycle
    # ------------------------------------------------------------------

    def start_kick(self, goal: KickGoal) -> None:
        """Begin a new kick with the given goal."""
        self._kick_active = True
        self._kick_start_time = time.monotonic()
        self._hold_until = self._kick_start_time + STARTUP_HOLD_SEC
        self._last_action.zero_()

        # Compute target in world frame
        yaw = _yaw_from_quat(self._root_quat)
        local_angle = float(yaw) + math.radians(goal.target_angle_deg)
        self._target_w[0] = self._root_pos[0] + goal.target_distance * math.cos(local_angle)
        self._target_w[1] = self._root_pos[1] + goal.target_distance * math.sin(local_angle)

        # Reset model hidden state if supported
        reset = getattr(self._model, "reset", None)
        if reset is not None:
            try:
                reset()
            except (RuntimeError, TypeError):
                try:
                    reset(torch.ones(1, dtype=torch.bool))
                except Exception:
                    pass

        # Publish active=true
        self._publish_active(True)
        print(f"[kick] STARTED target_w=({self._target_w[0]:.2f}, {self._target_w[1]:.2f})", flush=True)

    def finish_kick(self) -> None:
        """End the current kick."""
        self._kick_active = False
        self._publish_active(False)
        # Send zero-command to release control back to walk runner
        self._send_joint_command(self._joint_pos.clone())
        print("[kick] FINISHED", flush=True)

    def _publish_active(self, active: bool) -> None:
        if self._active_pub is None:
            return
        from std_msgs.msg import Bool
        self._active_pub.publish(Bool(data=active))

    # ------------------------------------------------------------------
    # Main inference loop (called each tick)
    # ------------------------------------------------------------------

    def step(self) -> bool:
        """
        Process one tick. Returns True if kick is active (caller should
        keep calling), False if idle.
        """
        with self._lock:
            goal = self._latest_goal
            self._latest_goal = None

        if goal is not None:
            self.start_kick(goal)

        if not self._kick_active:
            return False

        now = time.monotonic()
        elapsed = now - self._kick_start_time

        # Timeout check
        if elapsed > KICK_DURATION_SEC:
            self.finish_kick()
            return False

        # Startup hold (braking) phase — bring robot to a complete stop
        # before kick policy takes over.  Uses elevated leg stiffness to
        # actively brake any residual motion from the walk policy.
        #
        # Non-policy joints (head, arms) are preserved from current robot
        # state — the head stays at its last BT-commanded position.
        if now < self._hold_until:
            # Check if leg joints have settled
            leg_vels = self._joint_vel[self._policy_joint_idx]
            max_leg_vel = float(leg_vels.abs().max())

            if max_leg_vel > MAX_JOINT_VEL_FOR_KICK:
                # Robot is still moving — extend the hold to keep braking
                self._hold_until = max(self._hold_until, now + POLICY_DT)
                if int((now - self._kick_start_time) * 10) % 10 == 0:
                    print(f"[kick] braking... max_leg_vel={max_leg_vel:.2f} rad/s", flush=True)

            hold_targets = self._joint_pos.clone()
            hold_targets[self._policy_joint_idx] = DEFAULT_JOINT_POS[self._policy_joint_idx]
            # Use higher leg stiffness during braking for faster stop
            self._send_joint_command(
                hold_targets,
                stiffness=40.0,
                damping=2.0,
                leg_stiffness=80.0,
                leg_damping=3.0,
            )
            return True

        # Ready to kick
        leg_vels = self._joint_vel[self._policy_joint_idx]
        max_leg_vel = float(leg_vels.abs().max())
        print(f"[kick] braking complete — starting policy (max_leg_vel={max_leg_vel:.2f} rad/s)", flush=True)

        # Run inference
        obs = self._compute_observation()
        with torch.no_grad():
            output = self._model(obs)
            action = output[0] if isinstance(output, tuple) else output
            action = action.reshape(-1)

        if action.numel() != ACTION_DIM or not torch.isfinite(action).all():
            print("[kick] invalid action — aborting", flush=True)
            self.finish_kick()
            return False

        self._last_action.copy_(action)
        # Preserve non-policy joints (head, arms) from current robot state,
        # only override the 12 policy leg joints with kick policy output.
        targets = self._joint_pos.clone()
        targets[self._policy_joint_idx] = (
            DEFAULT_JOINT_POS[self._policy_joint_idx] + action * ACTION_SCALE
        )
        self._send_joint_command(targets)

        return True

    def _compute_observation(self) -> torch.Tensor:
        """Build 49-dim observation tensor matching training."""
        yaw = _yaw_from_quat(self._root_quat)

        # Gravity in base frame
        gravity_w = torch.tensor([0.0, 0.0, -1.0])
        gravity_b = _quat_apply_inverse(self._root_quat, gravity_w)

        # Ball position relative to robot in base frame
        c, s = torch.cos(yaw), torch.sin(yaw)
        # Use BT-provided ball position, falling back to default
        with self._lock:
            bx, by = self._ball_x, self._ball_y
        delta_w = torch.tensor([bx, by])  # already in robot frame from BT
        # The policy was trained with MuJoCo ball in world→robot conversion.
        # BT provides ball in robot frame directly, so just clamp.
        ball_rel = torch.stack((
            c * delta_w[0] + s * delta_w[1],
            -s * delta_w[0] + c * delta_w[1],
        ))
        # Actually BT sends ball in ROBOT frame already, so no conversion needed.
        ball_rel = torch.tensor([bx, by]).clamp(-3.0, 3.0)

        # Target position relative to robot
        delta_t = self._target_w - self._root_pos[:2]
        target_rel = torch.stack((
            c * delta_t[0] + s * delta_t[1],
            -s * delta_t[0] + c * delta_t[1],
        )).clamp(-2.0, 2.0) * 0.25

        # Command (always kick mode: [1, 0, 1])
        command = torch.tensor([1.0, 0.0, 1.0])

        # Joint state (policy joints only)
        jp = self._joint_pos[self._policy_joint_idx] - DEFAULT_JOINT_POS[self._policy_joint_idx]
        jv = self._joint_vel[self._policy_joint_idx] * 0.1

        obs = torch.cat((
            gravity_b,
            self._root_ang_vel,
            ball_rel,
            target_rel,
            command,
            jp,
            jv,
            self._last_action,
        ))

        if obs.numel() != OBS_DIM:
            raise RuntimeError(f"Expected {OBS_DIM} obs, got {obs.numel()}")

        return obs.unsqueeze(0)

    # ------------------------------------------------------------------
    # Robot command output
    # ------------------------------------------------------------------

    def _send_joint_command(
        self, targets: torch.Tensor,
        stiffness: float = 40.0,
        damping: float = 1.0,
        leg_stiffness: float | None = None,
        leg_damping: float | None = None,
    ) -> None:
        """Send joint position targets to the robot via SDK.

        Args:
            targets: 22-dim joint position target tensor.
            stiffness: default stiffness for all joints.
            damping: default damping for all joints.
            leg_stiffness: stiffness override for policy (leg) joints.
            leg_damping: damping override for policy (leg) joints.
        """
        if self._channel is None:
            return
        try:
            cmd = self._channel.CreateJointCmd()
            for i in range(min(22, len(targets))):
                cmd.joint_pos[i] = float(targets[i])
            stiff = [stiffness] * 22
            damp = [damping] * 22
            if leg_stiffness is not None or leg_damping is not None:
                for idx in self._policy_joint_idx.tolist():
                    if leg_stiffness is not None:
                        stiff[idx] = leg_stiffness
                    if leg_damping is not None:
                        damp[idx] = leg_damping
            cmd.joint_stiffness = stiff
            cmd.joint_damping = damp
            self._channel.SendJointCmd(cmd)
        except Exception as e:
            print(f"[kick] failed to send joint command: {e}", flush=True)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._kick_active = False
        self._publish_active(False)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        print("[kick] runner shut down", flush=True)


# ============================================================================
# Main
# ============================================================================

def main() -> int:
    parser = argparse.ArgumentParser(description="Custom-mode kick runner for K1")
    parser.add_argument("--checkpoint", required=True,
                        help="Path to TorchScript kick policy checkpoint (.pt)")
    parser.add_argument("--goal-topic", default="/inha/custom_motion/kick/goal",
                        help="ROS topic for kick goal (geometry_msgs/Pose2D)")
    parser.add_argument("--active-topic", default="/inha/custom_motion/kick/active",
                        help="ROS topic to signal kick active state (std_msgs/Bool)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run without SDK (test mode)")
    args = parser.parse_args()

    runner = KickRunner(
        checkpoint_path=args.checkpoint,
        goal_topic=args.goal_topic,
        active_topic=args.active_topic,
    )

    try:
        runner.load_model()
        runner.init_ros()
        if not args.dry_run:
            runner.init_sdk()
    except Exception as e:
        print(f"[kick] initialisation failed: {e}", flush=True)
        return 1

    print("[kick] waiting for kick goal...", flush=True)

    import rclpy
    rate = runner._node.create_rate(1.0 / POLICY_DT)  # 50 Hz

    try:
        while rclpy.ok():
            rclpy.spin_once(runner._node, timeout_sec=0.0)
            runner.update_robot_state()
            runner.step()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        runner.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
