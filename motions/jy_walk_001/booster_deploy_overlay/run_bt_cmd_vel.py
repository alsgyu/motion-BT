#!/usr/bin/env python3
from __future__ import annotations

import argparse
import multiprocessing as mp
import pkgutil
import sys
import threading
import time
from dataclasses import dataclass


DEFAULT_CHECKPOINT = (
    "/home/booster/Workspace/deploy/tasks/scratch/models/"
    "JY_walk_001_symmetry_2026-07-30_18-18-57_best_best.pt"
)


@dataclass
class CmdVelControlConfig:
    topic: str
    max_vx: float
    max_vy: float
    max_vyaw: float
    timeout_sec: float
    require_first_cmd: bool
    head_topic: str
    head_yaw_index: int
    head_pitch_index: int
    head_timeout_sec: float
    head_override: bool
    # -- Smoothing / rate-limiting --
    smoothing_alpha: float = 0.3       # 0=no smoothing, 1=instant (no filter)
    max_delta_vx: float = 0.15         # max vx change per control step (normalized)
    max_delta_vy: float = 0.15         # max vy change per control step (normalized)
    max_delta_vyaw: float = 0.2        # max vyaw change per control step (normalized)
    deadband: float = 0.08             # commands below this (normalized) are zeroed
    stop_decay_alpha: float = 0.5      # decay rate when command times out (lower = gentler stop)
    # -- Output clamping (hard limit on final normalized output) --
    max_output_vx: float = 1.0         # clamp |vx| <= this (1.0 = no extra limit)
    max_output_vy: float = 1.0         # clamp |vy| <= this
    max_output_vyaw: float = 1.0       # clamp |vyaw| <= this
    # -- Ramp-up: gradually increase max_output from safe to target --
    ramp_up_time_sec: float = 2.0      # time to ramp from safe output to max_output
    ramp_start_output: float = 0.5     # initial output clamp when cmd first arrives


class CmdVelControlService:
    """RemoteControlService-compatible command source backed by ROS cmd_vel."""

    def __init__(self, config: CmdVelControlConfig):
        self.config = config
        self._lock = threading.Lock()
        self._node = None
        self._sub = None
        self._head_sub = None
        self._kick_sub = None
        self._kick_active = False
        self._last_cmd_time = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._smoothed_vx = 0.0
        self._smoothed_vy = 0.0
        self._smoothed_vyaw = 0.0
        self._head_lock = mp.Lock()
        self._head_pitch = mp.Value("d", 0.0)
        self._head_yaw = mp.Value("d", 0.0)
        self._head_last_cmd_time = mp.Value("d", 0.0)
        # Ramp-up state: track when non-zero cmd first arrives to gradually raise max_output
        self._ramp_start_time = 0.0  # monotonic time when ramp began
        self._ramp_was_active = False  # whether ramp was in progress last step

    def get_operation_hint(self) -> str:
        hint = f"Listening for BT velocity commands on {self.config.topic}"
        if self.config.head_override:
            hint += f" and head commands on {self.config.head_topic}"
        return hint

    def get_custom_mode_operation_hint(self) -> str:
        return "Auto-starting custom mode for BT control."

    def get_rl_gait_operation_hint(self) -> str:
        return "Auto-starting RL gait for BT control."

    def start_custom_mode(self) -> bool:
        if not self.config.require_first_cmd:
            return True
        return self._has_fresh_command()

    def start_rl_gait(self) -> bool:
        if not self.config.require_first_cmd:
            return True
        return self._has_fresh_command()

    def get_vx_cmd(self) -> float:
        self._spin_once()
        with self._lock:
            if self._kick_active:
                self._smoothed_vx = 0.0
                return 0.0
            return self._smooth_vx_locked()

    def get_vy_cmd(self) -> float:
        self._spin_once()
        with self._lock:
            if self._kick_active:
                self._smoothed_vy = 0.0
                return 0.0
            return self._smooth_vy_locked()

    def get_vyaw_cmd(self) -> float:
        self._spin_once()
        with self._lock:
            if self._kick_active:
                self._smoothed_vyaw = 0.0
                return 0.0
            return self._smooth_vyaw_locked()

    # ------------------------------------------------------------------
    # Smoothing helpers (must be called while holding self._lock)
    # ------------------------------------------------------------------
    def _smooth_vx_locked(self) -> float:
        return self._apply_smoothing(
            raw=lambda: self._vx,
            smoothed_attr="_smoothed_vx",
            max_delta=self.config.max_delta_vx,
            max_abs=self.config.max_vx,
            max_output=self.config.max_output_vx,
        )

    def _smooth_vy_locked(self) -> float:
        return self._apply_smoothing(
            raw=lambda: self._vy,
            smoothed_attr="_smoothed_vy",
            max_delta=self.config.max_delta_vy,
            max_abs=self.config.max_vy,
            max_output=self.config.max_output_vy,
        )

    def _smooth_vyaw_locked(self) -> float:
        return self._apply_smoothing(
            raw=lambda: self._vyaw,
            smoothed_attr="_smoothed_vyaw",
            max_delta=self.config.max_delta_vyaw,
            max_abs=self.config.max_vyaw,
            max_output=self.config.max_output_vyaw,
        )

    def _apply_smoothing(
        self,
        raw,
        smoothed_attr: str,
        max_delta: float,
        max_abs: float,
        max_output: float = 1.0,
    ) -> float:
        """Apply EMA filter, deadband, rate limiter, output clamp, and normalization.

        Pipeline (all in normalized [-1, 1] space):
          1. Normalize raw command
          2. Deadband: zero out tiny commands
          3. EMA low-pass filter (smoothing_alpha)
          4. Rate limiter (max_delta per call)
          5. Ramp-aware output clamp: gradually increases from ramp_start_output
             to max_output over ramp_up_time_sec when a fresh command arrives.
          6. Return final normalized value
        """
        alpha = self.config.smoothing_alpha
        deadband = self.config.deadband

        # 1. Normalize raw to [-1, 1]
        raw_val = raw()
        raw_norm = self._normalize(raw_val, max_abs)

        # 2. Deadband
        if abs(raw_norm) < deadband:
            raw_norm = 0.0

        # 3. EMA: smoothed = alpha * raw + (1 - alpha) * prev_smoothed
        prev = getattr(self, smoothed_attr)
        target = raw_norm

        fresh = self._is_fresh_locked()

        # If command is stale, decay toward zero instead of holding
        if not fresh:
            target = 0.0
            # Use stop_decay_alpha for gentle deceleration
            decay_alpha = self.config.stop_decay_alpha
            new_val = decay_alpha * target + (1.0 - decay_alpha) * prev
        else:
            new_val = alpha * target + (1.0 - alpha) * prev

        # 4. Rate limiter: clamp delta
        delta = new_val - prev
        if delta > max_delta:
            new_val = prev + max_delta
        elif delta < -max_delta:
            new_val = prev - max_delta

        # 5. Ramp-aware output clamp
        effective_max = self._compute_ramp_max_output(max_output, fresh)
        if new_val > effective_max:
            new_val = effective_max
        elif new_val < -effective_max:
            new_val = -effective_max

        # Persist
        setattr(self, smoothed_attr, new_val)
        return new_val

    def _compute_ramp_max_output(self, target_max: float, fresh: bool) -> float:
        """Compute effective max_output with ramp-up from safe start to target.

        When a fresh command first arrives (after being idle/decaying), the output
        clamp starts at ramp_start_output and linearly increases to target_max over
        ramp_up_time_sec.  When the command is stale, the ramp resets so the next
        fresh command will ramp up again from the safe floor.
        """
        ramp_time = self.config.ramp_up_time_sec
        ramp_start = self.config.ramp_start_output

        # If ramp is disabled or start >= target, just use target
        if ramp_time <= 0.0 or ramp_start >= target_max:
            return target_max

        now = time.monotonic()

        if fresh:
            # Detect rising edge: was idle/decaying, now fresh
            if not self._ramp_was_active:
                self._ramp_start_time = now
            self._ramp_was_active = True

            elapsed = now - self._ramp_start_time
            if elapsed >= ramp_time:
                return target_max
            progress = elapsed / ramp_time
            return ramp_start + (target_max - ramp_start) * progress
        else:
            # Command is stale → reset ramp for next time
            self._ramp_was_active = False
            # Still clamp decaying output to target_max (don't artificially lower it)
            return target_max

    def get_head_override(self):
        if not self.config.head_override:
            return None

        with self._head_lock:
            last_cmd_time = self._head_last_cmd_time.value
            if last_cmd_time <= 0.0:
                return None
            if (
                self.config.head_timeout_sec > 0.0
                and time.monotonic() - last_cmd_time > self.config.head_timeout_sec
            ):
                return None
            return self._head_pitch.value, self._head_yaw.value

    def close(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        self._sub = None
        self._head_sub = None
        self._kick_sub = None

    def _ensure_node(self):
        if self._node is not None:
            return

        import rclpy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        if not rclpy.ok():
            return

        self._node = rclpy.create_node("bt_cmd_vel_control")
        self._sub = self._node.create_subscription(
            Twist,
            self.config.topic,
            self._cmd_vel_callback,
            10,
        )
        print(f"[bt_cmd_vel] subscribed: {self.config.topic}")
        if self.config.head_override:
            self._head_sub = self._node.create_subscription(
                JointState,
                self.config.head_topic,
                self._head_callback,
                10,
            )
            print(f"[bt_head] subscribed: {self.config.head_topic}")
        self._kick_sub = self._node.create_subscription(
            Bool,
            "/inha/custom_motion/kick/active",
            self._kick_active_callback,
            10,
        )
        print("[bt_cmd_vel] subscribed: /inha/custom_motion/kick/active")

    def _spin_once(self):
        import rclpy

        self._ensure_node()
        if self._node is not None and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.0)

    def _cmd_vel_callback(self, msg):
        with self._lock:
            self._vx = float(msg.linear.x)
            self._vy = float(msg.linear.y)
            self._vyaw = float(msg.angular.z)
            self._last_cmd_time = time.monotonic()

    def _head_callback(self, msg):
        yaw = None
        pitch = None

        if len(msg.name) == len(msg.position):
            for name, position in zip(msg.name, msg.position):
                lower_name = name.lower()
                if "head" not in lower_name:
                    continue
                if "yaw" in lower_name:
                    yaw = float(position)
                elif "pitch" in lower_name:
                    pitch = float(position)

        if yaw is None and len(msg.position) >= 1:
            yaw = float(msg.position[0])
        if pitch is None and len(msg.position) >= 2:
            pitch = float(msg.position[1])
        if yaw is None or pitch is None:
            return

        with self._head_lock:
            self._head_yaw.value = yaw
            self._head_pitch.value = pitch
            self._head_last_cmd_time.value = time.monotonic()

    def _kick_active_callback(self, msg):
        was_active = self._kick_active
        self._kick_active = bool(msg.data)
        if was_active != self._kick_active:
            print(f"[bt_cmd_vel] kick_active={self._kick_active}", flush=True)

    def _has_fresh_command(self) -> bool:
        self._spin_once()
        with self._lock:
            return self._last_cmd_time > 0.0 and self._is_fresh_locked()

    def _fresh_or_zero(self, value: float) -> float:
        if not self._is_fresh_locked():
            return 0.0
        return value

    def _is_fresh_locked(self) -> bool:
        if self._last_cmd_time <= 0.0:
            return False
        if self.config.timeout_sec <= 0.0:
            return True
        return time.monotonic() - self._last_cmd_time <= self.config.timeout_sec

    @staticmethod
    def _normalize(value: float, scale: float) -> float:
        if scale <= 1e-9:
            return 0.0
        value = max(-scale, min(scale, value))
        return value / scale


def import_all_tasks():
    import tasks as tasks_pkg

    for mod_info in pkgutil.walk_packages(tasks_pkg.__path__, prefix="tasks."):
        __import__(mod_info.name)


def patch_head_override(controller_mod):
    original_ctrl_step = controller_mod.BoosterRobotController.ctrl_step

    # Disable internal head tracking so BT's head command (via
    # patch_head_override) takes full control.  Otherwise
    # _apply_internal_head_targets() overwrites dof_targets head
    # joints with Loco API or ball-tracking values every ctrl_step.
    controller_mod.BoosterRobotController._apply_internal_head_targets = (
        lambda self, dof_targets: None
    )

    def ctrl_step_with_head_override(self, dof_targets):
        head_cmd = self.portal.remoteControlService.get_head_override()
        if head_cmd is None:
            return original_ctrl_step(self, dof_targets)

        pitch, yaw = head_cmd
        yaw_index = self.portal.remoteControlService.config.head_yaw_index
        pitch_index = self.portal.remoteControlService.config.head_pitch_index
        if yaw_index >= self.robot.num_joints or pitch_index >= self.robot.num_joints:
            return original_ctrl_step(self, dof_targets)

        dof_targets = dof_targets.clone()
        if yaw_index >= 0:
            dof_targets[yaw_index] = yaw
        if pitch_index >= 0:
            dof_targets[pitch_index] = pitch
        return original_ctrl_step(self, dof_targets)

    controller_mod.BoosterRobotController.ctrl_step = ctrl_step_with_head_override


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run booster_deploy with velocity commands from INHA BT."
    )
    parser.add_argument("--task", default="k1_scratch_walk_001")
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--topic", default="/inha/custom_motion/cmd_vel")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--net", default="127.0.0.1")
    parser.add_argument("--webots", action="store_true", default=False)
    parser.add_argument("--cmd-timeout", type=float, default=1.0)
    parser.add_argument("--require-first-cmd", action="store_true", default=False)
    parser.add_argument("--head-topic", default="/inha/custom_motion/head")
    parser.add_argument("--head-yaw-index", type=int, default=0)
    parser.add_argument("--head-pitch-index", type=int, default=1)
    parser.add_argument("--head-timeout", type=float, default=0.0)
    parser.add_argument("--disable-head-override", action="store_true", default=False)
    # -- Smoothing / rate-limiting --
    parser.add_argument("--config", type=str, default=None,
                        help="Path to YAML config file for smoothing parameters")
    parser.add_argument("--smoothing-alpha", type=float, default=None,
                        help="EMA smoothing factor (0=no smoothing, 1=instant)")
    parser.add_argument("--max-delta-vx", type=float, default=None,
                        help="Max vx change per control step (normalized)")
    parser.add_argument("--max-delta-vy", type=float, default=None,
                        help="Max vy change per control step (normalized)")
    parser.add_argument("--max-delta-vyaw", type=float, default=None,
                        help="Max vyaw change per control step (normalized)")
    parser.add_argument("--deadband", type=float, default=None,
                        help="Commands below this (normalized) are zeroed")
    parser.add_argument("--stop-decay-alpha", type=float, default=None,
                        help="Decay rate when cmd times out (lower=gentler stop)")
    # -- Output clamping --
    parser.add_argument("--max-output-vx", type=float, default=None,
                        help="Hard clamp |vx output| <= this (1.0=no limit)")
    parser.add_argument("--max-output-vy", type=float, default=None,
                        help="Hard clamp |vy output| <= this (1.0=no limit)")
    parser.add_argument("--max-output-vyaw", type=float, default=None,
                        help="Hard clamp |vyaw output| <= this (1.0=no limit)")
    # -- Ramp-up --
    parser.add_argument("--ramp-up-time-sec", type=float, default=None,
                        help="Time to ramp output clamp from safe to max (seconds)")
    parser.add_argument("--ramp-start-output", type=float, default=None,
                        help="Initial output clamp when command first arrives")
    args = parser.parse_args()

    # -- Load YAML config (lower priority than explicit CLI args) --
    yaml_smoothing: dict = {}
    config_path = args.config
    if config_path is None:
        # Try default location next to this script
        import os as _os
        _default_config = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "smoothing_config.yaml")
        if _os.path.isfile(_default_config):
            config_path = _default_config
    if config_path is not None:
        try:
            import yaml
            with open(config_path, "r") as f:
                yaml_data = yaml.safe_load(f)
            if isinstance(yaml_data, dict):
                yaml_smoothing = yaml_data.get("smoothing", {}) or {}
            print(f"[bt_cmd_vel] loaded smoothing config: {config_path}")
        except ImportError:
            print("[bt_cmd_vel] PyYAML not installed; skipping config file loading")
        except Exception as exc:
            print(f"[bt_cmd_vel] failed to load config {config_path}: {exc}")

    def _resolve(name: str, default: float) -> float:
        """CLI arg > YAML config > hardcoded default."""
        cli_val = getattr(args, name.replace("-", "_"), None)
        if cli_val is not None:
            return cli_val
        yaml_key = name  # e.g. "smoothing-alpha" → yaml key "smoothing-alpha" or "alpha"
        # Also try short form keys
        short_map = {
            "smoothing-alpha": "alpha",
            "max-delta-vx": "max_delta_vx",
            "max-delta-vy": "max_delta_vy",
            "max-delta-vyaw": "max_delta_vyaw",
            "deadband": "deadband",
            "stop-decay-alpha": "stop_decay_alpha",
            "max-output-vx": "max_output_vx",
            "max-output-vy": "max_output_vy",
            "max-output-vyaw": "max_output_vyaw",
            "ramp-up-time-sec": "ramp_up_time_sec",
            "ramp-start-output": "ramp_start_output",
        }
        yaml_key_short = short_map.get(name, name)
        return float(yaml_smoothing.get(yaml_key, yaml_smoothing.get(yaml_key_short, default)))

    smoothing_alpha = _resolve("smoothing-alpha", 0.3)
    max_delta_vx = _resolve("max-delta-vx", 0.15)
    max_delta_vy = _resolve("max-delta-vy", 0.15)
    max_delta_vyaw = _resolve("max-delta-vyaw", 0.2)
    deadband = _resolve("deadband", 0.08)
    stop_decay_alpha = _resolve("stop-decay-alpha", 0.5)
    max_output_vx = _resolve("max-output-vx", 1.0)
    max_output_vy = _resolve("max-output-vy", 1.0)
    max_output_vyaw = _resolve("max-output-vyaw", 1.0)
    ramp_up_time_sec = _resolve("ramp-up-time-sec", 2.0)
    ramp_start_output = _resolve("ramp-start-output", 0.5)

    print(
        f"[bt_cmd_vel] smoothing: alpha={smoothing_alpha:.3f} "
        f"delta_vx={max_delta_vx:.3f} delta_vy={max_delta_vy:.3f} delta_vyaw={max_delta_vyaw:.3f} "
        f"deadband={deadband:.4f} stop_decay={stop_decay_alpha:.3f} "
        f"max_out(vx={max_output_vx:.2f} vy={max_output_vy:.2f} vyaw={max_output_vyaw:.2f}) "
        f"ramp(up={ramp_up_time_sec:.1f}s start={ramp_start_output:.2f})",
        flush=True,
    )

    sys.path.append(".")
    import_all_tasks()

    from booster_deploy.utils.registry import get_task, list_tasks

    try:
        task_cfg = get_task(args.task)
    except KeyError:
        print(f"Unknown task '{args.task}'. Available tasks: {list(list_tasks().keys())}")
        return 1

    task_cfg.policy.device = args.device
    if args.checkpoint and hasattr(task_cfg.policy, "checkpoint_path"):
        task_cfg.policy.checkpoint_path = args.checkpoint

    if task_cfg.vel_command is None:
        print(f"Task '{args.task}' has no vel_command config.")
        return 1

    cmd_cfg = CmdVelControlConfig(
        topic=args.topic,
        max_vx=float(task_cfg.vel_command.vx_max),
        max_vy=float(task_cfg.vel_command.vy_max),
        max_vyaw=float(task_cfg.vel_command.vyaw_max),
        timeout_sec=args.cmd_timeout,
        require_first_cmd=args.require_first_cmd,
        head_topic=args.head_topic,
        head_yaw_index=args.head_yaw_index,
        head_pitch_index=args.head_pitch_index,
        head_timeout_sec=args.head_timeout,
        head_override=not args.disable_head_override,
        smoothing_alpha=smoothing_alpha,
        max_delta_vx=max_delta_vx,
        max_delta_vy=max_delta_vy,
        max_delta_vyaw=max_delta_vyaw,
        deadband=deadband,
        stop_decay_alpha=stop_decay_alpha,
        max_output_vx=max_output_vx,
        max_output_vy=max_output_vy,
        max_output_vyaw=max_output_vyaw,
        ramp_up_time_sec=ramp_up_time_sec,
        ramp_start_output=ramp_start_output,
    )

    class BoundCmdVelControlService(CmdVelControlService):
        def __init__(self):
            super().__init__(cmd_cfg)

    try:
        from booster_robotics_sdk_python import ChannelFactory  # type: ignore
    except ImportError:
        print("booster_robotics_sdk_python is not installed.")
        return 1

    ChannelFactory.Instance().Init(0, args.net)

    from booster_deploy.controllers import booster_robot_controller as controller_mod

    controller_mod.RemoteControlService = BoundCmdVelControlService
    patch_head_override(controller_mod)

    with controller_mod.BoosterRobotPortal(task_cfg, use_sim_time=args.webots) as portal:
        portal.run()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
