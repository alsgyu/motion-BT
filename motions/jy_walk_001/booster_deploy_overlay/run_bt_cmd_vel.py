#!/usr/bin/env python3
from __future__ import annotations

import argparse
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


class CmdVelControlService:
    """RemoteControlService-compatible command source backed by ROS cmd_vel."""

    def __init__(self, config: CmdVelControlConfig):
        self.config = config
        self._lock = threading.Lock()
        self._node = None
        self._sub = None
        self._last_cmd_time = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0

    def get_operation_hint(self) -> str:
        return f"Listening for BT velocity commands on {self.config.topic}"

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
            return self._normalize(self._fresh_or_zero(self._vx), self.config.max_vx)

    def get_vy_cmd(self) -> float:
        self._spin_once()
        with self._lock:
            return self._normalize(self._fresh_or_zero(self._vy), self.config.max_vy)

    def get_vyaw_cmd(self) -> float:
        self._spin_once()
        with self._lock:
            return self._normalize(self._fresh_or_zero(self._vyaw), self.config.max_vyaw)

    def close(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        self._sub = None

    def _ensure_node(self):
        if self._node is not None:
            return

        import rclpy
        from geometry_msgs.msg import Twist

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
    parser.add_argument("--cmd-timeout", type=float, default=0.5)
    parser.add_argument("--require-first-cmd", action="store_true", default=False)
    args = parser.parse_args()

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

    with controller_mod.BoosterRobotPortal(task_cfg, use_sim_time=args.webots) as portal:
        portal.run()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
