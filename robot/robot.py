
import argparse
import pathlib
import sys
import time
import warnings

# allow running this file directly: put the project root on the import path
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
# use the CRI git submodule (robot/common_robot_interface) ahead of any other installed copy
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent / "common_robot_interface"))

import numpy as np
from cri.controller import Controller
from cri.robot import SyncRobot

import config


class CriArm:
    def __init__(self, cfg: config.RobotConfig):
        self._dof = cfg.dof

        # CRI controller (from CRI's registry, e.g. "ur") + SyncRobot
        if cfg.controller not in Controller:
            raise KeyError(f"Unknown CRI controller '{cfg.controller}'. Available: {sorted(Controller)}")
        self._controller = Controller[cfg.controller](**cfg.controller_kwargs)
        self._robot = SyncRobot(self._controller)
        if cfg.tcp is not None:
            self._robot.tcp = tuple(cfg.tcp)

        # non-blocking joint streaming (degrees), provided by the CRI controller
        self._stream = getattr(self._controller, cfg.stream_method, None)
        if self._stream is None:
            warnings.warn(
                f"{cfg.controller} has no '{cfg.stream_method}'; falling back to "
                "SyncRobot.move_joints, which blocks per command."
            )
            self._stream = self._robot.move_joints
        self._gripper = getattr(self._controller, cfg.gripper_method) if cfg.gripper_method else None

        # safety
        self._lo = np.deg2rad(cfg.joint_lower_deg)
        self._hi = np.deg2rad(cfg.joint_upper_deg)
        self._max_dq = np.deg2rad(cfg.max_dq_deg_s)
        self._blend = cfg.blend_seconds
        self._q_start = None   # robot pose at first command
        self._q_cmd = None     # last command sent
        self._t_start = None
        self._t_last = None
        self._warned = False

        print(f"[robot] connected: {self._robot.info}")

    @property
    def dof(self) -> int:
        return self._dof

    def get_joints(self) -> np.ndarray:
        """Measured joint angles (rad)."""
        return np.deg2rad(np.asarray(self._robot.joint_angles, dtype=float)[: self._dof])

    def command_joints(self, q_target: np.ndarray) -> None:
        """Stream a joint target (rad) through the safety filter."""
        q_target = np.asarray(q_target, dtype=float)
        now = time.monotonic()
        if self._q_cmd is None:
            self._q_start = self.get_joints()
            self._q_cmd = self._q_start.copy()
            self._t_start = self._t_last = now

        # startup blend
        if self._blend > 0:
            s = min((now - self._t_start) / self._blend, 1.0)
            q_target = (1 - s) * self._q_start + s * q_target

        # velocity limit (dt capped so a stall can't permit a big jump)
        dt = min(max(now - self._t_last, 1e-3), 0.1)
        step = np.clip(q_target - self._q_cmd, -self._max_dq * dt, self._max_dq * dt)
        q = self._q_cmd + step

        # joint limits
        q_safe = np.clip(q, self._lo, self._hi)
        if not self._warned and not np.allclose(q, q_safe):
            print("[robot] WARNING: joint command clipped to limits")
            self._warned = True

        self._stream(np.rad2deg(q_safe))
        self._q_cmd, self._t_last = q_safe, now

    def command_gripper(self, value: float) -> None:
        """Gripper target in [0, 1] (0 = open); ignored if no gripper_method is configured."""
        if self._gripper is not None:
            self._gripper(float(value))

    def stop(self) -> None:
        for name in ("servo_stop", "stop"):
            fn = getattr(self._controller, name, None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:  # keep shutting down
                    print(f"[robot] {name} failed: {e}")
                return

    def close(self) -> None:
        self._robot.close()


def make_arm(robot: str) -> CriArm:
    """Build the robot registered under `robot` in config.ROBOT_CONFIG_MAP."""
    return CriArm(config.get(config.ROBOT_CONFIG_MAP, robot, "robot"))


# --------------------------------------------------------------------------- standalone
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True, help="Robot type, key in config.ROBOT_CONFIG_MAP")
    ap.add_argument("--hz", type=float, default=5.0, help="Print rate")
    args = ap.parse_args()

    arm = make_arm(args.robot)
    print("\nRobot active, waiting for input (none in standalone mode, no motion). Ctrl-C to exit.\n")
    try:
        while True:
            q = np.rad2deg(arm.get_joints())
            print("\rq(deg) = " + " ".join(f"{x:+7.1f}" for x in q), end="", flush=True)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\n")
    finally:
        arm.stop()
        arm.close()


if __name__ == "__main__":
    main()
