

import time
from typing import Optional

import numpy as np

from config import TeleopConfig


class LeaderArmBridge:
    def __init__(self, cfg: TeleopConfig, leader, arm):
        if leader.dof != arm.dof:
            raise ValueError(f"Leader has {leader.dof} joints but robot has {arm.dof}")
        self.cfg = cfg
        self.leader = leader
        self.arm = arm
        self.dt = 1.0 / cfg.rate_hz

    def run(self, verbose: bool = False, max_seconds: Optional[float] = None) -> None:
        t_start = time.time()
        last_print = 0.0
        print("Teleop running. Ctrl-C to stop.")
        try:
            while True:
                t0 = time.time()
                if max_seconds is not None and t0 - t_start > max_seconds:
                    break

                r = self.leader.read()
                self.arm.command_joints(r.q)
                if self.cfg.use_gripper and r.gripper is not None:
                    self.arm.command_gripper(r.gripper)

                if verbose and t0 - last_print > 0.2:
                    last_print = t0
                    print("leader(deg):", np.round(np.rad2deg(r.q), 1))

                sleep = self.dt - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\nStopping (Ctrl-C)...")
