"""Single entry point: leader -> communication -> robot.

    python run.py --robot ur10       # teleop real robot via CRI

Run check_alignment.py first to confirm GELLO and robot agree.
"""

import argparse

import config
from communication.bridge import LeaderArmBridge
from robot.robot import make_arm


def build(robot: str, leader_port=None):
    """Create (teleop config, leader, arm) for a robot type."""
    from gello.gello import make_leader

    tcfg = config.get(config.TELEOP_CONFIG_MAP, robot, "robot")
    arm = make_arm(robot)
    try:
        # resolve GELLO's 2*pi ambiguity against the robot's current pose
        leader = make_leader(tcfg.leader, port=leader_port, start_joints=arm.get_joints())
    except Exception:
        arm.close()
        raise
    return tcfg, leader, arm


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True, help="Robot type, key in config.TELEOP_CONFIG_MAP")
    ap.add_argument("--leader-port", default=None, help="Override GELLO serial port (e.g. COM5)")
    ap.add_argument("--print", dest="verbose", action="store_true", help="Print leader joints at ~5 Hz")
    ap.add_argument("--seconds", type=float, default=None, help="Stop after N seconds")
    args = ap.parse_args()

    tcfg, leader, arm = build(args.robot, args.leader_port)
    try:
        print("SAFETY: workspace clear, E-stop within reach. Teleop starting now; Ctrl-C stops.")
        LeaderArmBridge(tcfg, leader, arm).run(verbose=args.verbose, max_seconds=args.seconds)
    finally:
        arm.stop()
        arm.close()
        leader.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
