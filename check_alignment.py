

import argparse
import time

import numpy as np

from run import build


def report(q_leader: np.ndarray, q_robot: np.ndarray, max_diff_deg: float) -> str:
    d = np.rad2deg(q_leader - q_robot)
    worst = float(np.max(np.abs(d)))
    flag = "OK " if worst <= max_diff_deg else "FAR"
    return (
        f"[{flag}] diff(deg) = "
        + " ".join(f"{x:+6.1f}" for x in d)
        + f" | max={worst:5.1f}  rms={np.sqrt(np.mean(d ** 2)):5.1f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True, help="Robot type, key in config.TELEOP_CONFIG_MAP")
    ap.add_argument("--leader-port", default=None, help="Override GELLO serial port (e.g. COM5)")
    ap.add_argument("--max-diff", type=float, default=45.0, help="Flag as FAR above this (deg)")
    ap.add_argument("--hz", type=float, default=5.0, help="Print rate")
    args = ap.parse_args()

    _, leader, arm = build(args.robot, args.leader_port)
    print("\nAlignment check (no motion). Ctrl-C to exit.\n")
    try:
        while True:
            line = report(leader.read().q, arm.get_joints(), args.max_diff)
            print("\r" + line, end="", flush=True)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\n")
    finally:
        arm.close()
        leader.close()


if __name__ == "__main__":
    main()
