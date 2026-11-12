import argparse
import os
import pathlib
import re
import sys


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

import config
from gello.gello import detect_leader, find_port


# Reference pose (degrees, one value per arm joint) you physically put each GELLO in for calibration.
# Key = GELLO name in config.GELLO_CONFIG_MAP.
CALIB_POSE_MAP = {
    "ur": (0, -90, 90, -90, -90, 0),
    "mg400": (0, 0, 0, 0),
}


def best_offsets(raw: np.ndarray, start: np.ndarray, signs: np.ndarray) -> np.ndarray:
    """For each joint, the multiple of pi/2 in [-8pi, 8pi] that maps raw -> start best."""
    candidates = np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1)
    out = []
    for i in range(len(start)):
        err = np.abs(signs[i] * (raw[i] - candidates) - start[i])
        out.append(candidates[np.argmin(err)])
    return np.array(out)


def wait_for_space(prompt: str) -> None:
    """Block until the user presses the space bar (Ctrl-C aborts)."""
    print(f"\n{prompt}\n  -> press SPACE to confirm", flush=True)
    if os.name == "nt":
        import msvcrt

        while True:
            ch = msvcrt.getwch()
            if ch == " ":
                return
            if ch == "\x03":
                raise KeyboardInterrupt
    else:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while sys.stdin.read(1) != " ":
                pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key() -> str:
    """Return one key press (lower-case), without needing Enter."""
    if os.name == "nt":
        import msvcrt

        ch = msvcrt.getwch()
        if ch == "\x03":
            raise KeyboardInterrupt
        return ch.lower()
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def write_config(name: str, offsets_text: str, signs_text: str, grip_text: str = None) -> None:
    """Replace joint_offsets / joint_signs / gripper inside GELLO_CONFIG_MAP[name] in config.py."""
    path = pathlib.Path(config.__file__)
    src = path.read_text(encoding="utf-8")
    m = re.search(rf'^(\s*)"{re.escape(name)}"\s*:\s*GelloConfig\(', src, re.M)
    if not m:
        raise SystemExit(f'Could not find "{name}": GelloConfig( in {path}; paste the values by hand.')
    # the entry ends at the first line that closes it at the entry's own indentation: "<indent>),"
    end = re.compile(rf"^{m.group(1)}\),?\s*$", re.M).search(src, m.end())
    if not end:
        raise SystemExit(f"Could not find the end of the '{name}' entry; paste the values by hand.")
    block = src[m.end():end.start()]

    def sub(field: str, value: str, text: str) -> str:
        pat = re.compile(rf"{field}\s*=\s*\((?:[^()]|\([^()]*\))*\)", re.S)
        if not pat.search(text):
            raise SystemExit(f"'{field}' not found in the '{name}' entry; paste the values by hand.")
        return pat.sub(lambda _: f"{field}={value}", text, count=1)

    block = sub("joint_offsets", offsets_text, block)
    block = sub("joint_signs", signs_text, block)
    if grip_text is not None:
        block = sub("gripper", grip_text, block)
    path.write_text(src[: m.end()] + block + src[end.start():], encoding="utf-8")
    print(f'Updated GELLO_CONFIG_MAP["{name}"] in {path}')


def read_stable(driver, n: int = 10) -> np.ndarray:
    """Average a few readings (radians) to reduce noise."""
    return np.mean([driver.get_joints() for _ in range(n)], axis=0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leader", default=None, help="GELLO name in config.GELLO_CONFIG_MAP (e.g. ur); auto-detected if omitted")
    ap.add_argument("--joint-signs", type=int, nargs="+", default=None, help="Override signs from config")
    ap.add_argument("--port", default=None, help="Serial port; auto-detected from the U2D2 serial if omitted")
    args = ap.parse_args()

    from gello.drivers.dynamixel import DynamixelDriver

    name = args.leader or detect_leader()
    cfg = config.get(config.GELLO_CONFIG_MAP, name, "GELLO")
    n = len(cfg.joint_ids)
    signs = np.array(args.joint_signs if args.joint_signs else cfg.joint_signs, dtype=float)
    start_deg = config.get(CALIB_POSE_MAP, name, "calibration pose for GELLO")
    start = np.deg2rad(np.array(start_deg, dtype=float))
    if len(start) != n or len(signs) != n:
        raise SystemExit(f"GELLO '{name}' has {n} arm joints: CALIB_POSE_MAP['{name}'] and signs need {n} values.")

    ids = list(cfg.joint_ids) + ([cfg.gripper[0]] if cfg.gripper else [])
    driver = DynamixelDriver(ids, port=args.port or find_port(cfg.serial), baudrate=cfg.baudrate)
    driver.set_torque_mode(False)
    try:
        for _ in range(10):
            driver.get_joints()  # warm-up

        # 1) arm offsets
        wait_for_space(f"[1] Put GELLO in the reference pose (deg): {list(start_deg)}")
        raw = read_stable(driver)
        offsets = best_offsets(raw[:n], start, signs)
        result = signs * (raw[:n] - offsets)
        err = np.rad2deg(np.abs(result - start))
        print("    calibrated pose (deg): " + " ".join(f"{x:+7.1f}" for x in np.rad2deg(result)))
        if np.any(err > 20):
            print(f"    WARNING: joints {list(np.where(err > 20)[0] + 1)} are >20 deg from the reference; "
                  "check the pose and joint_signs.")

        # 2) gripper, measured at both ends
        grip = None
        if cfg.gripper:
            wait_for_space("[2] Gripper fully OPEN (trigger released)")
            g_open = float(np.rad2deg(read_stable(driver)[-1]))
            print(f"    open   = {g_open:.1f} deg")
            wait_for_space("[3] Gripper fully CLOSED (trigger pressed all the way)")
            g_closed = float(np.rad2deg(read_stable(driver)[-1]))
            print(f"    closed = {g_closed:.1f} deg")
            if abs(g_open - g_closed) < 5:
                print("    WARNING: open and closed differ by <5 deg; did the trigger move?")
            grip = (cfg.gripper[0], round(g_open, 1), round(g_closed, 1))

        ind = " " * 8
        offsets_text = "(\n" + "".join(
            f"{ind}    {int(round(o / (np.pi / 2)))} * np.pi / 2,\n" for o in offsets
        ) + f"{ind})"
        signs_text = "(" + ", ".join(str(int(s)) for s in signs) + ")"
        grip_text = str(grip) if grip else None

        print(f'\nResult for GELLO_CONFIG_MAP["{name}"]:\n')
        print(f"{ind}joint_offsets={offsets_text},")
        print(f"{ind}joint_signs={signs_text},")
        if grip_text:
            print(f"{ind}gripper={grip_text},")

        print("\nWrite these into config.py now? [y/N]", flush=True)
        if read_key() == "y":
            write_config(name, offsets_text, signs_text, grip_text)
        else:
            print("Not written; paste the values above into config.py by hand.")
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
