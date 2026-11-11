

import argparse
import glob
import os
import pathlib
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

# allow running this file directly: put the project root on the import path
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from config import GelloConfig


# --------------------------------------------------------------------------- reading
@dataclass
class LeaderReading:
    q: np.ndarray


    gripper: Optional[float] = None
    """Gripper/trigger value in [0, 1] (0 = open), or None if the device has none."""


# --------------------------------------------------------------------------- ports
def list_ports() -> List[str]:
    """Human-readable list of serial ports, for error messages."""
    from serial.tools import list_ports as lp

    return [f"{p.device}  serial={p.serial_number}  {p.description}" for p in lp.comports()]


def find_port(serial: str) -> str:
    """Return the port (e.g. COM5 or /dev/ttyUSB0) whose USB adapter serial matches.

    `serial` is the FTDI serial printed on the U2D2/adapter, e.g. "FT8J0WR4".
    """
    # Linux: stable by-id path contains the serial
    if os.name != "nt":
        for path in glob.glob("/dev/serial/by-id/*"):
            if serial in path:
                return path

    from serial.tools import list_ports as lp

    for p in lp.comports():
        if p.serial_number and serial in p.serial_number:
            return p.device

    found = "\n  ".join(list_ports()) or "(none)"
    raise RuntimeError(
        f"No serial port found for leader serial '{serial}'.\n"
        f"Ports found:\n  {found}\n"
        "Plug in the GELLO, or pass the port explicitly with --leader-port."
    )


FTDI_VID = 0x0403  # U2D2 uses an FTDI chip


def _base_serial(serial: str) -> str:
    """FTDI serial without the channel letter Windows appends (FTAAMOOQA -> FTAAMOOQ)."""
    return serial[:-1] if os.name == "nt" and len(serial) == 9 and serial.endswith("A") else serial


def detect_leader() -> str:
    """Return the name (key in config.GELLO_CONFIG_MAP) of the one connected GELLO, matched by U2D2 serial."""
    import config
    from serial.tools import list_ports as lp

    ports_info = list(lp.comports())
    connected = [p.serial_number or "" for p in ports_info]
    found = [name for name, cfg in config.GELLO_CONFIG_MAP.items() if any(cfg.serial in c for c in connected)]

    # U2D2s (FTDI adapters) whose serial is not registered in any GELLO_CONFIG_MAP entry
    known_serials = [c.serial for c in config.GELLO_CONFIG_MAP.values()]
    unknown = [
        (p.device, _base_serial(p.serial_number))
        for p in ports_info
        if p.vid == FTDI_VID and p.serial_number and not any(s in p.serial_number for s in known_serials)
    ]
    for device, serial in unknown:
        print(f"[leader] unregistered U2D2 on {device}: serial={serial}")

    if len(found) == 1:
        print(f"[leader] detected GELLO '{found[0]}' (U2D2 {config.GELLO_CONFIG_MAP[found[0]].serial})")
        return found[0]
    ports = "\n  ".join(list_ports()) or "(none)"
    if not found:
        known = ", ".join(f"{n}={c.serial}" for n, c in config.GELLO_CONFIG_MAP.items())
        msg = f"No registered GELLO connected (registered: {known}).\nPorts found:\n  {ports}"
        if unknown:
            msg += (
                "\n\nTo register a new U2D2, add an entry to GELLO_CONFIG_MAP in config.py, e.g.:\n"
                f'    "my_gello": GelloConfig(serial="{unknown[0][1]}", joint_ids=(...), '
                "joint_offsets=(...), joint_signs=(...), gripper=(...)),\n"
                "or, if it replaced an existing GELLO's U2D2, update that entry's serial."
            )
        raise RuntimeError(msg)
    raise RuntimeError(f"Several GELLOs connected {found}; pick one with --leader.")


# --------------------------------------------------------------------------- leader
class GelloLeader:
    def __init__(
        self,
        cfg: GelloConfig,
        port: str,
        start_joints: Optional[np.ndarray] = None,
        smoothing_alpha: float = 0.99,
    ):

        from gello.drivers.dynamixel import DynamixelDriver

        ids = list(cfg.joint_ids)
        offsets = list(cfg.joint_offsets)
        signs = list(cfg.joint_signs)
        if cfg.gripper is not None:
            ids.append(cfg.gripper[0])
            offsets.append(0.0)
            signs.append(1)
            self._gripper_open_close = (np.deg2rad(cfg.gripper[1]), np.deg2rad(cfg.gripper[2]))
        else:
            self._gripper_open_close = None

        self._n_arm = len(cfg.joint_ids)
        self._offsets = np.array(offsets, dtype=float)
        self._signs = np.array(signs, dtype=float)
        self._alpha = smoothing_alpha
        self._last = None

        print(f"[leader] connecting to GELLO on {port}")
        self._driver = DynamixelDriver(ids, port=port, baudrate=cfg.baudrate)
        self._driver.set_torque_mode(False)

        if start_joints is not None:
            self._snap_offsets_to(np.asarray(start_joints, dtype=float))

    @property
    def dof(self) -> int:
        """Number of arm joints (gripper not included)."""
        return self._n_arm

    def _raw(self) -> np.ndarray:
        return (self._driver.get_joints() - self._offsets) * self._signs

    def _snap_offsets_to(self, start_joints: np.ndarray) -> None:
        current = self._raw()[: self._n_arm]
        assert start_joints.shape == current.shape, (
            f"start_joints has {start_joints.shape[0]} values, leader has {self._n_arm} joints"
        )
        turns = np.round((current - start_joints) / (2 * np.pi))
        self._offsets[: self._n_arm] += 2 * np.pi * turns * self._signs[: self._n_arm]

    def read(self) -> LeaderReading:
        pos = self._raw()
        if self._last is None:
            self._last = pos
        else:
            pos = self._last * (1 - self._alpha) + pos * self._alpha
            self._last = pos

        gripper = None
        if self._gripper_open_close is not None:
            g_open, g_close = self._gripper_open_close
            gripper = float(np.clip((pos[-1] - g_open) / (g_close - g_open), 0.0, 1.0))
        return LeaderReading(q=pos[: self._n_arm].copy(), gripper=gripper)

    def close(self) -> None:
        self._driver.close()


def make_leader(
    name: Optional[str] = None, port: Optional[str] = None, start_joints: Optional[np.ndarray] = None
) -> GelloLeader:
    """Build the GELLO registered as `name` in config.GELLO_CONFIG_MAP (auto-detected if None)."""
    import config

    name = name or detect_leader()
    cfg = config.get(config.GELLO_CONFIG_MAP, name, "GELLO")
    return GelloLeader(cfg, port=port or find_port(cfg.serial), start_joints=start_joints)


# --------------------------------------------------------------------------- standalone
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leader", default=None, help="GELLO name in config.GELLO_CONFIG_MAP (e.g. ur); auto-detected if omitted")
    ap.add_argument("--port", default=None, help="Serial port; auto-detected from the U2D2 serial if omitted")
    ap.add_argument("--hz", type=float, default=10.0, help="Print rate")
    args = ap.parse_args()

    leader = make_leader(args.leader, port=args.port)
    print("\nReading GELLO (torque off, move it by hand). Ctrl-C to exit.\n")
    try:
        while True:
            r = leader.read()
            line = "q(deg) = " + " ".join(f"{x:+7.1f}" for x in np.rad2deg(r.q))
            if r.gripper is not None:
                line += f" | gripper = {r.gripper:4.2f}"
            print("\r" + line, end="", flush=True)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\n")
    finally:
        leader.close()


if __name__ == "__main__":
    main()
