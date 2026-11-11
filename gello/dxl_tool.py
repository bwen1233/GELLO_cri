"""Dynamixel helper: scan the bus, change IDs, and watch live positions.

Examples (port defaults to COM5, baud 57600):
    python gello/dxl_tool.py scan
    python gello/dxl_tool.py setid 1 5          # change motor ID 1 -> 5 (only that motor should have ID 1)
    python gello/dxl_tool.py assign 3           # chain has joints 1..3 plugged in; give the new one ID 3
    python gello/dxl_tool.py watch              # live positions; move a joint to see which ID changes
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_ID = 7
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132


def to_signed32(v: int) -> int:
    return v - (1 << 32) if v >= (1 << 31) else v


class Bus:
    def __init__(self, port: str, baud: int):
        self.port = PortHandler(port)
        if not self.port.openPort():
            sys.exit(f"Cannot open {port}")
        if not self.port.setBaudRate(baud):
            sys.exit(f"Cannot set baud {baud}")
        self.pk = PacketHandler(2.0)

    def close(self):
        self.port.closePort()

    def scan(self) -> dict:
        """Return {id: model} for every motor that answers a broadcast ping."""
        ids, res = self.pk.broadcastPing(self.port)
        if res != COMM_SUCCESS:
            return {}
        return {i: v[0] for i, v in ids.items()}

    def position(self, dxl_id: int):
        v, res, _ = self.pk.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return to_signed32(v) if res == COMM_SUCCESS else None

    def set_id(self, old: int, new: int):
        self.pk.write1ByteTxRx(self.port, old, ADDR_TORQUE_ENABLE, 0)  # ID is EEPROM: torque must be off
        res, err = self.pk.write1ByteTxRx(self.port, old, ADDR_ID, new)
        if res != COMM_SUCCESS or err:
            sys.exit(f"Failed to change ID {old} -> {new}: {self.pk.getTxRxResult(res)} {err}")
        time.sleep(0.1)
        _, res, _ = self.pk.ping(self.port, new)
        if res != COMM_SUCCESS:
            sys.exit(f"Wrote ID {new} but it does not answer")
        print(f"ID {old} -> {new} OK")


def print_scan(found: dict):
    if not found:
        print("No motors answered.")
    for i in sorted(found):
        print(f"  ID {i:3d}  model {found[i]}")


def cmd_assign(bus: Bus, k: int):
    """Joints 1..k are plugged in, 1..k-1 already have correct IDs. Give the new one ID k."""
    found = bus.scan()
    print_scan(found)
    expected = set(range(1, k))
    missing = expected - found.keys()
    new = sorted(set(found) - expected - {k})
    if missing:
        sys.exit(
            f"IDs {sorted(missing)} do not answer: the new motor probably shares one of these IDs.\n"
            f"Plug the new motor in ALONE, run `scan`, then `setid <its id> {k}`."
        )
    if k in found and not new:
        print(f"Joint {k} already has ID {k}. Nothing to do.")
        return
    if k in found or len(new) != 1:
        sys.exit(f"Expected exactly one new motor, found {new}. Plug in only joints 1..{k}.")
    bus.set_id(new[0], k)


def cmd_watch(bus: Bus):
    ids = sorted(bus.scan())
    if not ids:
        sys.exit("No motors answered.")
    base = {i: bus.position(i) for i in ids}
    print("Move one joint at a time; the ID with the biggest change is marked. Ctrl+C to stop.")
    try:
        while True:
            pos = {i: bus.position(i) for i in ids}
            delta = {i: abs(pos[i] - base[i]) if pos[i] is not None and base[i] is not None else 0 for i in ids}
            moving = max(delta, key=delta.get)
            line = "  ".join(
                f"{'*' if i == moving and delta[i] > 50 else ' '}ID{i}:{pos[i]!s:>6}" for i in ids
            )
            print("\r" + line, end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=57600)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    s = sub.add_parser("setid")
    s.add_argument("old", type=int)
    s.add_argument("new", type=int)
    a = sub.add_parser("assign")
    a.add_argument("k", type=int)
    sub.add_parser("watch")
    args = ap.parse_args()

    bus = Bus(args.port, args.baud)
    try:
        if args.cmd == "scan":
            print_scan(bus.scan())
        elif args.cmd == "setid":
            bus.set_id(args.old, args.new)
        elif args.cmd == "assign":
            cmd_assign(bus, args.k)
        elif args.cmd == "watch":
            cmd_watch(bus)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
