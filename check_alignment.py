import numpy as np
import time
from typing import Optional, Sequence
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gello.agents.gello_agent import GelloAgent

DEFAULT_UR_IP = "192.168.1.141"

DEFAULT_GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0WR4-if00-port0"



def alignment_check(
    gello: GelloAgent,
    rtde_r: RTDEReceive,
    leader_idx: Sequence[int] = (0, 1, 2, 3, 4, 5),
    hz: float = 5.0,
) -> None:
    """
    Prints UR vs GELLO joint difference continuously (degrees), without sending any command.
    Exit with Ctrl-C.
    """
    period = 1.0 / float(hz)
    print("\nAlignment check (no motion). Ctrl-C to exit this check.\n")
    print("Format: dq(deg) = (GELLO - UR) for joints 0..5; also prints max/rms.\n")

    try:
        while True:
            q_ur = np.array(rtde_r.getActualQ(), dtype=float)  # (6,)

            q_all = np.array(gello.act(obs={}), dtype=float)
            if q_all.shape[0] < 6:
                raise RuntimeError(f"GELLO returned too few values: shape={q_all.shape}")

            q_g = q_all[list(leader_idx)]  # pick first 6 (or any mapping you set)

            # dq = q_g - q_ur
            dq = q_g
            dq_deg = dq * 180.0 / np.pi

            max_abs_deg = float(np.max(np.abs(dq_deg)))
            rms_deg = float(np.sqrt(np.mean(dq_deg**2)))

            msg = (
                "dq(deg)="
                + " ".join([f"{d:+6.1f}" for d in dq_deg])
                + f" | max={max_abs_deg:5.1f}  rms={rms_deg:5.1f}"
            )

            print("\r" + msg, end="", flush=True)
            time.sleep(period)

    except KeyboardInterrupt:
        print("\n\nAlignment check finished.\n")




def main():
    gello = GelloAgent(port=DEFAULT_GELLO_PORT)
    rtde_r = RTDEReceive(DEFAULT_UR_IP)


    alignment_check(gello, rtde_r, leader_idx=(0, 1, 2, 3, 4, 5), hz=5.0)

if __name__ == "__main__":
    main()