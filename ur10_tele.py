#!/usr/bin/env python3
import argparse
import csv
import os
import sys
import time
from typing import Optional, Sequence

import numpy as np

# GELLO
from gello.agents.gello_agent import GelloAgent

# UR RTDE
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# Pinocchio
try:
    import pinocchio as pin
except Exception as e:
    pin = None


DEFAULT_UR_IP = "192.168.1.141"

DEFAULT_GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0WR4-if00-port0"

DEFAULT_URDF = os.path.expanduser("~/tmp/urdf/ur10.urdf")
DEFAULT_EE_FRAME = "tool0"

# Optional: custom gripper TCP offset w.r.t. tool0 (meters)
# Set to None if you don't want TCP shift.
DEFAULT_TCP_XYZ = (0.0, 0.0, 0.18)   # example; change to your CAD value

# RTDE servoJ settings (start conservative)
DEFAULT_RATE_HZ = 100.0
DEFAULT_LOOKAHEAD = 0.10
DEFAULT_GAIN = 200
DEFAULT_BLEND_SECONDS = 4.0

# Joint velocity clamp (rad/s) for UR10 (conservative)
DEFAULT_MAX_DQ = (0.4, 0.4, 0.4, 0.7, 0.7, 0.9)

# Leader -> UR mapping (optional)
DEFAULT_JOINT_REORDER = None                 # e.g. (0,1,2,3,4,5)
DEFAULT_JOINT_SIGN = None                    # e.g. (1,1,-1,1,1,1)
DEFAULT_JOINT_BIAS = None                    # e.g. (0,0,0,0,0,0)

# Jacobian evaluation / twist frame
DEFAULT_JAC_QREF = "actual"                  # "actual" or "cmd"
DEFAULT_TWIST_FRAME = "world_aligned"        # "world_aligned" or "local"

# Logging
DEFAULT_CSV_PATH = "teleop_log.csv"
DEFAULT_PRINT = True


def parse_list(arg: Optional[object], cast=float) -> Optional[np.ndarray]:
    if arg is None:
        return None
    if isinstance(arg, (list, tuple, np.ndarray)):
        return np.array([cast(x) for x in arg])
    if isinstance(arg, str):
        if arg.strip() == "":
            return None
        parts = [p.strip() for p in arg.split(",") if p.strip() != ""]
        return np.array([cast(p) for p in parts])
    raise TypeError(f"Unsupported type for parse_list: {type(arg)}")


def find_frame_id(model: "pin.Model", preferred: Sequence[str]) -> int:
    for name in preferred:
        if model.existFrame(name):
            return model.getFrameId(name)
    # last resort: try to use the last frame (often tool-ish), but warn
    return model.nframes - 1


def clamp_dq(q_prev: np.ndarray, q_target: np.ndarray, dt: float, max_dq: np.ndarray) -> np.ndarray:
    dq = (q_target - q_prev) / dt
    dq = np.clip(dq, -max_dq, max_dq)
    return q_prev + dq * dt


def lowpass(prev: np.ndarray, new: np.ndarray, alpha: float) -> np.ndarray:
    # alpha in [0,1]; larger = less smoothing
    return alpha * new + (1.0 - alpha) * prev


def main():
    ap = argparse.ArgumentParser(description="GELLO leader -> UR10 RTDE servoJ teleop with v_cmd extraction (Scheme A).")

    ap.add_argument("--ur-ip", type=str, default=DEFAULT_UR_IP)
    ap.add_argument(
        "--gello-port",
        type=str,
        default=DEFAULT_GELLO_PORT,
        help="Use the stable /dev/serial/by-id/... path.",
    )

    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Control rate in Hz. Start conservative (50Hz).")
    ap.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD, help="RTDE servoJ lookahead_time (s).")
    ap.add_argument("--gain", type=int, default=DEFAULT_GAIN, help="RTDE servoJ gain. Start conservative.")
    ap.add_argument("--blend-seconds", type=float, default=DEFAULT_BLEND_SECONDS, help="Initial blending time to avoid first-step jumps.")
    ap.add_argument(
        "--max-dq",
        type=str,
        default=",".join(map(str, DEFAULT_MAX_DQ)),
        help="Per-joint velocity clamp (rad/s), comma-separated.",
    )

    # Mapping options (leader -> UR)
    ap.add_argument(
        "--joint-reorder",
        type=str,
        default=DEFAULT_JOINT_REORDER,
        help="Comma-separated indices to reorder leader joints into UR order (e.g. '0,1,2,3,4,5').",
    )
    ap.add_argument(
        "--joint-sign",
        type=str,
        default=DEFAULT_JOINT_SIGN,
        help="Comma-separated +/-1 to flip mapped joints (e.g. '1,1,-1,1,1,1').",
    )
    ap.add_argument(
        "--joint-bias",
        type=str,
        default=DEFAULT_JOINT_BIAS,
        help="Comma-separated joint bias (rad) added after reorder/sign, length 6.",
    )

    # Jacobian/URDF
    ap.add_argument(
        "--urdf",
        type=str,
        default=DEFAULT_URDF,
        help="Path to UR10 URDF. Required to compute v_cmd = J(q_ref)*dq_cmd with Pinocchio.",
    )
    ap.add_argument(
        "--ee-frame",
        type=str,
        default=DEFAULT_EE_FRAME,
        help="End-effector frame name in URDF. If not set, script tries common names.",
    )
    ap.add_argument(
        "--jac-qref",
        type=str,
        choices=["actual", "cmd"],
        default=DEFAULT_JAC_QREF,
        help="Use q_actual or q_cmd for Jacobian evaluation.",
    )
    ap.add_argument(
        "--twist-frame",
        type=str,
        choices=["world_aligned", "local"],
        default=DEFAULT_TWIST_FRAME,
        help="Pinocchio Jacobian reference frame for twist.",
    )

    # Logging
    ap.add_argument("--csv", type=str, default=DEFAULT_CSV_PATH, help="CSV log path.")
    ap.add_argument("--no-csv", action="store_true", help="Disable CSV logging.")
    ap.add_argument("--print", action="store_true", help="Print q_cmd and v_cmd at ~5Hz.")

    args = ap.parse_args()

    dt = 1.0 / float(args.rate)
    max_dq = parse_list(args.max_dq, float)
    if max_dq is None or len(max_dq) != 6:
        raise ValueError("--max-dq must be 6 values (rad/s).")

    reorder = parse_list(args.joint_reorder, int)
    jsign = parse_list(args.joint_sign, float)
    jbias = parse_list(args.joint_bias, float)

    # --- Build kinematics (Pinocchio) ---
    if pin is None:
        raise RuntimeError(
            "Pinocchio is not available in this environment. Install pinocchio (pip/apt) or run without v_cmd (not recommended)."
        )
    if args.urdf is None or not os.path.exists(args.urdf):
        raise RuntimeError(
            "URDF path missing or not found. Provide --urdf /path/to/ur10.urdf to compute Jacobian and v_cmd."
        )

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()

    preferred_frames = []
    if args.ee_frame:
        preferred_frames.append(args.ee_frame)
    # Common UR frame names across different URDFs
    preferred_frames += ["tool0", "tcp", "ee_link", "wrist_3_link", "flange", "tool0_controller"]
    ee_frame_id = find_frame_id(model, preferred_frames)

    if args.twist_frame == "world_aligned":
        ref_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    else:
        ref_frame = pin.ReferenceFrame.LOCAL

    # --- Connect GELLO leader ---
    gello = GelloAgent(port=args.gello_port)

    # --- Connect UR via RTDE ---
    rtde_c = RTDEControl(args.ur_ip)
    rtde_r = RTDEReceive(args.ur_ip)

    # Safety gate
    print("Connected to UR and GELLO.")
    print("SAFETY: Ensure workspace is clear and E-Stop is reachable.")
    token = input("Type START and press Enter to enable motion: ").strip()
    if token != "START":
        print("Aborted (did not receive START).")
        return

    # Initialize command state from actual robot to avoid first-step jumps
    q_actual = np.array(rtde_r.getActualQ(), dtype=float)
    q_cmd = q_actual.copy()
    q_cmd_prev = q_cmd.copy()
    dq_cmd_filt = np.zeros(6, dtype=float)

    # Optional blending
    t_start = time.time()
    blend_T = max(0.0, float(args.blend_seconds))

    # CSV logging
    csv_f = None
    csv_w = None
    if not args.no_csv:
        csv_f = open(args.csv, "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow([
            "t",
            *[f"q_cmd_{i}" for i in range(6)],
            *[f"dq_cmd_{i}" for i in range(6)],
            *[f"v_cmd_{i}" for i in range(6)],
            *[f"q_actual_{i}" for i in range(6)],
            *[f"tcp_pose_{i}" for i in range(6)],
            *[f"tcp_speed_{i}" for i in range(6)],
        ])
        csv_f.flush()

    # Print throttling
    last_print = 0.0

    print("Teleop running. Ctrl-C to stop.")
    try:
        while True:
            t0 = time.time()

            # --- Read leader ---
            q_all = np.array(gello.act(obs={}), dtype=float)

            # GELLO may return 7 values (6 arm joints + 1 trigger/gripper).
            # For UR10 teleop we use only the first 6 joints and ignore the 7th.
            if q_all.shape[0] == 6:
                q_leader = q_all
            elif q_all.shape[0] == 7:
                q_leader = q_all[:6]   # ignore gripper/trigger
            else:
                raise RuntimeError(f"Expected 6 or 7 values from GELLO, got shape {q_all.shape}")

            # --- Map leader -> UR target ---
            q_map = q_leader.copy()
            if reorder is not None:
                if len(reorder) != 6:
                    raise ValueError("--joint-reorder must have 6 indices.")
                q_map = q_map[reorder]
            if jsign is not None:
                if len(jsign) != 6:
                    raise ValueError("--joint-sign must have 6 values.")
                q_map = q_map * jsign
            if jbias is not None:
                if len(jbias) != 6:
                    raise ValueError("--joint-bias must have 6 values.")
                q_map = q_map + jbias

            q_target = q_map

            # --- Blend-in (avoid sudden jumps at start) ---
            if blend_T > 0.0:
                s = (time.time() - t_start) / blend_T
                s = float(np.clip(s, 0.0, 1.0))
                q_target = (1.0 - s) * q_actual + s * q_target

            # --- Rate limiting in joint space ---
            q_cmd = clamp_dq(q_cmd, q_target, dt, max_dq)

            # --- Compute dq_cmd and (filtered) ---
            dq_cmd = (q_cmd - q_cmd_prev) / dt
            # Simple low-pass on dq to reduce differentiation noise; alpha ~ 0.2 is conservative
            dq_cmd_filt = lowpass(dq_cmd_filt, dq_cmd, alpha=0.2)
            q_cmd_prev = q_cmd.copy()

            # --- Read robot state for Jacobian reference ---
            q_actual = np.array(rtde_r.getActualQ(), dtype=float)
            tcp_pose = np.array(rtde_r.getActualTCPPose(), dtype=float)     # executed pose
            tcp_speed = np.array(rtde_r.getActualTCPSpeed(), dtype=float)   # executed twist

            if args.jac_qref == "actual":
                q_ref = q_actual
            else:
                q_ref = q_cmd

            # --- Jacobian and v_cmd (commanded twist definition) ---
            # Pinocchio expects q in model configuration space; for UR this is 6 joint angles.
            pin.forwardKinematics(model, data, q_ref)
            pin.updateFramePlacements(model, data)
            J = pin.computeFrameJacobian(model, data, q_ref, ee_frame_id, ref_frame)  # 6x6
            v_cmd = J @ dq_cmd_filt  # 6D twist

            # --- Send servoJ (joint target) ---
            rtde_c.servoJ(q_cmd.tolist(), 0.0, 0.0, dt, float(args.lookahead), int(args.gain))

            # --- Logging ---
            t = time.time()
            if csv_w is not None:
                csv_w.writerow([
                    t,
                    *q_cmd.tolist(),
                    *dq_cmd_filt.tolist(),
                    *v_cmd.tolist(),
                    *q_actual.tolist(),
                    *tcp_pose.tolist(),
                    *tcp_speed.tolist(),
                ])
                # flush occasionally
                if int(t * 10) % 10 == 0:
                    csv_f.flush()

            if args.print and (t - last_print) > 0.2:
                last_print = t
                print("q_cmd:", np.round(q_cmd, 3), "v_cmd:", np.round(v_cmd, 3))

            # --- Timing ---
            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("Stopping (KeyboardInterrupt)...")
    finally:
        # Stop servo safely
        try:
            rtde_c.servoStop()
        except Exception:
            pass
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        if csv_f is not None:
            csv_f.flush()
            csv_f.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
