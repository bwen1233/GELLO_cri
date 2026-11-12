# GELLO_cri

GELLO leader-arm teleoperation for robots driven through CRI (Common Robot Interface).

```
gello   ──►  communication  ──►  robot
(GELLO)      (read & stream)     (safety → installed cri package)
```

## Layout

```
GELLO_cri/
├── config.py                 # ALL configuration (Python registry, like gello's PORT_CONFIG_MAP)
│                             #   GELLO_CONFIG_MAP  name   -> U2D2 serial, joint ids / signs / offsets / gripper
│                             #   ROBOT_CONFIG_MAP  type   -> CRI controller, IP, limits, home, safety
│                             #   TELEOP_CONFIG_MAP type   -> which GELLO drives it, rate, gripper on/off
├── run.py                    # teleop: python run.py --robot ur10
├── check_alignment.py        # external safety check, no motion: GELLO vs robot joint difference
├── gello/                    # GELLO LAYER: leader-arm device code
│   ├── gello.py              # GELLO: auto-detect by U2D2 serial, Dynamixel -> joints in robot convention
│   ├── dxl_tool.py           # scan bus, set servo IDs, watch live positions
│   ├── drivers/dynamixel.py  # Dynamixel servo driver
│   ├── calibrate.py          # find joint offsets for a GELLO
├── communication/
│   └── bridge.py             # LeaderArmBridge: read leader -> command robot at rate_hz
├── robot/                    # ROBOT LAYER
│   ├── robot.py              # robot type -> CRI SyncRobot, rad <-> deg, startup safety
│   └── common_robot_interface/  # CRI, git submodule (bwen1233/common_robot_interface)
└── urdf/ur5.urdf             # reference only (kinematics come from CRI)
```

`gello` and `robot` never import each other; `config.py` is the only shared module.

## Install

```bash
git clone --recurse-submodules <this repo>      # or, in an existing clone: git submodule update --init
conda create -n gello_cri python=3.11
conda activate gello_cri
pip install -r requirements.txt
pip install -e robot/common_robot_interface      # optional: robot/robot.py already imports the submodule directly
```

CRI is the git submodule `robot/common_robot_interface`; `robot/robot.py` puts it first on the import path,
so that copy is used even if another CRI is installed. Change CRI there, commit and push it in the
submodule, then commit the new submodule pointer in this repo.

## Use

The connected GELLO is found automatically: each `GELLO_CONFIG_MAP` entry has its U2D2's USB `serial`,
and the one plugged in is picked. With several plugged in, choose with `--leader <name>` (e.g. `--leader ur`).

1. **Calibrate the GELLO** (once per unit; only the GELLO is connected). Set its reference pose (degrees) in
   `CALIB_POSE_MAP` at the top of `gello/calibrate.py`, then run:
   ```bash
   python gello/calibrate.py
   ```
   It asks you to press SPACE three times: GELLO in the reference pose, gripper fully open, gripper fully
   closed. It then prints `joint_offsets`, `joint_signs` and `gripper` and asks `[y/N]`: press `y` to write
   them into that GELLO's entry (e.g. `GELLO_CONFIG_MAP["ur"]`) in `config.py`, or paste them by hand.

   Each side can also be run on its own (no bridge, so nothing flows between them):
   ```bash
   python gello/gello.py                     # read GELLO, print joints (input only)
   python robot/robot.py --robot ur10        # activate robot, print joints (no input, no motion)
   ```

2. **Safety check, no motion** (not run automatically; run it yourself before teleop):
   ```bash
   python check_alignment.py --robot ur10
   ```

3. **Teleoperate:**
   ```bash
   python run.py --robot ur10 --print
   ```
   Motion starts immediately (eased in over `blend_seconds`). Ctrl-C stops.

Every command goes through `robot.robot.CriArm.command_joints`: it eases in from the robot's pose over
`blend_seconds`, limits joint speed to `max_dq_deg_s`, and clips to the joint limits.

Testing the GELLO alone (no robot): `python gello/gello.py`.

## Adding things

- **New GELLO:** give it its own U2D2, add an entry to `GELLO_CONFIG_MAP` under a readable name with
  `serial=` set to that U2D2's USB serial (shown by `python gello/gello.py` when nothing matches), then
  point the robot's `TELEOP_CONFIG_MAP` entry at that name.
- **New robot type:** add entries to `ROBOT_CONFIG_MAP` (CRI controller name, e.g. `"ur"`, + args) and
  `TELEOP_CONFIG_MAP` (which GELLO drives it).
- **Gripper:** implement it in CRI, set `gripper_method` in `ROBOT_CONFIG_MAP` and
  `use_gripper=True` in `TELEOP_CONFIG_MAP`.

## Notes

- `robot.py` streams joints via `stream_method` (default `servo_joints`) on the CRI
  controller. CRI's UR controller does not have it yet; until it is added, `robot.py` falls
  back to the blocking `SyncRobot.move_joints` with a warning.
- GELLO driver and calibration code come from
  [gello_software](https://github.com/wuphilipp/gello_software); the full upstream copy was
  removed and is still in this repo's git history.
