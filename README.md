# GELLO_simple

Based on [gello_software](https://github.com/wuphilipp/gello_software), a minimal wrapper for using a
GELLO leader arm to teleoperate a UR robot directly over `ur_rtde` — no ZMQ
server/client split, no camera nodes, one script, one process.

## Structure

```
GELLO_simple/
├── check_alignment.py   # read-only diagnostic: prints GELLO vs UR joint offset, no motion
├── ur10_tele.py          # standalone GELLO -> UR teleop loop (RTDE servoJ)
├── urdf/
│   └── ur5.urdf           # generic UR5 URDF (generated from ROS Humble's ur_description via xacro)
└── gello_software/       # vendored copy of gello_software (flattened, no submodules)
    └── gello/             # the `gello` package (GelloAgent, Dynamixel driver, robot interfaces...)
```

`gello_software` here is a **flattened copy**, not a git submodule — it won't
auto-update from upstream. `third_party/` (DynamixelSDK, mujoco_menagerie)
was dropped: DynamixelSDK now comes from PyPI (`dynamixel-sdk` in
`requirements.txt`), and mujoco_menagerie (MuJoCo sim assets) isn't needed
since this repo only targets real UR hardware.

`urdf/ur5.urdf` uses generic/default UR5 kinematic parameters (not
calibrated to any specific physical robot's factory offsets) — good enough
for the Jacobian/TCP-twist computation these scripts do, but if you need
robot-accurate forward kinematics, regenerate it with your UR's actual
kinematics calibration (see `ur_description`'s docs on extracting
`kinematics_parameters_file` from the controller).

## Install

**Option A — venv/uv:**
```bash
git clone git@github.com:bwen1233/GELLO_simple.git
cd GELLO_simple

uv venv --python 3.11
source .venv/bin/activate
uv pip install -r gello_software/requirements.txt
uv pip install -e gello_software
```

**Option B — conda:**
```bash
git clone git@github.com:bwen1233/GELLO_simple.git
cd GELLO_simple

conda create -n gello python=3.11
conda activate gello
pip install -r gello_software/requirements.txt
pip install -e gello_software
```
(Most of these deps — `ur-rtde`, `pyrealsense2`, `xarm-python-sdk`, etc. —
aren't reliably on conda-forge, so conda here is just used for the Python
interpreter/env; installs still go through `pip`.)

**Serial port permissions** (needed to talk to the GELLO's U2D2/FTDI adapter):
```bash
sudo usermod -aG dialout $USER
# log out/in or reboot for this to take effect
```

## Hardware setup

**1. Calibrate the GELLO leader arm.** This is a separate, required step —
don't skip straight to the alignment check. Set the GELLO and the UR to a
matching known pose, then run:
```bash
python gello_software/scripts/gello_get_offset.py \
    --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
    --joint-signs 1 1 -1 1 1 1 \
    --port /dev/serial/by-id/<your-GELLO-device>
```
Add the printed offsets to `PORT_CONFIG_MAP` in
`gello_software/gello/agents/gello_agent.py`, keyed by that same
`/dev/serial/by-id/...` path. (This repo already ships an entry for the
GELLO unit it was built with — if you're plugging that *exact* physical
device into a new machine it should resolve to the same `by-id` path and
you can skip re-calibrating. Any other GELLO unit needs its own entry.)

**2. Network.** The UR controller must be reachable from the machine running
these scripts (same LAN/subnet), and the UR must be in **Remote Control**
mode.

## Usage

**1. Alignment check (no motion) — run after calibrating, before any motion:**
```bash
python check_alignment.py
```
This only cross-checks the GELLO reading against whatever offsets are
currently in `PORT_CONFIG_MAP` — it verifies the calibration from step 1
above, it doesn't produce it. Move the GELLO leader through its range and
confirm the printed joint differences look sane (no ~180°/360° offset, no
axis with a flipped sign).

**2. Teleoperation:**
```bash
python ur10_tele.py \
    --ur-ip <UR controller IP> \
    --urdf urdf/ur5.urdf
```
You'll be prompted to type `START` before motion is enabled — make sure the
workspace is clear and the E-stop is within reach first.

Key flags (see `python ur10_tele.py --help` for the full list):
- `--urdf` / `--ee-frame` — robot URDF and end-effector frame used to compute
  the Jacobian/TCP twist.
- `--joint-reorder` / `--joint-sign` / `--joint-bias` — optional leader→robot
  joint remapping, in case GELLO and UR joint order/direction don't match.
- `--max-dq` — per-joint velocity clamp (rad/s), conservative by default.
- `--rate`, `--lookahead`, `--gain`, `--blend-seconds` — `servoJ` timing
  parameters; start conservative on a new setup.

Stop with `Ctrl-C` — the script calls `servoStop()`/`stopScript()` on exit.

## Known limitations

- GELLO's 7th value (gripper/trigger) is read but discarded — no gripper
  teleop yet, joint teleop only.
- Single-arm only (no bimanual support carried over from upstream).
- No camera integration — if you need synchronized vision data for data
  collection, that's not wired up here.
