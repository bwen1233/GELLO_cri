# GELLO_cri

Teleoperate robot arms driven through CRI (Common Robot Interface) with a GELLO leader arm. Currently supports `ur10`, `ur5` and `mg400`.

## Install

```bash
git clone --recurse-submodules https://github.com/bwen1233/GELLO_cri.git
cd GELLO_cri
# already cloned without submodules: git submodule update --init

conda create -n gello_cri python=3.11
conda activate gello_cri
pip install -r requirements.txt
```

CRI is the git submodule at `robot/common_robot_interface` and is used directly; no separate install is needed.

## Configure

All configuration lives in [config.py](config.py):

| Map | Purpose |
| --- | --- |
| `GELLO_CONFIG_MAP` | Per GELLO: U2D2 serial, servo IDs, joint signs/offsets, gripper |
| `ROBOT_CONFIG_MAP` | Per robot type: CRI controller, IP, joint limits, speed limits, home pose |
| `TELEOP_CONFIG_MAP` | Which GELLO drives each robot type |

Check the robot IP (e.g. `ip` in `ROBOT_CONFIG_MAP["ur10"]`) before use.

The connected GELLO is detected automatically from its U2D2 USB serial. With several plugged in, choose one with `--leader <name>`, or set the serial port directly with `--port COM5` / `--leader-port COM5`.

## Usage


### 1. Calibrate the GELLO (once per GELLO, only the GELLO connected)

Check the reference pose (degrees) in `CALIB_POSE_MAP` at the top of [gello/calibrate.py](gello/calibrate.py), then run:

```bash
python gello/calibrate.py
```

Press SPACE three times when prompted: GELLO in the reference pose, gripper fully open, gripper fully closed. It prints `joint_offsets`, `joint_signs` and `gripper`; press `y` to write them into `config.py`.

### 3. Test each side on its own(optional)

```bash
python gello/gello.py                   # read the GELLO, print joints
python robot/robot.py --robot ur10      # connect to the robot, print joints (no motion)
```

### 4. Safety check (no motion; always run before teleop)

```bash
python check_alignment.py --robot ur10
```

Shows the per-joint difference between the GELLO and the robot; joints above `--max-diff` (default 45°) are flagged FAR. Bring the GELLO close to the robot's current pose before starting teleop.

### 5. Teleoperate

```bash
python run.py --robot ur10 --print
```

- Motion starts immediately, easing in from the robot's current pose over `blend_seconds`
- Joint speed is limited by `max_dq_deg_s` and commands are clipped to the joint limits
- `Ctrl-C` stops; `--seconds N` stops after N seconds; `--print` prints the leader joints

## Adding devices

- **New GELLO:** add an entry to `GELLO_CONFIG_MAP` with `serial` set to its U2D2 USB serial (printed by `python gello/gello.py` when nothing matches), add its reference pose to `CALIB_POSE_MAP`, then calibrate.
- **New robot:** add the CRI controller and its arguments to `ROBOT_CONFIG_MAP`, and set the GELLO that drives it in `TELEOP_CONFIG_MAP`.
- **Gripper:** implement it in CRI, then set `gripper_method` in `ROBOT_CONFIG_MAP` and `use_gripper=True` in `TELEOP_CONFIG_MAP`.
