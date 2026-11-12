

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


# --------------------------------------------------------------------------- GELLO
@dataclass
class GelloConfig:
    """robot_q[i] = joint_signs[i] * (servo[joint_ids[i]] - joint_offsets[i])

    joint_ids order is the robot joint order (reorder by listing servo ids differently).
    joint_offsets are multiples of pi/2, from: python gello/calibrate.py
    """

    serial: str
    """USB serial of this GELLO's U2D2 (e.g. "FT8J0WR4"); used to find and auto-detect it."""
    joint_ids: Sequence[int]
    joint_offsets: Sequence[float]
    joint_signs: Sequence[int]
    gripper: Optional[Tuple[int, float, float]] = None
    """(servo_id, open_deg, closed_deg), or None."""
    baudrate: int = 57600

    def __post_init__(self):
        n = len(self.joint_ids)
        assert len(self.joint_offsets) == n and len(self.joint_signs) == n, (
            "joint_ids, joint_offsets and joint_signs must have the same length"
        )
        assert all(abs(s) == 1 for s in self.joint_signs), "joint_signs must be +/-1"
        ids = list(self.joint_ids) + ([self.gripper[0]] if self.gripper else [])
        dup = sorted({i for i in ids if ids.count(i) > 1})
        assert not dup, f"servo ID(s) {dup} used more than once in joint_ids/gripper (GELLO {self.serial})"


GELLO_CONFIG_MAP: Dict[str, GelloConfig] = {
    # 6-DoF GELLO for UR5/UR10
    "ur": GelloConfig(
        serial="FT8J0WR4",
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            2 * np.pi / 2,
            4 * np.pi / 2,
            3 * np.pi / 2,
            5 * np.pi / 2,
            4 * np.pi / 2,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper=(7, 199.2, 148.2),
    ),
    # 4-DoF GELLO for Dobot MG400: servos 1, 2, 3, 5 -> J1..J4; servo 4 is skipped (no MG400 joint), 6 = gripper
    "mg400": GelloConfig(
        serial="FTAAMOOQ",
        joint_ids=(1, 2, 3, 5),
        joint_offsets=(
            1 * np.pi / 2,
            -1 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
        ),     # placeholder: run python gello/calibrate.py
        joint_signs=(1, -1, -1, -1),
        gripper=(6, 198.5, 157.3),      # placeholder: run python gello/calibrate.py
    ),
}

# --------------------------------------------------------------------------- Teleop
# Pairs each robot (key, same as ROBOT_CONFIG_MAP) with the GELLO that drives it (key in GELLO_CONFIG_MAP).
@dataclass
class TeleopConfig:
    leader: str
    rate_hz: float = 100.0
    use_gripper: bool = False


TELEOP_CONFIG_MAP: Dict[str, TeleopConfig] = {
    "ur10": TeleopConfig(leader="ur"),
    "ur5": TeleopConfig(leader="ur"),
    "mg400": TeleopConfig(leader="mg400"),
}


# --------------------------------------------------------------------------- Robot
@dataclass
class RobotConfig:
    dof: int
    controller: str
    controller_kwargs: Dict[str, Any] = field(default_factory=dict)
    stream_method: str = "servo_joints"
    gripper_method: Optional[str] = None
    tcp: Optional[Sequence[float]] = None
    home_deg: Sequence[float] = ()

    joint_lower_deg: Optional[Sequence[float]] = None
    joint_upper_deg: Optional[Sequence[float]] = None
    """Software joint limits; None = no clipping (the robot controller enforces its own)."""

    max_dq_deg_s: Sequence[float] = ()

    blend_seconds: float = 4.0

    def __post_init__(self):
        for name in ("home_deg", "max_dq_deg_s"):
            assert len(getattr(self, name)) == self.dof, f"{name} must have {self.dof} values"
        assert (self.joint_lower_deg is None) == (self.joint_upper_deg is None), (
            "set both joint_lower_deg and joint_upper_deg, or neither"
        )
        if self.joint_lower_deg is not None:
            for name in ("joint_lower_deg", "joint_upper_deg"):
                assert len(getattr(self, name)) == self.dof, f"{name} must have {self.dof} values"

    @property
    def home(self) -> np.ndarray:
        return np.deg2rad(self.home_deg)


_UR_LIMITS = dict(
    joint_lower_deg=(-360,) * 6,
    joint_upper_deg=(360,) * 6,
    max_dq_deg_s=(23, 23, 23, 40, 40, 52),  # 0.4/0.4/0.4/0.7/0.7/0.9 rad/s, conservative
    home_deg=(0, -90, 90, -90, -90, 0),
)

ROBOT_CONFIG_MAP: Dict[str, RobotConfig] = {
    "ur10": RobotConfig(
        dof=6,
        controller="ur",
        controller_kwargs={"ip": "192.168.1.141"},
        tcp=(0, 0, 0, 0, 0, 0),
        **_UR_LIMITS,
    ),
    "ur5": RobotConfig(
        dof=6,
        controller="ur",
        controller_kwargs={"ip": "192.168.1.141"},
        tcp=(0, 0, 0, 0, 0, 0),
        **_UR_LIMITS,
    ),

    "mg400": RobotConfig(
        dof=4,
        controller="mg400",
        # MG400 has no ServoJ: servo_joints streams short JointMovJ segments (SpeedJ/AccJ/CP in %,
        # scaled by the global speed_factor), one every servo_period s, blended by CP.
        # Overall speed is still capped by max_dq_deg_s below.
        controller_kwargs={
            "ip": "192.168.1.6",
            "speed_factor": 50,      # global SpeedFactor %, was 7 on the pendant
            "servo_speed": 100,
            "servo_acc": 100,
            "servo_cp": 100,
            "servo_period": 0.05,    # 20 segments/s, ~1.5 deg each at 30 deg/s
            "servo_max_lag": 10.0,   # pause adding segments if the arm falls >10 deg behind
        },
        stream_method="servo_joints",
        tcp=None,                       
        home_deg=(0, 0, 0, 0),
        # no joint_lower_deg / joint_upper_deg: the MG400 controller enforces its own joint limits
        max_dq_deg_s=(30, 30, 30, 60),
    ),
}


def get(table: Dict[str, Any], key: str, what: str):
    if key not in table:
        raise KeyError(f"Unknown {what} '{key}'. Available: {sorted(table)}")
    return table[key]
