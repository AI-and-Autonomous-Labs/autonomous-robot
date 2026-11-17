"""Shared data types passed between modules (SCRUM-38).

Every module talks to the others only through these classes, so each team
member can work on a module independently and it still plugs into the main
loop (lanebot/app.py).

Coordinate convention (same as CARLA / Unreal):
    vehicle frame: x forward, y RIGHT, z up, metres, origin at the vehicle's
    reference point (ground level, centre of the car).
    angles: radians unless the name says _deg; positive yaw/heading = turning right.

Sign conventions used everywhere:
    LaneEstimate.offset_m        > 0  -> vehicle is RIGHT of the lane centre
    LaneEstimate.heading_err_rad > 0  -> vehicle points RIGHT of the lane direction
    LaneEstimate.curvature_1pm   > 0  -> lane bends to the RIGHT ahead
    ControlCommand.steer         > 0  -> steer right (CARLA convention)
    PlannerOutput.target_offset_m < 0 -> drive left of the lane centre (avoidance)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Tuple

import numpy as np


class DriveState(str, Enum):
    """States of the behaviour state machine (see docs/architecture.md)."""
    FOLLOW_LANE = "FOLLOW_LANE"   # normal lane keeping at cruise speed
    BRAKING = "BRAKING"           # obstacle ahead inside braking distance: decelerate
    STOPPED = "STOPPED"           # standing still in front of an obstacle
    AVOIDING = "AVOIDING"         # (stretch) passing the obstacle in the left lane
    SAFE_STOP = "SAFE_STOP"       # lane lost: controlled stop until the lane is found again


@dataclass
class VehicleState:
    timestamp: float              # simulation time [s]
    speed_mps: float              # forward speed [m/s]
    x: float                      # world position [m]
    y: float
    z: float
    yaw_deg: float                # world yaw [deg]


@dataclass
class SensorFrame:
    """Everything the sensors produced for one simulation tick."""
    frame: int
    vehicle: VehicleState
    image_bgr: Optional[np.ndarray]          # HxWx3 uint8, front camera
    lidar_xyz: Optional[np.ndarray]          # Nx3 float32, VEHICLE frame
    collisions: int = 0                      # collision events since last frame
    lane_invasions: int = 0                  # lane-marking crossings since last frame


@dataclass
class LaneEstimate:
    """Ego-lane geometry in the vehicle frame.

    The lane centre line is y(x) = c0 + c1*x + c2*x^2 (vehicle frame), so
    offset = -c0, heading error = -atan(c1), curvature = 2*c2/(1+c1^2)^1.5.
    Vision and map-based estimators both fill this in the same way.
    """
    valid: bool
    offset_m: float = 0.0
    heading_err_rad: float = 0.0
    curvature_1pm: float = 0.0
    lane_width_m: float = 3.5
    confidence: float = 0.0                  # 0..1
    coeffs: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    source: str = "none"                     # "vision" | "map" | "none"
    debug: Dict[str, Any] = field(default_factory=dict, repr=False)

    def centre_y(self, x: np.ndarray | float) -> np.ndarray | float:
        c0, c1, c2 = self.coeffs
        return c0 + c1 * x + c2 * np.square(x)

    @staticmethod
    def from_coeffs(coeffs, lane_width_m: float, confidence: float, source: str,
                    debug: Optional[dict] = None) -> "LaneEstimate":
        c0, c1, c2 = (float(c) for c in coeffs)
        return LaneEstimate(
            valid=True,
            offset_m=-c0,
            heading_err_rad=-float(np.arctan(c1)),
            curvature_1pm=2.0 * c2 / (1.0 + c1 * c1) ** 1.5,
            lane_width_m=float(lane_width_m),
            confidence=float(confidence),
            coeffs=(c0, c1, c2),
            source=source,
            debug=debug or {},
        )

    @staticmethod
    def invalid(source: str = "none") -> "LaneEstimate":
        return LaneEstimate(valid=False, source=source)


@dataclass
class ObstacleInfo:
    detected: bool
    distance_m: float = float("inf")         # front bumper -> nearest obstacle point
    lateral_m: float = 0.0                   # obstacle y relative to the corridor centre
    num_points: int = 0
    confidence: float = 0.0                  # fraction of recent frames with a detection
    raw_detected: bool = False               # this frame only, before debouncing


@dataclass
class PlannerOutput:
    state: DriveState
    speed_cap_mps: float                     # max speed allowed by the behaviour
    target_offset_m: float = 0.0             # lateral target relative to the lane centre
    target_slope: float = 0.0                # d(target_offset)/ds  (preview for the lateral controller)
    target_curvature: float = 0.0            # d2(target_offset)/ds2 (feed-forward)
    use_map_lane: bool = False               # True while avoiding (track the original lane via the map)
    emergency_brake: bool = False
    reason: str = ""


@dataclass
class ControlCommand:
    throttle: float = 0.0                    # 0..1
    steer: float = 0.0                       # -1..1
    brake: float = 0.0                       # 0..1
    hand_brake: bool = False
