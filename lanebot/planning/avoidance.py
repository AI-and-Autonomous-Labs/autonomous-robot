"""Simple obstacle avoidance path (SCRUM-37, stretch goal).

The manoeuvre is a lateral-offset profile over distance travelled s, relative
to the ORIGINAL lane centre (negative = left):

    OUT   0 .. R                 offset smoothly 0 -> -W      (cosine ramp)
    PASS  R .. R + P             hold -W until the obstacle is passed
    BACK  R + P .. 2R + P        offset smoothly -W -> 0
    DONE

with W = lane width, R = ramp length, P = obstacle distance + obstacle
length + buffer - R (at least 0). The lateral controller tracks this offset
while the lane reference comes from the CARLA map (the camera would re-lock
onto the new lane during the move).
"""
from __future__ import annotations

import math


class AvoidanceManeuver:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.active = False
        self.s = 0.0
        self.width = 3.5
        self.ramp = cfg["ramp_length_m"]
        self.hold = 0.0

    def start(self, obstacle_distance_m: float, lane_width_m: float):
        self.active = True
        self.s = 0.0
        self.width = lane_width_m
        pass_end = obstacle_distance_m + self.cfg["obstacle_length_m"] + self.cfg["pass_buffer_m"]
        self.hold = max(0.0, pass_end - self.ramp)

    @property
    def total_length(self) -> float:
        return 2 * self.ramp + self.hold

    @property
    def done(self) -> bool:
        return self.active and self.s >= self.total_length

    @property
    def phase(self) -> str:
        if not self.active:
            return "IDLE"
        if self.s < self.ramp:
            return "OUT"
        if self.s < self.ramp + self.hold:
            return "PASS"
        if self.s < self.total_length:
            return "BACK"
        return "DONE"

    def update(self, distance_travelled_m: float) -> float:
        """Advance by the distance driven this tick and return the target offset."""
        if not self.active:
            return 0.0
        self.s += max(0.0, distance_travelled_m)
        return self.target_offset(self.s)

    def target_offset(self, s: float) -> float:
        W, R, H = self.width, self.ramp, self.hold
        if s <= 0:
            return 0.0
        if s < R:
            return -W * 0.5 * (1 - math.cos(math.pi * s / R))
        if s < R + H:
            return -W
        if s < 2 * R + H:
            return -W * 0.5 * (1 + math.cos(math.pi * (s - R - H) / R))
        return 0.0

    def derivatives(self, s: float):
        """(slope, curvature) of the offset profile at s, by finite differences."""
        h = 0.25
        a, b, c = self.target_offset(s - h), self.target_offset(s), self.target_offset(s + h)
        return (c - a) / (2 * h), (c - 2 * b + a) / (h * h)

    def stop(self):
        self.active = False
        self.s = 0.0
