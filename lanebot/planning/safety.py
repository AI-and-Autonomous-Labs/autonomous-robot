"""Stopping distance and braking thresholds (SCRUM-35).

    stopping distance  d_stop(v, a) = v * t_react + v^2 / (2 a)

Thresholds used by the state machine:
    brake trigger    : obstacle closer than d_stop(v, a_comfort) + stop_margin
                       -> start a comfortable stop that ends stop_margin before it
    emergency brake  : obstacle closer than d_stop(v, a_max) + min_gap
                       -> full brake
    approach speed   : v_allowed(d) = sqrt(2 a_comfort (d - stop_margin))
                       (the speed from which a comfortable stop still fits)
"""
from __future__ import annotations

import math


class SafetyModel:
    def __init__(self, cfg: dict):
        self.t_react = cfg["reaction_time_s"]
        self.a_comf = cfg["comfort_decel_mps2"]
        self.a_max = cfg["max_decel_mps2"]
        self.stop_margin = cfg["stop_margin_m"]
        self.min_gap = cfg["min_gap_m"]

    def stopping_distance(self, v: float, decel: float) -> float:
        return v * self.t_react + v * v / (2.0 * decel)

    def brake_trigger_distance(self, v: float) -> float:
        return self.stopping_distance(v, self.a_comf) + self.stop_margin

    def emergency_distance(self, v: float) -> float:
        return self.stopping_distance(v, self.a_max) + self.min_gap

    def approach_speed(self, distance: float) -> float:
        """Max speed that still allows a comfortable stop stop_margin before the obstacle."""
        d = distance - self.stop_margin - 0.0
        if d <= 0:
            return 0.0
        # solve v*t + v^2/(2a) = d for v
        a, t = self.a_comf, self.t_react
        return max(0.0, -a * t + math.sqrt((a * t) ** 2 + 2 * a * d))

    def table(self, speeds_kmh=(10, 20, 30, 40, 50)):
        """Rows of (km/h, comfort stop m, trigger m, emergency m) for the docs / test plan."""
        rows = []
        for kmh in speeds_kmh:
            v = kmh / 3.6
            rows.append((kmh, round(self.stopping_distance(v, self.a_comf), 1),
                         round(self.brake_trigger_distance(v), 1), round(self.emergency_distance(v), 1)))
        return rows
