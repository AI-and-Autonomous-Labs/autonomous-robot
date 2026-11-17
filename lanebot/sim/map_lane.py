"""Lane geometry from the CARLA map (ground truth).

Used for:
  * logging the true lane error next to the vision estimate (SCRUM-32/40),
  * `lane_detection.source: map` while the vision detector is being built,
  * the avoidance manoeuvre, which tracks the ORIGINAL lane via the map
    while the car drives in the neighbouring lane (SCRUM-37),
  * labels for recorded frames (SCRUM-27).

The output is the same LaneEstimate as the vision detector produces, so the
controller cannot tell the difference.
"""
from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from lanebot.interfaces import LaneEstimate


def world_to_vehicle(px, py, vx, vy, yaw_deg):
    """World (x, y) points -> vehicle frame (x fwd, y right). CARLA is left-handed (yaw turns x towards y)."""
    yaw = math.radians(yaw_deg)
    dx, dy = np.asarray(px) - vx, np.asarray(py) - vy
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def fit_lane(xs: np.ndarray, ys: np.ndarray, width: float, source: str = "map") -> LaneEstimate:
    if len(xs) < 3:
        return LaneEstimate.invalid(source)
    coeffs = np.polyfit(xs, ys, 2)[::-1]
    return LaneEstimate.from_coeffs(coeffs, width, 1.0, source)


def next_straightest(wp, step: float):
    """wp.next() can branch at junctions; keep the branch with the smallest heading change."""
    nxt = wp.next(step)
    if not nxt:
        return None
    yaw = wp.transform.rotation.yaw
    return min(nxt, key=lambda w: abs((w.transform.rotation.yaw - yaw + 180) % 360 - 180))


class MapLaneReference:
    def __init__(self, carla_map, lookahead_m: float = 30.0, behind_m: float = 5.0, step_m: float = 2.0):
        import carla                                       # imported here so pure modules never need CARLA
        self._carla = carla
        self.map = carla_map
        self.lookahead = lookahead_m
        self.behind = behind_m
        self.step = step_m
        self.locked_lane_id: Optional[int] = None
        self.locked_road_id: Optional[int] = None

    # ------------------------------------------------------------ waypoints
    def waypoint(self, location):
        return self.map.get_waypoint(location, project_to_road=True, lane_type=self._carla.LaneType.Driving)

    def lock_current_lane(self, location):
        wp = self.waypoint(location)
        self.locked_lane_id, self.locked_road_id = wp.lane_id, wp.road_id

    def unlock(self):
        self.locked_lane_id = self.locked_road_id = None

    def _locked_waypoint(self, location):
        wp = self.waypoint(location)
        if self.locked_lane_id is None or wp.lane_id == self.locked_lane_id or wp.road_id != self.locked_road_id:
            return wp
        for getter in ("get_right_lane", "get_left_lane"):
            cand = wp
            for _ in range(3):
                cand = getattr(cand, getter)()
                if cand is None:
                    break
                if cand.lane_id == self.locked_lane_id:
                    return cand
        return wp

    def centre_points(self, wp) -> List:
        pts = [wp]
        back = wp
        for _ in range(int(self.behind / self.step)):
            prev = back.previous(self.step)
            if not prev:
                break
            back = prev[0]
            pts.insert(0, back)
        cur = wp
        for _ in range(int(self.lookahead / self.step)):
            cur = next_straightest(cur, self.step)
            if cur is None:
                break
            pts.append(cur)
        return pts

    # ------------------------------------------------------------- estimate
    def estimate(self, vehicle_transform, use_locked: bool = False) -> LaneEstimate:
        loc = vehicle_transform.location
        wp = self._locked_waypoint(loc) if use_locked else self.waypoint(loc)
        if wp is None:
            return LaneEstimate.invalid("map")
        pts = self.centre_points(wp)
        px = np.array([p.transform.location.x for p in pts])
        py = np.array([p.transform.location.y for p in pts])
        xs, ys = world_to_vehicle(px, py, loc.x, loc.y, vehicle_transform.rotation.yaw)
        est = fit_lane(xs, ys, wp.lane_width, "map")
        est.debug["waypoint"] = wp
        return est

    def left_lane_available(self, location) -> bool:
        """Same-direction driving lane to the left, and no junction coming up soon."""
        wp = self.waypoint(location)
        left = wp.get_left_lane()
        if left is None or left.lane_type != self._carla.LaneType.Driving:
            return False
        if left.lane_id * wp.lane_id <= 0:            # opposite direction
            return False
        cur = wp
        for _ in range(int(60 / self.step)):
            cur = next_straightest(cur, self.step)
            if cur is None or cur.is_junction:
                return False
        return True
