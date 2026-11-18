"""Obstacle detection from the front LiDAR (SCRUM-33) with noise filtering,
ego-lane gating and range measurement support (SCRUM-34).

Input: LiDAR points already in the VEHICLE frame (x fwd, y right, z up) and
the current LaneEstimate. Steps:
  1. range gate (min/max range, only points ahead of the bumper),
  2. fit the local road surface z = a + b*x (handles slopes) and keep points
     between min_height and max_height above it  -> removes road + overhangs,
  3. keep points inside a corridor around the lane-centre path y(x) from the
     lane estimate (shifted sideways for the "is the left lane free?" check)
     -> ignores parked cars, walls and objects in other lanes,
  4. cluster along x and take the nearest cluster with >= min_points points
     -> removes isolated noise points / rain / single stray returns,
  5. debounce over the last frames (needs debounce_on hits to appear,
     debounce_off misses to disappear) -> no flicker,
  6. distance = nearest point of that cluster minus the front-bumper x.
"""
from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np

from lanebot.interfaces import LaneEstimate, ObstacleInfo


def fit_ground(points: np.ndarray, iters: int = 3, band: float = 0.6, keep: float = 0.15):
    """Robust line fit z = a + b*x of the road surface. Returns (a, b)."""
    if len(points) < 10:
        return 0.0, 0.0
    x, z = points[:, 0], points[:, 2]
    sel = np.abs(z) < band                     # road near the vehicle's own ground level
    a, b = 0.0, 0.0
    for _ in range(iters):
        if sel.sum() < 10:
            break
        A = np.stack([np.ones(sel.sum()), x[sel]], axis=1)
        a, b = np.linalg.lstsq(A, z[sel], rcond=None)[0]
        b = float(np.clip(b, -0.12, 0.12))     # at most ~7 % grade
        resid = z - (a + b * x)
        sel = np.abs(resid) < max(keep, band * 0.5)
        band *= 0.5
    return float(a), float(b)


class LidarObstacleDetector:
    def __init__(self, cfg: dict, bumper_x_m: float):
        self.cfg = cfg
        self.bumper_x = float(bumper_x_m)
        n = int(cfg["window"])
        self.hist = {}                          # per lateral shift: deque of raw detections
        self.state = {}                         # per lateral shift: debounced detected flag
        self.misses = {}
        self.dist_hist = {}
        self._n = n

    def detect(self, points: Optional[np.ndarray], lane: Optional[LaneEstimate],
               lateral_shift_m: float = 0.0, want_debug: bool = False) -> ObstacleInfo:
        key = round(lateral_shift_m, 2)
        hist = self.hist.setdefault(key, deque(maxlen=self._n))
        dists = self.dist_hist.setdefault(key, deque(maxlen=3))

        raw = self._detect_raw(points, lane, lateral_shift_m, want_debug)
        hist.append(raw.raw_detected)
        if raw.raw_detected:
            dists.append(raw.distance_m)
            self.misses[key] = 0
        else:
            self.misses[key] = self.misses.get(key, 0) + 1

        on = self.state.get(key, False)
        if not on and sum(hist) >= self.cfg["debounce_on"]:
            on = True
        elif on and self.misses[key] >= self.cfg["debounce_off"]:
            on = False
            dists.clear()
        self.state[key] = on

        raw.confidence = sum(hist) / max(len(hist), 1)
        raw.detected = on
        if on and dists:
            raw.distance_m = float(np.median(dists)) if raw.raw_detected else float(dists[-1])
        elif not on:
            raw.distance_m = float("inf")
        return raw

    def _detect_raw(self, points, lane, shift, want_debug) -> ObstacleInfo:
        c = self.cfg
        info = ObstacleInfo(detected=False)
        if points is None or len(points) == 0:
            return info
        pts = points[np.isfinite(points).all(axis=1)]
        rng = np.hypot(pts[:, 0], pts[:, 1])
        keep = (rng >= c["min_range_m"]) & (rng <= c["max_range_m"]) & (pts[:, 0] > 0.0)
        pts = pts[keep]
        if len(pts) == 0:
            return info

        a, b = fit_ground(pts)          # fit on everything ahead (mostly road)
        pts = pts[pts[:, 0] > self.bumper_x]
        h = pts[:, 2] - (a + b * pts[:, 0])
        pts = pts[(h >= c["min_height_m"]) & (h <= c["max_height_m"])]

        if lane is not None and lane.valid:
            y_c = lane.centre_y(pts[:, 0]) + shift
        else:
            y_c = np.full(len(pts), shift)
        lat = pts[:, 1] - y_c
        in_corr = np.abs(lat) <= c["corridor_half_width_m"]
        cand, cand_lat = pts[in_corr], lat[in_corr]
        self.last_debug = {"corridor_points": cand, "ground": (a, b)} if want_debug else None
        if len(cand) < c["min_points"]:
            return info

        order = np.argsort(cand[:, 0])
        xs = cand[order, 0]
        L = c["cluster_length_m"]
        j = 0
        for i in range(len(xs)):                       # nearest window of length L with enough points
            while xs[i] - xs[j] > L:
                j += 1
            if i - j + 1 >= c["min_points"]:
                sel = order[j:i + 1]
                info.raw_detected = True
                info.distance_m = float(max(0.0, xs[j] - self.bumper_x))
                info.lateral_m = float(np.median(cand_lat[sel]))
                info.num_points = int(i - j + 1)
                break
        return info
