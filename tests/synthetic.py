"""Synthetic camera images and LiDAR clouds for tests (no CARLA needed).

The camera renderer ray-casts every pixel onto a flat road with painted
lines, using the same pinhole model as CARLA, so the lane detector sees
images that are geometrically identical to the simulator's.
"""
from __future__ import annotations

import math

import numpy as np

from lanebot.perception.camera_geometry import CameraModel


class RoadRenderer:
    def __init__(self, cam: CameraModel, seed: int = 0):
        self.cam = cam
        self.rng = np.random.default_rng(seed)
        f = cam.focal_px
        u, v = np.meshgrid(np.arange(cam.width) + 0.5, np.arange(cam.height) + 0.5)
        r = (u - cam.width / 2) / f
        up = (cam.height / 2 - v) / f
        phi = math.radians(-cam.pitch_deg)
        dx = math.cos(phi) + up * math.sin(phi)
        dz = -math.sin(phi) + up * math.cos(phi)
        self.ground = dz < -1e-3
        t = np.where(self.ground, cam.height_m / np.maximum(-dz, 1e-6), np.nan)
        self.X = t * dx                    # camera-ground frame, forward
        self.Y = t * r                     # right
        self.noise = self.rng.normal(0, 6, size=(cam.height, cam.width)).astype(np.float32)

    def render(self, line_fn, dashed_fn=None, line_width=0.15, far=80.0) -> np.ndarray:
        """line_fn(X, Y) -> list of lateral distances to each painted line (metres)."""
        img = np.empty((self.cam.height, self.cam.width, 3), np.uint8)
        img[:] = (235, 206, 135)                         # sky (BGR)
        g = self.ground & (self.X < far)
        X, Y = self.X[g], self.Y[g]
        grey = 85 + self.noise[g]
        paint = np.zeros(X.shape, bool)
        for k, d in enumerate(line_fn(X, Y)):
            on = np.abs(d) < line_width / 2
            if dashed_fn is not None:
                on &= dashed_fn(k, X)
            paint |= on
        val = np.where(paint, 225.0, grey)
        val = np.clip(val, 0, 255).astype(np.uint8)
        img[g] = np.stack([val, val, val], axis=1)
        img[self.ground & (self.X >= far)] = (90, 90, 90)
        return img


def lane_lines(c0, c1, c2, width=3.5, extra_lanes=True):
    """Lines of the ego lane whose centre is y = c0 + c1 X + c2 X^2 (camera-ground frame)."""
    offs = [-width / 2, width / 2] + ([-1.5 * width, 1.5 * width] if extra_lanes else [])

    def fn(X, Y):
        yc = c0 + c1 * X + c2 * X * X
        return [Y - (yc + o) for o in offs]
    return fn


def lidar_cloud(obstacle_x=None, obstacle_y=0.0, obstacle_w=1.8, obstacle_h=1.5, ground_slope=0.0,
                noise_points=0, side_objects=(), seed=0, mount_z=0.9):
    """Point cloud in the VEHICLE frame: road surface + optional box obstacle + clutter."""
    rng = np.random.default_rng(seed)
    xs, ys = np.meshgrid(np.arange(3, 55, 0.4), np.arange(-12, 12, 0.3))
    ground = np.stack([xs.ravel(), ys.ravel(), ground_slope * xs.ravel() + rng.normal(0, 0.02, xs.size)], 1)
    pts = [ground]

    def box(x0, y0, w, h, depth=0.3):
        yy, zz = np.meshgrid(np.arange(y0 - w / 2, y0 + w / 2, 0.1), np.arange(0.05, h, 0.1))
        face = np.stack([np.full(yy.size, x0) + rng.uniform(0, depth, yy.size), yy.ravel(),
                         zz.ravel() + ground_slope * x0], 1)
        return face

    if obstacle_x is not None:
        g = pts[0]
        pts[0] = g[~((g[:, 0] > obstacle_x) & (np.abs(g[:, 1] - obstacle_y) < obstacle_w / 2 + 0.5))]  # shadow
        pts.append(box(obstacle_x, obstacle_y, obstacle_w, obstacle_h))
    for (x0, y0) in side_objects:
        pts.append(box(x0, y0, 1.8, 1.5))
    if noise_points:
        pts.append(np.stack([rng.uniform(3, 50, noise_points), rng.uniform(-1, 1, noise_points),
                             rng.uniform(0.3, 2.0, noise_points)], 1))
    return np.vstack(pts).astype(np.float32)
