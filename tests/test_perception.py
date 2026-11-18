"""Lane detector and obstacle detector on synthetic data (SCRUM-28/29/33/34)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from lanebot.config import load_config  # noqa: E402
from lanebot.interfaces import LaneEstimate  # noqa: E402
from lanebot.perception.lane_detector import VisionLaneDetector  # noqa: E402
from lanebot.perception.obstacle_detector import LidarObstacleDetector  # noqa: E402
from synthetic import RoadRenderer, lane_lines, lidar_cloud  # noqa: E402

CFG = load_config()
CAM = CFG["sensors"]["camera"]


def make_detector():
    return VisionLaneDetector(CFG["lane_detection"], CAM)


def scene(offset, heading_deg, curvature, dashed=False, drop=None, seed=0):
    """Camera image for a vehicle `offset` m right of centre, pointing heading_deg right of the lane."""
    det = make_detector()
    r = RoadRenderer(det.cam, seed)
    # lane centre in the camera-ground frame (camera is cam_x ahead of the vehicle origin)
    h = -math.radians(heading_deg)          # lane direction relative to the car
    cx = det.cam_x
    # vehicle frame: y_c(x) = -offset + tan(h) x + (k/2) x^2  -> shift to camera frame x = X + cx
    a0, a1, a2 = -offset, math.tan(h), curvature / 2
    c0 = a0 + a1 * cx + a2 * cx * cx
    c1 = a1 + 2 * a2 * cx
    fn = lane_lines(c0, c1, a2)
    dash = None
    if dashed:
        dash = lambda k, X: (k != 0) | ((X % 12) < 6)          # left line dashed
    if drop is not None:
        base = fn
        fn = lambda X, Y: [d if k != drop else np.full_like(d, 99.0) for k, d in enumerate(base(X, Y))]
    return det, r.render(fn, dash)


class TestLaneDetector(unittest.TestCase):
    def check(self, offset, heading, curv, tol_off=0.12, tol_head=1.2, **kw):
        det, img = scene(offset, heading, curv, **kw)
        for _ in range(3):                          # a few frames: smoothing converges
            est = det.process(img)
        self.assertTrue(est.valid, "lane not found")
        self.assertAlmostEqual(est.offset_m, offset, delta=tol_off)
        self.assertAlmostEqual(math.degrees(est.heading_err_rad), heading, delta=tol_head)
        return est

    def test_centred_straight(self):
        est = self.check(0.0, 0.0, 0.0)
        self.assertGreater(est.confidence, 0.9)
        self.assertAlmostEqual(est.lane_width_m, 3.5, delta=0.2)

    def test_offset_right(self):
        self.check(0.6, 0.0, 0.0)

    def test_offset_left_and_heading(self):
        self.check(-0.5, 3.0, 0.0)

    def test_heading_left(self):
        self.check(0.2, -4.0, 0.0)

    def test_curve_right(self):
        est = self.check(0.0, 0.0, 1 / 150.0, tol_off=0.15, tol_head=1.5)
        self.assertGreater(est.curvature_1pm, 0.002)

    def test_curve_left(self):
        est = self.check(0.3, 0.0, -1 / 120.0, tol_off=0.15, tol_head=1.5)
        self.assertLess(est.curvature_1pm, -0.002)

    def test_dashed_line(self):
        self.check(0.3, 1.0, 0.0, dashed=True)

    def test_one_line_missing(self):
        est = self.check(0.4, 0.0, 0.0, drop=1, tol_off=0.2)
        self.assertLess(est.confidence, 1.0)

    def test_no_lines_is_invalid_after_hold(self):
        det = make_detector()
        r = RoadRenderer(det.cam)
        img = r.render(lambda X, Y: [])
        est = None
        for _ in range(CFG["lane_detection"]["hold_frames"] + 2):
            est = det.process(img)
        self.assertFalse(est.valid)


class TestObstacleDetector(unittest.TestCase):
    def setUp(self):
        self.lane = LaneEstimate.from_coeffs((0, 0, 0), 3.5, 1.0, "test")

    def run_frames(self, det, cloud, lane=None, n=4, **kw):
        info = None
        for _ in range(n):
            info = det.detect(cloud, lane or self.lane, **kw)
        return info

    def test_obstacle_in_lane(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        info = self.run_frames(det, lidar_cloud(obstacle_x=30.0))
        self.assertTrue(info.detected)
        self.assertAlmostEqual(info.distance_m, 30.0 - 2.4, delta=0.3)

    def test_empty_road_and_slope(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        info = self.run_frames(det, lidar_cloud(ground_slope=0.05), n=6)
        self.assertFalse(info.detected)

    def test_obstacle_on_slope(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        info = self.run_frames(det, lidar_cloud(obstacle_x=25, ground_slope=0.04))
        self.assertTrue(info.detected)
        self.assertAlmostEqual(info.distance_m, 22.6, delta=0.4)

    def test_ignores_other_lanes_and_noise(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        cloud = lidar_cloud(side_objects=[(20, -3.5), (15, 3.6), (35, -7)], noise_points=4)
        info = self.run_frames(det, cloud, n=6)
        self.assertFalse(info.detected)

    def test_follows_curved_corridor(self):
        # lane bends right (y = 0.004 x^2): a car at x=30, y=3.6 is IN the lane, one at y=0 is not
        curved = LaneEstimate.from_coeffs((0, 0, 0.004), 3.5, 1.0, "test")
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        self.assertTrue(self.run_frames(det, lidar_cloud(obstacle_x=30, obstacle_y=3.6), lane=curved).detected)
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        self.assertFalse(self.run_frames(det, lidar_cloud(obstacle_x=30, obstacle_y=-0.6), lane=curved, n=6).detected)

    def test_left_lane_check_with_shift(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        info = self.run_frames(det, lidar_cloud(obstacle_x=20, obstacle_y=-3.5), lateral_shift_m=-3.5)
        self.assertTrue(info.detected)

    def test_debounce(self):
        det = LidarObstacleDetector(CFG["obstacle_detection"], bumper_x_m=2.4)
        with_obs, without = lidar_cloud(obstacle_x=20), lidar_cloud()
        self.assertFalse(det.detect(with_obs, self.lane).detected)       # 1 hit: not yet
        self.assertTrue(det.detect(with_obs, self.lane).detected)        # 2 hits: on
        self.assertTrue(det.detect(without, self.lane).detected)         # brief dropout ignored
        for _ in range(CFG["obstacle_detection"]["debounce_off"]):
            info = det.detect(without, self.lane)
        self.assertFalse(info.detected)


if __name__ == "__main__":
    unittest.main()
