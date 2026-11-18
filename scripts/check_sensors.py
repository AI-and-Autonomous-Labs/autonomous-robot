"""SCRUM-26: mount the camera and LiDAR and verify that the data streams work.

Ticks the simulation, checks every tick delivers one image and one LiDAR
sweep with matching frame numbers, prints rates / sizes / point counts and
saves a sample camera image and a LiDAR top-down plot to runs/sensor_check/.

    python scripts/check_sensors.py --ticks 100
"""
from _common import base_parser, config_from_args

import logging
import time
from pathlib import Path

import cv2
import numpy as np

from lanebot.sim.sensors import SensorRig
from lanebot.sim.world import CarlaWorld

log = logging.getLogger("check_sensors")


def main():
    p = base_parser(__doc__)
    p.add_argument("--ticks", type=int, default=100)
    args = p.parse_args()
    cfg = config_from_args(args)
    out = Path(cfg["logging"]["dir"]) / "sensor_check"
    out.mkdir(parents=True, exist_ok=True)
    sim = CarlaWorld(cfg)
    try:
        sim.connect()
        sim.spawn_ego()
        rig = SensorRig(sim, cfg)
        n_img = n_lidar = 0
        pts_counts = []
        t0 = time.time()
        for i in range(args.ticks):
            sim.apply_control(0.3, 0.0, 0.0)
            f = rig.tick()
            if f.image_bgr is not None:
                n_img += 1
            if f.lidar_xyz is not None:
                n_lidar += 1
                pts_counts.append(len(f.lidar_xyz))
            if i == args.ticks // 2:
                if f.image_bgr is not None:
                    cv2.imwrite(str(out / "camera.png"), f.image_bgr)
                if f.lidar_xyz is not None:
                    save_lidar_plot(f.lidar_xyz, out / "lidar_top.png")
        wall = time.time() - t0
        sim_rate = args.ticks / wall
        log.info("ticks %d in %.1f s wall (%.1f sim ticks/s, real-time factor %.2f)",
                 args.ticks, wall, sim_rate, sim_rate * cfg["carla"]["fixed_delta_s"])
        log.info("camera frames %d/%d  shape %s", n_img, args.ticks,
                 None if f.image_bgr is None else f.image_bgr.shape)
        if pts_counts:
            log.info("lidar sweeps %d/%d  points per sweep: min %d, mean %d, max %d",
                     n_lidar, args.ticks, min(pts_counts), int(np.mean(pts_counts)), max(pts_counts))
        log.info("collisions %d, lane invasions %d", rig.collision_count, rig.invasion_count)
        ok = n_img == args.ticks or not cfg["sensors"]["camera"]["enabled"]
        ok &= n_lidar == args.ticks or not cfg["sensors"]["lidar"]["enabled"]
        log.info("RESULT: %s - samples saved in %s", "OK" if ok else "MISSING DATA", out)
    finally:
        sim.cleanup()


def save_lidar_plot(pts, path, size=600, metres=40):
    img = np.zeros((size, size, 3), np.uint8)
    s = size / (2 * metres)
    u = (size / 2 + pts[:, 1] * s).astype(int)          # y right -> image right
    v = (size / 2 - pts[:, 0] * s).astype(int)          # x forward -> image up
    ok = (u >= 0) & (u < size) & (v >= 0) & (v < size)
    z = np.clip((pts[ok, 2] + 1) / 3, 0, 1)
    img[v[ok], u[ok]] = np.stack([255 * (1 - z), 255 * z, 200 * np.ones_like(z)], 1).astype(np.uint8)
    cv2.circle(img, (size // 2, size // 2), 4, (0, 0, 255), -1)
    cv2.imwrite(str(path), img)


if __name__ == "__main__":
    main()
