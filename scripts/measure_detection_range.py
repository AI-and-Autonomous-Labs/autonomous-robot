"""SCRUM-34: measure the obstacle detection range and distance accuracy.

The ego car stands still; an obstacle is placed at increasing distances in
the ego lane (and optionally in the next lane, which must NOT be detected).
For each distance the detector runs for --frames ticks and the detection
rate and distance error are printed and saved to runs/detection_range.csv.

    python scripts/measure_detection_range.py
    python scripts/measure_detection_range.py --blueprint static.prop.trafficcone01 --max 40
    python scripts/measure_detection_range.py --lane left      # false-positive check
"""
from _common import base_parser, config_from_args

import csv
import logging
from pathlib import Path

import numpy as np

from lanebot.perception.obstacle_detector import LidarObstacleDetector
from lanebot.sim.map_lane import MapLaneReference
from lanebot.sim.obstacles import ObstacleSpawner, gap_to_obstacles
from lanebot.sim.sensors import SensorRig
from lanebot.sim.world import CarlaWorld

log = logging.getLogger("range")


def main():
    p = base_parser(__doc__)
    p.add_argument("--blueprint", default="vehicle.audi.a2")
    p.add_argument("--lane", default="ego", choices=["ego", "left", "right"])
    p.add_argument("--min", type=float, default=5)
    p.add_argument("--max", type=float, default=60)
    p.add_argument("--step", type=float, default=5)
    p.add_argument("--frames", type=int, default=20)
    args = p.parse_args()
    args.set.append("sensors.camera.enabled=false")
    cfg = config_from_args(args)
    sim = CarlaWorld(cfg)
    rows = []
    try:
        sim.connect()
        sim.spawn_ego()
        sim.apply_control(0, 0, 1.0, hand_brake=True)
        rig = SensorRig(sim, cfg)
        spawner = ObstacleSpawner(sim)
        ref = MapLaneReference(sim.map)
        for d in np.arange(args.min, args.max + 1e-6, args.step):
            spawner.spawn_all([{"blueprint": args.blueprint, "distance_m": float(d), "lane": args.lane}])
            det = LidarObstacleDetector(cfg["obstacle_detection"], sim.front_bumper_x())
            hits, dists = 0, []
            for _ in range(5):                       # let the obstacle settle
                rig.tick()
            gap = gap_to_obstacles(sim.ego, spawner.spawned) if args.lane == "ego" else float("nan")
            for _ in range(args.frames):
                f = rig.tick()
                lane = ref.estimate(sim.ego.get_transform())
                info = det.detect(f.lidar_xyz, lane)
                hits += info.raw_detected
                if info.raw_detected:
                    dists.append(info.distance_m)
            rate = hits / args.frames
            est = float(np.median(dists)) if dists else float("nan")
            rows.append([args.blueprint, args.lane, round(float(d), 1), round(gap, 2), round(rate, 2),
                         round(est, 2), round(est - gap, 2) if dists else ""])
            log.info("placed at %5.1f m  true gap %6.2f m  detected %3.0f %%  estimated %6.2f m",
                     d, gap, 100 * rate, est)
            spawner.destroy_all()
            rig.tick()
    finally:
        sim.cleanup()
    out = Path(cfg["logging"]["dir"]) / "detection_range.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    new = not out.exists()
    with open(out, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["blueprint", "lane", "placed_m", "true_gap_m", "detection_rate", "estimated_m", "error_m"])
        w.writerows(rows)
    ok = [r for r in rows if r[4] >= 0.9]
    if args.lane == "ego":
        print(f"reliable detection (>=90 %) up to {max((r[2] for r in ok), default=0)} m placement")
    else:
        print("false detections in the other lane:", sum(r[4] > 0 for r in rows), "of", len(rows), "distances")
    print("results appended to", out)


if __name__ == "__main__":
    main()
