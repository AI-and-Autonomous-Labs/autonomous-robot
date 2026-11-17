"""CSV logging of states, distances and lane error (SCRUM-40).

One row per tick in runs/<run_name>/log.csv plus state transitions in
transitions.csv and the config used in config.yaml, so every run can be
analysed later (scripts/plot_run.py).
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path
from typing import Optional

import yaml

COLUMNS = [
    "t", "frame", "state", "speed_mps", "target_speed_mps", "speed_cap_mps",
    "lane_valid", "lane_conf", "lane_source", "offset_m", "heading_deg", "curvature",
    "gt_offset_m", "gt_heading_deg", "lane_error_m", "target_offset_m",
    "obstacle", "obstacle_dist_m", "obstacle_raw", "obstacle_points", "gt_gap_m",
    "throttle", "steer", "brake", "emergency", "collisions", "lane_invasions",
    "x", "y", "yaw_deg", "distance_m",
]


class RunLogger:
    def __init__(self, cfg: dict, run_name: Optional[str] = None):
        self.enabled = cfg["logging"]["enabled"]
        self.every = int(cfg["logging"].get("every_n_ticks", 1))
        self.n = 0
        self.dir = None
        if not self.enabled:
            return
        name = run_name or time.strftime("%Y%m%d-%H%M%S")
        self.dir = Path(cfg["logging"]["dir"]) / name
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.dir / "config.yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        self._f = open(self.dir / "log.csv", "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._f, fieldnames=COLUMNS)
        self._w.writeheader()

    def log(self, row: dict):
        if not self.enabled:
            return
        self.n += 1
        if (self.n - 1) % self.every:
            return
        out = {}
        for k in COLUMNS:
            v = row.get(k, "")
            if isinstance(v, float):
                v = "inf" if math.isinf(v) else round(v, 4)
            out[k] = v
        self._w.writerow(out)

    def write_transitions(self, transitions):
        if not self.enabled:
            return
        with open(self.dir / "transitions.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "from", "to", "reason"])
            w.writerows(transitions)

    def write_summary(self, summary: dict):
        if not self.enabled:
            return
        with open(self.dir / "summary.yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump(summary, f, sort_keys=False)

    def close(self):
        if self.enabled and not self._f.closed:
            self._f.close()
