"""Sensor rig: front RGB camera, front LiDAR, collision and lane-invasion
sensors (SCRUM-26). In synchronous mode every tick produces exactly one
camera image and one full LiDAR sweep; `tick()` returns them together as a
SensorFrame with the frame numbers checked.
"""
from __future__ import annotations

import logging
import queue
from typing import Optional

import numpy as np

from lanebot.interfaces import SensorFrame
from lanebot.sim.world import SLOW_OP_TIMEOUT_S

log = logging.getLogger(__name__)


class SensorRig:
    def __init__(self, sim, cfg: dict):
        import carla
        self.carla = carla
        self.sim = sim
        self.cfg = cfg
        s = cfg["sensors"]
        self.queues = {}
        self.sensors = {}
        self.collision_count = 0
        self.invasion_count = 0
        self.last_collision = None
        self._warmup_ticks = 3
        bp_lib = sim.world.get_blueprint_library()
        ego = sim.ego
        dt = float(cfg["carla"]["fixed_delta_s"])

        if s["camera"]["enabled"]:
            c = s["camera"]
            bp = bp_lib.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(c["image_w"]))
            bp.set_attribute("image_size_y", str(c["image_h"]))
            bp.set_attribute("fov", str(c["fov_deg"]))
            bp.set_attribute("sensor_tick", "0.0")
            # CARLA 1.0 camera look / cost options; skipped on servers that lack them.
            for attr, key in (("post_process_profile", "post_process_profile"),
                              ("enable_postprocess_effects", "postprocess_effects"),
                              ("enable_dlss", "dlss"), ("dlss_screen_percentage", "dlss_screen_percentage")):
                val = c.get(key)
                if val is None or val == "" or not bp.has_attribute(attr):
                    continue
                bp.set_attribute(attr, str(val).lower() if isinstance(val, bool) else str(val))
            tf = carla.Transform(carla.Location(x=c["x"], y=c["y"], z=c["z"]),
                                 carla.Rotation(pitch=c["pitch_deg"]))
            self._add("camera", bp, tf)

        if s["lidar"]["enabled"]:
            l = s["lidar"]
            # "front_bumper": just ahead of the bumper so the beams don't hit the car's own body
            self.lidar_x = sim.front_bumper_x() + 0.2 if l["x"] == "front_bumper" else float(l["x"])
            self.lidar_offset = np.array([self.lidar_x, l["y"], l["z"]], dtype=np.float32)
            bp = bp_lib.find("sensor.lidar.ray_cast")
            bp.set_attribute("range", str(l["range_m"]))
            bp.set_attribute("channels", str(l["channels"]))
            bp.set_attribute("points_per_second", str(l["points_per_second"]))
            bp.set_attribute("rotation_frequency", str(1.0 / dt))     # one full sweep per tick
            bp.set_attribute("upper_fov", str(l["upper_fov_deg"]))
            bp.set_attribute("lower_fov", str(l["lower_fov_deg"]))
            for attr, val in (("dropoff_general_rate", "0.0"), ("noise_stddev", "0.0")):
                if bp.has_attribute(attr):
                    bp.set_attribute(attr, val)
            tf = carla.Transform(carla.Location(x=self.lidar_x, y=l["y"], z=l["z"]))
            self._add("lidar", bp, tf)

        if s.get("collision", True):
            sensor = sim.world.spawn_actor(bp_lib.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
            sensor.listen(self._on_collision)
            sim.actors.append(sensor)
            self.sensors["collision"] = sensor
        if s.get("lane_invasion", True):
            sensor = sim.world.spawn_actor(bp_lib.find("sensor.other.lane_invasion"), carla.Transform(), attach_to=ego)
            sensor.listen(self._on_invasion)
            sim.actors.append(sensor)
            self.sensors["lane_invasion"] = sensor

    def _add(self, name, bp, tf):
        q: "queue.Queue" = queue.Queue()
        sensor = self.sim.world.spawn_actor(bp, tf, attach_to=self.sim.ego)
        sensor.listen(q.put)
        self.sim.actors.append(sensor)
        self.sensors[name] = sensor
        self.queues[name] = q

    def _on_collision(self, event):
        self.collision_count += 1
        other = getattr(event, "other_actor", None)
        self.last_collision = getattr(other, "type_id", "unknown")
        log.warning("COLLISION with %s", self.last_collision)

    def _on_invasion(self, event):
        self.invasion_count += 1

    # ------------------------------------------------------------------ tick
    def tick(self, timeout: float = 5.0) -> SensorFrame:
        """Advance the simulation one step and return all sensor data of that step."""
        col0, inv0 = self.collision_count, self.invasion_count
        if self._warmup_ticks > 0:          # first frames of new cameras compile shaders on UE5
            self._warmup_ticks -= 1
            timeout = max(timeout, SLOW_OP_TIMEOUT_S)
            self.sim.client.set_timeout(SLOW_OP_TIMEOUT_S)
            try:
                frame = self.sim.tick()
            finally:
                self.sim.client.set_timeout(float(self.cfg["carla"]["timeout_s"]))
        else:
            frame = self.sim.tick()
        data = {}
        for name, q in self.queues.items():
            while True:
                item = q.get(timeout=timeout)
                if item.frame == frame:
                    data[name] = item
                    break
                if item.frame > frame:          # should not happen in sync mode
                    log.warning("%s: got frame %d, expected %d", name, item.frame, frame)
                    data[name] = item
                    break
        image = self.image_to_bgr(data["camera"]) if "camera" in data else None
        lidar = self.lidar_to_vehicle(data["lidar"]) if "lidar" in data else None
        return SensorFrame(frame=frame, vehicle=self.sim.vehicle_state(), image_bgr=image, lidar_xyz=lidar,
                           collisions=self.collision_count - col0, lane_invasions=self.invasion_count - inv0)

    @staticmethod
    def image_to_bgr(image) -> np.ndarray:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        return arr[:, :, :3].copy()                      # CARLA delivers BGRA

    def lidar_to_vehicle(self, measurement) -> np.ndarray:
        pts = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
        return pts + self.lidar_offset                   # sensor frame is x fwd, y right, z up (no rotation)
