"""CARLA connection, synchronous mode, ego vehicle and cleanup (SCRUM-23)."""
from __future__ import annotations

import logging
import math
import random
from typing import List, Optional

from lanebot.interfaces import VehicleState
from lanebot.sim.map_lane import next_straightest

log = logging.getLogger(__name__)


class CarlaWorld:
    def __init__(self, cfg: dict):
        import carla
        self.carla = carla
        self.cfg = cfg
        c = cfg["carla"]
        self.client = carla.Client(c["host"], int(c["port"]))
        self.client.set_timeout(float(c["timeout_s"]))
        self.world = None
        self.map = None
        self.ego = None
        self.actors: List = []           # everything we spawned (destroyed in cleanup)
        self.original_settings = None
        self.spawn_transform = None
        random.seed(c.get("seed", 0))

    # --------------------------------------------------------------- setup
    def connect(self):
        c = self.cfg["carla"]
        world = self.client.get_world()
        if c.get("reload_map") or not world.get_map().name.split("/")[-1].startswith(c["map"]):
            log.info("loading map %s", c["map"])
            world = self.client.load_world(c["map"])
        self.world = world
        self.map = world.get_map()
        self.original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(c["fixed_delta_s"])
        settings.no_rendering_mode = bool(c.get("no_rendering", False))
        world.apply_settings(settings)
        weather = getattr(self.carla.WeatherParameters, c.get("weather", "ClearNoon"), None)
        if weather is not None:
            world.set_weather(weather)
        world.tick()
        return self

    def spawn_ego(self, spawn_index=None):
        v = self.cfg["vehicle"]
        bp = self.world.get_blueprint_library().find(v["blueprint"])
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", v.get("role_name", "hero"))
        spawn_points = self.map.get_spawn_points()
        idx = v["spawn"]["index"] if spawn_index is None else spawn_index
        if idx == "auto":
            idx = choose_spawn_point(self.map, spawn_points, v["spawn"]["min_clear_road_m"],
                                     v["spawn"]["require_left_lane"], self.carla)
        tf = spawn_points[int(idx)]
        tf.location.z += 0.3
        self.ego = self.world.try_spawn_actor(bp, tf)
        if self.ego is None:
            raise RuntimeError(f"could not spawn ego at spawn point {idx} (occupied?)")
        self.actors.append(self.ego)
        self.spawn_transform = tf
        self.spawn_index = int(idx)
        log.info("ego %s spawned at spawn point %s", v["blueprint"], idx)
        for _ in range(10):                 # let it settle on the road
            self.world.tick()
        return self.ego

    # ---------------------------------------------------------------- state
    def vehicle_state(self) -> VehicleState:
        tf = self.ego.get_transform()
        vel = self.ego.get_velocity()
        fwd = tf.get_forward_vector()
        speed = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z      # signed forward speed
        snap = self.world.get_snapshot()
        return VehicleState(snap.timestamp.elapsed_seconds, float(speed),
                            tf.location.x, tf.location.y, tf.location.z, tf.rotation.yaw)

    def front_bumper_x(self) -> float:
        bb = self.ego.bounding_box
        return float(bb.location.x + bb.extent.x)

    def max_wheel_angle_deg(self) -> float:
        try:
            return float(self.ego.get_physics_control().wheels[0].max_steer_angle)
        except Exception:
            return 70.0

    def follow_with_spectator(self):
        tf = self.ego.get_transform()
        yaw = math.radians(tf.rotation.yaw)
        loc = self.carla.Location(tf.location.x - 9 * math.cos(yaw), tf.location.y - 9 * math.sin(yaw),
                                  tf.location.z + 5)
        self.world.get_spectator().set_transform(
            self.carla.Transform(loc, self.carla.Rotation(pitch=-20, yaw=tf.rotation.yaw)))

    def apply_control(self, throttle: float, steer: float, brake: float, hand_brake: bool = False):
        self.ego.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), steer=float(steer), brake=float(brake), hand_brake=bool(hand_brake)))

    def tick(self) -> int:
        return self.world.tick()

    # -------------------------------------------------------------- cleanup
    def cleanup(self):
        try:
            for a in reversed(self.actors):
                try:
                    if hasattr(a, "stop") and getattr(a, "is_listening", False):
                        a.stop()
                except Exception:
                    pass
            self.client.apply_batch_sync([self.carla.command.DestroyActor(a) for a in self.actors if a is not None])
        except Exception as e:                  # never hide the original error
            log.warning("cleanup: %s", e)
        self.actors.clear()
        if self.world is not None and self.original_settings is not None:
            self.world.apply_settings(self.original_settings)


def choose_spawn_point(cmap, spawn_points, min_clear_m: float, need_left_lane: bool, carla) -> int:
    """Pick the spawn point with the longest junction-free road ahead (and a left lane if required)."""
    best, best_len = 0, -1.0
    for i, sp in enumerate(spawn_points):
        wp = cmap.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction:
            continue
        if need_left_lane:
            left = wp.get_left_lane()
            if left is None or left.lane_type != carla.LaneType.Driving or left.lane_id * wp.lane_id <= 0:
                continue
        length, cur = 0.0, wp
        while length < min_clear_m * 1.5:
            cur = next_straightest(cur, 5.0)
            if cur is None or cur.is_junction:
                break
            length += 5.0
        if length > best_len:
            best, best_len = i, length
        if length >= min_clear_m * 1.5:
            break
    log.info("auto spawn point %d (%.0f m clear road)", best, best_len)
    return best
