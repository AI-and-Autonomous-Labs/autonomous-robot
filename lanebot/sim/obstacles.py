"""Config-driven static obstacle spawner (SCRUM-25).

Each obstacle in the config is placed along the ego lane, measured from the
ego spawn point, so the same YAML works on any map:

    obstacles:
      - blueprint: vehicle.mini.cooper   # any vehicle or static.prop.* blueprint
        distance_m: 70                   # along the lane from the spawn point
        lateral_offset_m: 0.0            # + = right of the lane centre
        lane: ego                        # ego | left | right
        yaw_offset_deg: 0

CARLA cannot scale props at runtime, so "size" is chosen by blueprint.
Handy blueprints on CARLA 1.0 (0.9.x ids such as vehicle.audi.a2 are mapped
to their 1.0 equivalent automatically, see BLUEPRINT_ALIASES in world.py):
    static.prop.trafficcone01   traffic cone
    static.prop.streetbarrier   street barrier
    static.prop.creasedbox01    cardboard box
    vehicle.mini.cooper         small car
    vehicle.carlacola.actors    box truck
The real bounding box of each spawned obstacle is logged.
"""
from __future__ import annotations

import logging
import math
from typing import List

from lanebot.sim.map_lane import next_straightest
from lanebot.sim.world import find_blueprint

log = logging.getLogger(__name__)


class ObstacleSpawner:
    def __init__(self, sim):
        import carla
        self.carla = carla
        self.sim = sim
        self.spawned: List = []

    def waypoint_ahead(self, start_wp, distance: float, step: float = 2.0):
        wp, travelled = start_wp, 0.0
        while travelled + step <= distance:
            nxt = next_straightest(wp, step)
            if nxt is None:
                break
            wp, travelled = nxt, travelled + step
        rest = distance - travelled
        if rest > 0.05:
            nxt = next_straightest(wp, rest)
            wp = nxt or wp
        return wp

    def spawn_all(self, specs: list, start_transform=None) -> List:
        start_transform = start_transform or self.sim.spawn_transform
        start_wp = self.sim.map.get_waypoint(start_transform.location, project_to_road=True,
                                             lane_type=self.carla.LaneType.Driving)
        for spec in specs or []:
            actor = self.spawn_one(spec, start_wp)
            if actor is not None:
                self.spawned.append(actor)
        return self.spawned

    def spawn_one(self, spec: dict, start_wp):
        wp = self.waypoint_ahead(start_wp, float(spec["distance_m"]))
        lane = spec.get("lane", "ego")
        if lane in ("left", "right"):
            side = wp.get_left_lane() if lane == "left" else wp.get_right_lane()
            if side is None:
                log.error("obstacle %s: no %s lane at %.0f m", spec["blueprint"], lane, spec["distance_m"])
                return None
            wp = side
        tf = wp.transform
        yaw = math.radians(tf.rotation.yaw)
        off = float(spec.get("lateral_offset_m", 0.0))
        # right vector in CARLA (left-handed): (-sin, cos)
        loc = self.carla.Location(tf.location.x - off * math.sin(yaw),
                                  tf.location.y + off * math.cos(yaw),
                                  tf.location.z + 0.2)
        rot = self.carla.Rotation(yaw=tf.rotation.yaw + float(spec.get("yaw_offset_deg", 0.0)))
        bp = find_blueprint(self.sim.world, spec["blueprint"])
        actor = self.sim.world.try_spawn_actor(bp, self.carla.Transform(loc, rot))
        if actor is None:
            log.error("could not spawn obstacle %s at %.0f m", spec["blueprint"], spec["distance_m"])
            return None
        if bp.id.startswith("vehicle."):
            actor.apply_control(self.carla.VehicleControl(hand_brake=True, brake=1.0))   # parked
        self.sim.actors.append(actor)
        bb = getattr(actor, "bounding_box", None)
        size = f"{2*bb.extent.x:.1f} x {2*bb.extent.y:.1f} x {2*bb.extent.z:.1f} m" if bb else "?"
        log.info("obstacle %s at %.0f m (%s lane, offset %.1f m), size %s",
                 spec["blueprint"], spec["distance_m"], lane, off, size)
        return actor

    def destroy_all(self):
        for a in self.spawned:
            try:
                a.destroy()
                if a in self.sim.actors:
                    self.sim.actors.remove(a)
            except Exception:
                pass
        self.spawned.clear()


def gap_to_obstacles(ego, obstacles) -> float:
    """Ground-truth bumper-to-obstacle gap along the ego heading (for logs and pass/fail).

    Only counts obstacles roughly in front (|lateral| < 2 m); returns inf if none.
    """
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    best = float("inf")
    ego_ext = ego.bounding_box.extent.x + ego.bounding_box.location.x
    for ob in obstacles:
        l = ob.get_location()
        dx, dy = l.x - tf.location.x, l.y - tf.location.y
        ahead = dx * fx + dy * fy
        lateral = -dx * fy + dy * fx
        bb = getattr(ob, "bounding_box", None)
        ob_ext = bb.extent.x if bb is not None else 0.5
        if ahead > 0 and abs(lateral) < 2.0:
            best = min(best, ahead - ego_ext - ob_ext)
    return best
