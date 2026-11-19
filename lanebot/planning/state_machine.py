"""Behaviour state machine (SCRUM-38) with brake / stop / resume logic (SCRUM-36).

    FOLLOW_LANE --obstacle within brake trigger--> BRAKING
    BRAKING     --speed ~ 0--------------------------> STOPPED
    BRAKING     --obstacle gone for clear_time-------> FOLLOW_LANE
    STOPPED     --path clear for resume_time---------> FOLLOW_LANE
    STOPPED     --(avoidance on) waited, left lane free--> AVOIDING
    AVOIDING    --manoeuvre finished-----------------> FOLLOW_LANE
    AVOIDING    --left lane blocked------------------> BRAKING
    any         --lane lost for lane_lost_time-------> SAFE_STOP
    SAFE_STOP   --lane back for lane_recover_time----> FOLLOW_LANE

Pure Python: no CARLA calls, so it is unit-tested in tests/.
"""
from __future__ import annotations

from typing import Optional

from lanebot.control.longitudinal import kmh
from lanebot.interfaces import DriveState, LaneEstimate, ObstacleInfo, PlannerOutput
from lanebot.planning.avoidance import AvoidanceManeuver
from lanebot.planning.safety import SafetyModel


class BehaviourStateMachine:
    def __init__(self, safety_cfg: dict, speed_cfg: dict, avoid_cfg: dict):
        self.cfg = safety_cfg
        if avoid_cfg.get("enabled"):
            # stop further back so there is room to steer out around the obstacle
            safety_cfg = dict(safety_cfg, stop_margin_m=max(safety_cfg["stop_margin_m"], avoid_cfg["stop_gap_m"]))
        self.safety = SafetyModel(safety_cfg)
        self.cruise = kmh(speed_cfg["cruise_kmh"])
        self.avoid_cfg = avoid_cfg
        self.avoidance = AvoidanceManeuver(avoid_cfg)
        self.state = DriveState.FOLLOW_LANE
        self.time_in_state = 0.0
        self.clear_timer = 0.0          # how long no obstacle has been seen
        self.lane_lost_timer = 0.0
        self.lane_ok_timer = 0.0
        self.last_obstacle_distance = float("inf")
        self.transitions = []           # (time, from, to, reason) for logs / tests
        self.t = 0.0

    # ----------------------------------------------------------------- update
    def update(self, lane: LaneEstimate, obstacle: ObstacleInfo, speed_mps: float, dt: float,
               left_lane_free: bool = False, obstacle_left: Optional[ObstacleInfo] = None) -> PlannerOutput:
        self.t += dt
        self.time_in_state += dt
        s = self.safety

        # timers
        if obstacle.detected:
            self.clear_timer = 0.0
            self.last_obstacle_distance = obstacle.distance_m
        else:
            self.clear_timer += dt
        if lane.valid:
            self.lane_ok_timer += dt
            self.lane_lost_timer = 0.0
        else:
            self.lane_lost_timer += dt
            self.lane_ok_timer = 0.0

        st = self.state
        d = obstacle.distance_m

        # lane loss overrides everything except an active map-guided avoidance
        if st != DriveState.AVOIDING and st != DriveState.SAFE_STOP and self.lane_lost_timer >= self.cfg["lane_lost_time_s"]:
            self._go(DriveState.SAFE_STOP, "lane lost")
        elif st == DriveState.FOLLOW_LANE:
            # +1 m buffer: a car that crept up to the obstacle at low speed must still end in STOPPED
            if obstacle.detected and d <= s.brake_trigger_distance(speed_mps) + 1.0:
                self._go(DriveState.BRAKING, f"obstacle at {d:.1f} m")
        elif st == DriveState.BRAKING:
            if speed_mps <= self.cfg["stopped_speed_mps"] and obstacle.detected:
                self._go(DriveState.STOPPED, f"stopped {d:.1f} m before obstacle")
            elif self.clear_timer >= self.cfg["clear_time_s"]:
                self._go(DriveState.FOLLOW_LANE, "obstacle gone")
        elif st == DriveState.STOPPED:
            if self.clear_timer >= self.cfg["resume_time_s"]:
                self._go(DriveState.FOLLOW_LANE, "path clear, resuming")
            elif (self.avoid_cfg.get("enabled") and left_lane_free and obstacle.detected
                  and self.time_in_state >= self.avoid_cfg["wait_before_s"]):
                self.avoidance.start(d, lane.lane_width_m if lane.valid else 3.5)
                self._go(DriveState.AVOIDING, "passing on the left")
        elif st == DriveState.AVOIDING:
            if obstacle_left is not None and obstacle_left.detected and \
                    obstacle_left.distance_m <= s.brake_trigger_distance(speed_mps) and self.avoidance.phase == "OUT":
                self.avoidance.stop()
                self._go(DriveState.BRAKING, "left lane blocked, aborting pass")
            elif self.avoidance.done:
                self.avoidance.stop()
                self._go(DriveState.FOLLOW_LANE, "pass complete")
        elif st == DriveState.SAFE_STOP:
            if self.lane_ok_timer >= self.cfg["lane_recover_time_s"]:
                self._go(DriveState.FOLLOW_LANE, "lane recovered")

        return self._output(obstacle, speed_mps, dt)

    # ----------------------------------------------------------------- output
    def _output(self, obstacle: ObstacleInfo, speed: float, dt: float) -> PlannerOutput:
        s = self.safety
        st = self.state
        d = obstacle.distance_m if obstacle.detected else float("inf")
        emergency = obstacle.detected and st in (DriveState.FOLLOW_LANE, DriveState.BRAKING) \
            and d <= s.emergency_distance(speed)

        if st == DriveState.FOLLOW_LANE:
            cap = min(self.cruise, s.approach_speed(d)) if obstacle.detected else self.cruise
            return PlannerOutput(st, cap, emergency_brake=emergency, reason="cruise")
        if st == DriveState.BRAKING:
            return PlannerOutput(st, s.approach_speed(d) if obstacle.detected else 0.0,
                                 emergency_brake=emergency, reason="braking")
        if st == DriveState.STOPPED:
            return PlannerOutput(st, 0.0, reason="holding")
        if st == DriveState.AVOIDING:
            offset = self.avoidance.update(speed * dt)
            slope, curv = self.avoidance.derivatives(self.avoidance.s)
            return PlannerOutput(st, kmh(self.avoid_cfg["speed_kmh"]), target_offset_m=offset,
                                 target_slope=slope, target_curvature=curv,
                                 use_map_lane=True, reason=self.avoidance.phase)
        return PlannerOutput(DriveState.SAFE_STOP, 0.0, reason="lane lost")

    def _go(self, new: DriveState, reason: str):
        if new != self.state:
            self.transitions.append((round(self.t, 2), self.state.value, new.value, reason))
            self.state = new
            self.time_in_state = 0.0
