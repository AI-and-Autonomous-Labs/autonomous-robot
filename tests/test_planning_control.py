"""Controllers, safety model, state machine and avoidance (SCRUM-30/31/35/36/37/38)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lanebot.config import load_config  # noqa: E402
from lanebot.control.lateral import LateralController  # noqa: E402
from lanebot.control.longitudinal import LongitudinalController, SpeedPlanner, curve_speed_limit  # noqa: E402
from lanebot.control.pid import PID  # noqa: E402
from lanebot.interfaces import DriveState, LaneEstimate, ObstacleInfo  # noqa: E402
from lanebot.planning.avoidance import AvoidanceManeuver  # noqa: E402
from lanebot.planning.safety import SafetyModel  # noqa: E402
from lanebot.planning.state_machine import BehaviourStateMachine  # noqa: E402

CFG = load_config()
DT = CFG["carla"]["fixed_delta_s"]


def lane(offset=0.0, heading_deg=0.0, curv=0.0):
    est = LaneEstimate.from_coeffs((-offset, -math.tan(math.radians(heading_deg)), curv / 2), 3.5, 1.0, "test")
    return est


NO_OBS = ObstacleInfo(detected=False)


def obs(d):
    return ObstacleInfo(detected=True, distance_m=d, raw_detected=True, confidence=1.0)


class TestPID(unittest.TestCase):
    def test_proportional_and_limits(self):
        p = PID(2.0, 0, 0, out_min=-1, out_max=1)
        self.assertAlmostEqual(p.update(0.25, 0.05), 0.5)
        self.assertEqual(p.update(5.0, 0.05), 1.0)

    def test_integral_anti_windup(self):
        p = PID(0.1, 1.0, 0, i_limit=0.3, out_min=-1, out_max=1)
        for _ in range(1000):
            out = p.update(1.0, 0.05)
        self.assertLessEqual(out, 0.4 + 1e-9)            # kp*1 + i_limit


class TestLateral(unittest.TestCase):
    def setUp(self):
        self.c = LateralController(CFG["control"]["lateral"])

    def test_steers_back_to_centre(self):
        self.assertLess(self.c.update(lane(offset=0.5), 0.0, 8.0, DT), 0)      # right of centre -> steer left
        self.c.reset()
        self.assertGreater(self.c.update(lane(offset=-0.5), 0.0, 8.0, DT), 0)
        self.c.reset()
        self.assertGreater(self.c.update(lane(heading_deg=-5), 0.0, 8.0, DT), 0)  # pointing left -> steer right

    def test_target_offset(self):
        # car on the centre but should be 3.5 m left (avoidance) -> steer left
        self.assertLess(self.c.update(lane(), -3.5, 5.0, DT), 0)

    def test_curve_feedforward(self):
        self.assertGreater(self.c.update(lane(curv=1 / 60), 0.0, 5.0, DT), 0)

    def test_limits(self):
        cfg = CFG["control"]["lateral"]
        steer = 0.0
        for _ in range(100):
            steer = self.c.update(lane(offset=-3.0), 0.0, cfg["high_speed_mps"], DT)
        self.assertAlmostEqual(abs(steer), cfg["max_steer_high_speed"], places=6)
        c2 = LateralController(cfg)
        first = c2.update(lane(offset=-3.0), 0.0, 1.0, DT)
        self.assertLessEqual(abs(first), cfg["max_steer_rate_per_s"] * DT + 1e-9)

    def test_closed_loop_kinematic(self):
        """Bicycle model starting 1 m off centre converges to < 0.1 m within 15 s at 30 km/h."""
        c = LateralController(CFG["control"]["lateral"])
        y, yaw, v, L = 1.0, 0.0, 30 / 3.6, CFG["control"]["lateral"]["wheelbase_m"]
        for _ in range(int(15 / DT)):
            s = c.update(lane(offset=y, heading_deg=math.degrees(yaw)), 0.0, v, DT)
            yaw += v / L * math.tan(s * math.radians(70)) * DT
            y += v * math.sin(yaw) * DT
        self.assertLess(abs(y), 0.1)
        self.assertLess(abs(math.degrees(yaw)), 1.0)


class TestSpeed(unittest.TestCase):
    def test_curve_limit(self):
        self.assertAlmostEqual(curve_speed_limit(1 / 50, 2.0, 1.0, 30.0), 10.0)
        self.assertEqual(curve_speed_limit(0.0, 2.0, 1.0, 8.3), 8.3)

    def test_planner_ramps_up_and_drops(self):
        sp = SpeedPlanner(CFG["control"]["speed"], CFG["control"]["longitudinal"])
        t1 = sp.update(100, 0.0, DT, 0.0)
        self.assertAlmostEqual(t1, CFG["control"]["longitudinal"]["max_accel_mps2"] * DT)
        for _ in range(400):
            t = sp.update(100, 0.0, DT, sp.target)
        self.assertAlmostEqual(t, 30 / 3.6, places=3)
        self.assertEqual(sp.update(0.0, 0.0, DT, t), 0.0)

    def test_longitudinal(self):
        lc = LongitudinalController(CFG["control"]["longitudinal"])
        thr, br = lc.update(8.0, 2.0, DT)
        self.assertGreater(thr, 0)
        self.assertEqual(br, 0)
        lc.reset()
        thr, br = lc.update(2.0, 8.0, DT)
        self.assertEqual(thr, 0)
        self.assertGreater(br, 0)
        self.assertEqual(lc.update(0.0, 0.05, DT), (0.0, CFG["control"]["longitudinal"]["hold_brake"]))
        self.assertEqual(lc.update(5.0, 5.0, DT, emergency=True), (0.0, 1.0))


class TestSafety(unittest.TestCase):
    def test_numbers(self):
        s = SafetyModel(CFG["safety"])
        v = 30 / 3.6
        self.assertAlmostEqual(s.stopping_distance(v, 3.0), v * 0.3 + v * v / 6.0)
        self.assertGreater(s.brake_trigger_distance(v), s.emergency_distance(v))
        # approach speed is the inverse of the stopping distance
        d = 25.0
        va = s.approach_speed(d)
        self.assertAlmostEqual(s.stopping_distance(va, 3.0) + s.stop_margin, d, places=6)
        self.assertEqual(s.approach_speed(1.0), 0.0)


class TestStateMachine(unittest.TestCase):
    def fsm(self, avoid=False):
        a = dict(CFG["avoidance"], enabled=avoid)
        return BehaviourStateMachine(CFG["safety"], CFG["control"]["speed"], a)

    def test_brake_stop_resume(self):
        f = self.fsm()
        out = f.update(lane(), obs(50), 8.3, DT)
        self.assertEqual(out.state, DriveState.FOLLOW_LANE)               # 50 m: still far
        out = f.update(lane(), obs(15), 8.3, DT)
        self.assertEqual(out.state, DriveState.BRAKING)
        self.assertLess(out.speed_cap_mps, 8.3)
        out = f.update(lane(), obs(3.2), 0.05, DT)
        self.assertEqual(out.state, DriveState.STOPPED)
        self.assertEqual(out.speed_cap_mps, 0.0)
        t = 0
        while out.state == DriveState.STOPPED and t < 10:
            out = f.update(lane(), NO_OBS, 0.0, DT)
            t += DT
        self.assertEqual(out.state, DriveState.FOLLOW_LANE)
        self.assertAlmostEqual(t, CFG["safety"]["resume_time_s"], delta=2 * DT)

    def test_emergency(self):
        f = self.fsm()
        out = f.update(lane(), obs(4.0), 8.3, DT)
        self.assertTrue(out.emergency_brake)

    def test_short_dropout_keeps_stopped(self):
        f = self.fsm()
        f.update(lane(), obs(6), 5.0, DT)
        f.update(lane(), obs(3.0), 0.0, DT)
        self.assertEqual(f.state, DriveState.STOPPED)
        for _ in range(5):
            f.update(lane(), NO_OBS, 0.0, DT)
        self.assertEqual(f.state, DriveState.STOPPED)

    def test_lane_lost(self):
        f = self.fsm()
        out = None
        for _ in range(int(CFG["safety"]["lane_lost_time_s"] / DT) + 1):
            out = f.update(LaneEstimate.invalid(), NO_OBS, 8.0, DT)
        self.assertEqual(out.state, DriveState.SAFE_STOP)
        self.assertEqual(out.speed_cap_mps, 0.0)
        for _ in range(int(CFG["safety"]["lane_recover_time_s"] / DT) + 1):
            out = f.update(lane(), NO_OBS, 0.0, DT)
        self.assertEqual(out.state, DriveState.FOLLOW_LANE)

    def test_avoidance_flow(self):
        f = self.fsm(avoid=True)
        f.update(lane(), obs(6), 5.0, DT)
        f.update(lane(), obs(4.0), 0.0, DT)
        self.assertEqual(f.state, DriveState.STOPPED)
        for _ in range(int(CFG["avoidance"]["wait_before_s"] / DT) + 2):
            out = f.update(lane(), obs(4.0), 0.0, DT, left_lane_free=True)
        self.assertEqual(out.state, DriveState.AVOIDING)
        self.assertTrue(out.use_map_lane)
        offsets = []
        for _ in range(2000):
            out = f.update(lane(), obs(4.0), 4.0, DT, left_lane_free=True, obstacle_left=NO_OBS)
            offsets.append(out.target_offset_m)
            if out.state != DriveState.AVOIDING:
                break
        self.assertEqual(out.state, DriveState.FOLLOW_LANE)
        self.assertAlmostEqual(min(offsets), -3.5, delta=0.01)
        self.assertAlmostEqual(offsets[-2], 0.0, delta=0.1)

    def test_no_avoid_when_left_blocked(self):
        f = self.fsm(avoid=True)
        f.update(lane(), obs(6), 5.0, DT)
        f.update(lane(), obs(4.0), 0.0, DT)
        for _ in range(200):
            f.update(lane(), obs(4.0), 0.0, DT, left_lane_free=False)
        self.assertEqual(f.state, DriveState.STOPPED)


class TestAvoidanceProfile(unittest.TestCase):
    def test_profile_continuous(self):
        m = AvoidanceManeuver(CFG["avoidance"])
        m.start(obstacle_distance_m=5.0, lane_width_m=3.5)
        prev = 0.0
        s = 0.0
        while s < m.total_length + 1:
            o = m.target_offset(s)
            self.assertLess(abs(o - prev), 0.1)          # smooth at 0.1 m steps
            prev = o
            s += 0.1
        self.assertGreaterEqual(m.ramp + m.hold, 5.0 + CFG["avoidance"]["obstacle_length_m"])


if __name__ == "__main__":
    unittest.main()
