"""SCRUM-24: road layout - find straight, gentle-curve and sharp-curve test
sections with lane markings in a CARLA town.

CARLA towns can't be edited from Python (custom roads need RoadRunner, and
roads generated from OpenDRIVE have no painted markings for the camera), so
instead this script scans the town and lists spawn points whose next
`--length` metres are straight / gently curved / sharply curved. It also
draws the labels in the simulator so you can fly there and look.

    python scripts/find_track_segments.py --map Town04
    python scripts/find_track_segments.py --map Town05 --length 80

Use the printed indices in config/scenarios.yaml (vehicle.spawn.index).
"""
from _common import base_parser, config_from_args

import math

from lanebot.sim.map_lane import next_straightest

STRAIGHT_K = 1 / 300.0     # radius > 300 m
GENTLE_K = 1 / 60.0        # 60 m < radius < 300 m, else sharp


def classify(wp, length, step=2.0, carla=None):
    yaws, cur, travelled, junction = [], wp, 0.0, False
    while travelled < length:
        yaws.append(cur.transform.rotation.yaw)
        cur = next_straightest(cur, step)
        if cur is None:
            return None
        junction |= cur.is_junction
        travelled += step
    k_max = 0.0
    win = max(1, int(10 / step))
    for i in range(len(yaws) - win):
        dyaw = (yaws[i + win] - yaws[i] + 180) % 360 - 180
        k_max = max(k_max, abs(math.radians(dyaw)) / (win * step))
    kind = "straight" if k_max < STRAIGHT_K else "gentle" if k_max < GENTLE_K else "sharp"
    return kind, k_max, junction


def main():
    p = base_parser(__doc__)
    p.add_argument("--length", type=float, default=100.0)
    p.add_argument("--draw-seconds", type=float, default=120.0)
    args = p.parse_args()
    cfg = config_from_args(args)
    import carla
    client = carla.Client(cfg["carla"]["host"], cfg["carla"]["port"])
    client.set_timeout(cfg["carla"]["timeout_s"])
    world = client.get_world()
    if not world.get_map().name.split("/")[-1].startswith(cfg["carla"]["map"]):
        world = client.load_world(cfg["carla"]["map"])
    cmap = world.get_map()
    colours = {"straight": carla.Color(0, 255, 0), "gentle": carla.Color(255, 200, 0), "sharp": carla.Color(255, 0, 0)}
    found = {"straight": [], "gentle": [], "sharp": []}
    for i, sp in enumerate(cmap.get_spawn_points()):
        wp = cmap.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction:
            continue
        res = classify(wp, args.length)
        if res is None:
            continue
        kind, k, junction = res
        left = wp.get_left_lane()
        has_left = left is not None and left.lane_type == carla.LaneType.Driving and left.lane_id * wp.lane_id > 0
        found[kind].append((i, k, junction, has_left))
        world.debug.draw_string(sp.location + carla.Location(z=2), f"{i}:{kind}", draw_shadow=False,
                                color=colours[kind], life_time=args.draw_seconds, persistent_lines=True)
    for kind, rows in found.items():
        print(f"\n{kind.upper()} sections ({len(rows)}):  index  min-radius  junction  left-lane")
        for i, k, j, l in sorted(rows, key=lambda r: (r[2], -r[3]))[:15]:
            radius = "inf" if k < 1e-6 else f"{1 / k:7.0f} m"
            print(f"   {i:5d}   {radius:>9}   {'yes' if j else 'no':>5}   {'yes' if l else 'no':>5}")
    print("\nPrefer rows without junctions. Labels are drawn in the simulator for", args.draw_seconds, "s.")


if __name__ == "__main__":
    main()
