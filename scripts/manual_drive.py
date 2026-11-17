"""SCRUM-23: spawn the ego vehicle and drive it with speed / steering commands.

Scripted (default): runs a fixed sequence of (seconds, throttle, steer, brake)
steps and prints speed and position, so everyone can check their CARLA setup.

    python scripts/manual_drive.py
    python scripts/manual_drive.py --step 3,0.5,0,0 --step 2,0.3,-0.3,0 --step 2,0,0,1

Keyboard (needs `pip install pygame`): W/S throttle/brake, A/D steer, Q quit.

    python scripts/manual_drive.py --keyboard
"""
from _common import base_parser, config_from_args

import logging

import numpy as np

from lanebot.sim.world import CarlaWorld

log = logging.getLogger("manual_drive")

DEFAULT_SEQUENCE = [   # seconds, throttle, steer, brake
    (4.0, 0.6, 0.0, 0.0),
    (2.0, 0.4, -0.25, 0.0),
    (2.0, 0.4, 0.25, 0.0),
    (2.0, 0.4, 0.0, 0.0),
    (3.0, 0.0, 0.0, 1.0),
]


def scripted(sim, sequence, dt):
    for secs, thr, steer, brake in sequence:
        log.info("throttle %.2f steer %+.2f brake %.2f for %.1f s", thr, steer, brake, secs)
        for i in range(int(secs / dt)):
            sim.apply_control(thr, steer, brake)
            sim.tick()
            sim.follow_with_spectator()
            if i % int(1 / dt) == 0:
                s = sim.vehicle_state()
                log.info("  t=%.1f speed %.1f km/h  pos (%.1f, %.1f) yaw %.0f",
                         s.timestamp, s.speed_mps * 3.6, s.x, s.y, s.yaw_deg)


def keyboard(sim, cfg, dt):
    import carla
    import pygame
    from lanebot.sim.sensors import SensorRig
    cfg["sensors"]["lidar"]["enabled"] = False
    rig = SensorRig(sim, cfg)
    w, h = cfg["sensors"]["camera"]["image_w"], cfg["sensors"]["camera"]["image_h"]
    pygame.init()
    screen = pygame.display.set_mode((w, h))
    pygame.display.set_caption("LaneBot manual drive - W/S/A/D, Q to quit")
    steer = 0.0
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_q):
                return
        keys = pygame.key.get_pressed()
        thr = 0.6 if keys[pygame.K_w] else 0.0
        brake = 1.0 if keys[pygame.K_s] else 0.0
        target = -0.5 if keys[pygame.K_a] else 0.5 if keys[pygame.K_d] else 0.0
        steer += np.clip(target - steer, -0.08, 0.08)
        sim.apply_control(thr, steer, brake)
        frame = rig.tick()
        sim.follow_with_spectator()
        if frame.image_bgr is not None:
            rgb = frame.image_bgr[:, :, ::-1]
            screen.blit(pygame.surfarray.make_surface(rgb.swapaxes(0, 1)), (0, 0))
            pygame.display.flip()


def main():
    p = base_parser(__doc__)
    p.add_argument("--keyboard", action="store_true")
    p.add_argument("--step", action="append", default=[], help="seconds,throttle,steer,brake")
    args = p.parse_args()
    cfg = config_from_args(args)
    dt = cfg["carla"]["fixed_delta_s"]
    sim = CarlaWorld(cfg)
    try:
        sim.connect()
        sim.spawn_ego()
        if args.keyboard:
            keyboard(sim, cfg, dt)
        else:
            seq = [tuple(float(v) for v in s.split(",")) for s in args.step] or DEFAULT_SEQUENCE
            scripted(sim, seq, dt)
    finally:
        sim.cleanup()


if __name__ == "__main__":
    main()
