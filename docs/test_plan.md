# Test plan (SCRUM-41)

Goal: show that the vehicle follows its lane and stops safely before an obstacle (and, as a stretch,
passes it) across the speeds, obstacle types and distances below. Every scenario is defined in
`config/scenarios.yaml` and run automatically:

```bash
python scripts/run_scenarios.py                 # all
python scripts/run_scenarios.py --only S03 S07  # some
python scripts/run_scenarios.py --repeat 3      # flakiness check
```

Results go to `runs/scenarios-<time>/results.md`, a table ready to paste into Jira or the report (SCRUM-42).

## Environment

* CARLA (same version on all machines), map **Town04**, synchronous mode at 20 Hz, weather ClearNoon unless stated.
* Ego: `vehicle.tesla.model3`, spawn point chosen automatically (long road with a same-direction left lane),
  or fixed with `--set vehicle.spawn.index=N` (use `scripts/find_track_segments.py` to pick sections).
* Lane source: vision (camera) unless stated.

## Scenarios and pass criteria

| ID | What | Speed | Obstacle | Pass criteria |
|---|---|---|---|---|
| S01 | Lane keeping, straight road | 30 km/h | none | no collision, max true offset ≤ 0.5 m, 0 lane invasions, ≥ 300 m |
| S02 | Full lap (straights + curves) | 30 km/h | none | lap completed, max offset ≤ 0.6 m, 0 lane invasions |
| S03 | Stop for parked car | 30 km/h | car, 70 m, ego lane | no collision, ends STOPPED, gap 1–6 m |
| S04 | Stop at higher speed | 40 km/h | car, 100 m | as S03 |
| S05 | Small, low obstacle | 20 km/h | traffic cone, 50 m | as S03 |
| S06 | Obstacle off-centre in the lane | 30 km/h | barrier, 60 m, 0.8 m right | as S03 |
| S07 | Obstacle in the next lane (false-positive check) | 30 km/h | car in left lane, 60 m | no collision, never STOPPED, ≥ 150 m |
| S08 | Short reaction distance | from standstill | car, 25 m | no collision, ends STOPPED, gap 0.8–6 m |
| S09 | Wet weather | 30 km/h | car, 70 m | as S03 |
| S10 | Baseline with map lanes (isolates control from perception) | 30 km/h | car, 70 m | as S03, max offset ≤ 0.3 m |
| S11 | (stretch) Avoid obstacle | 15 km/h while passing | car, 70 m | no collision, enters AVOIDING, ends FOLLOW_LANE, ≥ 150 m |

"Gap" is the true bumper-to-obstacle distance from the simulator, not the detector's estimate.
"Max true offset" is measured against the CARLA map while following the lane.

## Reference numbers (from `config/default.yaml`)

| Speed km/h | Comfortable stop (3 m/s²) m | Brake trigger m | Emergency distance m |
|---|---|---|---|
| 10 | 2.1 | 5.1 | 2.4 |
| 20 | 6.8 | 9.8 | 5.0 |
| 30 | 14.1 | 17.1 | 8.8 |
| 40 | 23.9 | 26.9 | 13.8 |
| 50 | 36.3 | 39.3 | 20.0 |

The LiDAR detector must see obstacles reliably beyond the brake trigger distance at the cruise speed. Check
with `python scripts/measure_detection_range.py` (and `--blueprint static.prop.trafficcone01`, `--lane left`).

## Component checks before the scenario runs

| Check | Command | Expected |
|---|---|---|
| Unit + fake-CARLA tests | `python -m unittest discover tests` | all pass |
| Sensors stream every tick | `python scripts/check_sensors.py` | RESULT: OK |
| Lane detector accuracy | `python scripts/record_samples.py` then `python scripts/eval_lane_detector.py runs/samples` | lane found > 95 %, mean offset error < 0.15 m |
| Detection range | `python scripts/measure_detection_range.py` | ≥ 90 % detection up to ≥ 40 m |
| Lane-keeping lap | `python scripts/run.py --no-obstacles --lap --time 900` then `python scripts/plot_run.py runs/<run>` | max offset < 0.6 m |

## Recording failures

For each FAIL: keep the run folder (`log.csv`, `plot.png`, `transitions.csv`), run `scripts/plot_run.py`,
and open a Jira bug linked to SCRUM-42 with the scenario ID, the failure reason from `results.md`, and the plot.
