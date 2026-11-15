# Working on this repo (SCRUM-22)

## Branches

* `main` always runs. Never commit to it directly; only merge via pull request.
* One branch per Jira ticket, named after the key: `SCRUM-28-lane-detection`, `SCRUM-36-brake-logic`.
* Keep branches short-lived (a few days). Rebase on `main` before opening the PR.

## Commits

Start every message with the ticket key so it shows on the Jira issue:

```
SCRUM-30 add PID steering with gains from config
SCRUM-30 #comment tuned kp to 0.22 on Town04 #done
```

(`#comment`, `#time 2h`, `#done` are Jira Smart Commits, if the GitHub app is connected.)

## Pull requests

* Title `SCRUM-xx short description`, at least one reviewer from the team.
* Before asking for review: `python -m unittest discover tests` passes, and you ran
  `python scripts/run.py` (or the scenario for your ticket) in CARLA once.
* Changing a shared type in `lanebot/interfaces.py` or a key in `config/default.yaml`? Say so in the PR,
  because it affects everyone's module.

## Rules that keep modules compatible

1. Modules exchange data only through the classes in `lanebot/interfaces.py`.
2. Only `lanebot/sim/` imports `carla`. Everything else must run in the unit tests without CARLA.
3. Every number you tune goes in `config/default.yaml`, not in the code.
4. Use the sign conventions in `docs/architecture.md` (y right, offset > 0 = right of centre).
