"""Shared command-line handling for the scripts (adds the repo to sys.path)."""
import argparse
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lanebot.config import load_config  # noqa: E402


def base_parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", default=None, help="extra YAML merged over config/default.yaml")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override a config value, e.g. --set control.lateral.kp=0.3 (repeatable)")
    p.add_argument("--host", default=None)
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--map", default=None, help="CARLA town, e.g. Town04")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def config_from_args(args) -> dict:
    overrides = list(args.set)
    if args.host:
        overrides.append(f"carla.host={args.host}")
    if args.port:
        overrides.append(f"carla.port={args.port}")
    if args.map:
        overrides.append(f"carla.map={args.map}")
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S")
    return load_config(args.config, overrides)
