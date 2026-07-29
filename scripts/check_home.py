#!/usr/bin/env python3
"""Fail closed unless a fresh joint state is at the commissioned Home."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [f"joint{i}" for i in range(1, 7)]
DEFAULT_CATALOG = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "panthera_motion"
    / "config"
    / "motion_catalog.yaml"
)


def load_home(catalog_path):
    with open(catalog_path, encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)
    home = catalog.get("points", {}).get("home_near", {}).get("joints")
    if (
        not isinstance(home, list)
        or len(home) != len(JOINTS)
        or any(not isinstance(value, (int, float)) for value in home)
    ):
        raise ValueError("motion catalog home_near must contain six numeric joints")
    return [float(value) for value in home]


def home_error(names, positions, home):
    if len(names) != len(positions):
        raise ValueError("joint name/position length mismatch")
    values = dict(zip(names, positions))
    missing = [name for name in JOINTS if name not in values]
    if missing:
        raise ValueError(f"missing joints: {','.join(missing)}")
    actual = [values[name] for name in JOINTS]
    errors = [abs(actual[i] - home[i]) for i in range(6)]
    return actual, errors


def self_test(home):
    actual, errors = home_error(list(reversed(JOINTS)), list(reversed(home)), home)
    assert actual == home and max(errors) == 0.0
    try:
        home_error(JOINTS[:-1], home[:-1], home)
    except ValueError:
        pass
    else:
        raise AssertionError("missing joint must fail closed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--catalog", default=str(DEFAULT_CATALOG))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    try:
        home = load_home(args.catalog)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"HOME_CONFIG_INVALID {exc}", file=sys.stderr)
        return 4
    if args.self_test:
        self_test(home)
        print("check_home self-test OK")
        return 0
    if not 0.0 < args.tolerance <= 0.2 or args.timeout <= 0.0:
        parser.error("invalid timeout/tolerance")

    rclpy.init()
    node = Node("workcell_home_guard")
    latest = []
    node.create_subscription(JointState, "/joint_states", latest.append, 10)
    deadline = time.monotonic() + args.timeout
    try:
        while not latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not latest:
            print("HOME_UNKNOWN no fresh /joint_states message", file=sys.stderr)
            return 3
        try:
            actual, errors = home_error(
                latest[-1].name, latest[-1].position, home
            )
        except ValueError as exc:
            print(f"HOME_UNKNOWN {exc}", file=sys.stderr)
            return 3
        worst = max(range(6), key=errors.__getitem__)
        summary = (
            f"max_error={errors[worst]:.6f}rad joint={JOINTS[worst]} "
            f"actual=[{','.join(f'{value:.6f}' for value in actual)}]"
        )
        if errors[worst] > args.tolerance:
            print(f"HOME_MISMATCH {summary}", file=sys.stderr)
            return 2
        print(f"HOME_OK {summary}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
