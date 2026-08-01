#!/usr/bin/env python3
"""Execute fixed routes and save per-joint controller telemetry."""

import argparse
import csv
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from panthera_interfaces.action import ExecuteMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [f'joint{index}' for index in range(1, 7)]


class TelemetryRecorder(Node):
    def __init__(self, speed_scale):
        super().__init__('route_telemetry_recorder')
        self.speed_scale = speed_scale
        self.motion = ActionClient(self, ExecuteMotion, '/motion/execute')
        self.active_route = ''
        self.started_at = None
        self.rows = []
        self.efforts = {joint: math.nan for joint in JOINTS}
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state', self._on_controller_state, 100)

    def _on_joint_state(self, message):
        for name, effort in zip(message.name, message.effort):
            if name in self.efforts:
                self.efforts[name] = effort

    def _on_controller_state(self, message):
        if self.started_at is None:
            return
        names = list(message.joint_names)
        row = {'time_s': time.monotonic() - self.started_at, 'route': self.active_route}
        fields = (
            ('desired_position', message.desired.positions),
            ('actual_position', message.actual.positions),
            ('error_position', message.error.positions),
            ('desired_velocity', message.desired.velocities),
            ('actual_velocity', message.actual.velocities),
            ('desired_acceleration', message.desired.accelerations),
        )
        for joint in JOINTS:
            index = names.index(joint)
            for label, values in fields:
                row[f'{joint}_{label}'] = values[index] if index < len(values) else math.nan
            row[f'{joint}_effort'] = self.efforts[joint]
        self.rows.append(row)

    def execute(self, route):
        if not self.motion.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('motion action unavailable')
        goal = ExecuteMotion.Goal()
        goal.route_name = route
        goal.speed_scale = self.speed_scale
        goal.dry_run = False
        self.active_route = route
        sent = self.motion.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=8.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            raise RuntimeError(f'{route}: goal rejected or response timeout')
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=90.0)
        if not result.done():
            sent.result().cancel_goal_async()
            raise RuntimeError(f'{route}: result timeout')
        wrapped = result.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            raise RuntimeError(
                f'{route}: status={wrapped.status} code={wrapped.result.error_code} '
                f'{wrapped.result.message}')


def finite(values):
    return [value for value in values if math.isfinite(value)]


def percentile(values, ratio):
    values = sorted(finite(values))
    if not values:
        return 0.0
    index = min(len(values) - 1, round((len(values) - 1) * ratio))
    return values[index]


def summarize(rows, routes, elapsed):
    times = [row['time_s'] for row in rows]
    gaps = [right - left for left, right in zip(times, times[1:])]
    summary = {
        'routes': routes,
        'elapsed_sec': elapsed,
        'samples': len(rows),
        'max_sample_gap_sec': max(gaps, default=0.0),
        'joints': {},
    }
    for joint in JOINTS:
        errors = finite([row[f'{joint}_error_position'] for row in rows])
        desired_velocity = finite([row[f'{joint}_desired_velocity'] for row in rows])
        actual_velocity = finite([row[f'{joint}_actual_velocity'] for row in rows])
        actual_acceleration = []
        for previous, current in zip(rows, rows[1:]):
            if previous['route'] != current['route']:
                continue
            interval = current['time_s'] - previous['time_s']
            if interval <= 1e-4:
                continue
            before = previous[f'{joint}_actual_velocity']
            after = current[f'{joint}_actual_velocity']
            if math.isfinite(before) and math.isfinite(after):
                actual_acceleration.append(abs(after - before) / interval)
        desired_acceleration = finite(
            [row[f'{joint}_desired_acceleration'] for row in rows])
        efforts = finite([row[f'{joint}_effort'] for row in rows])
        summary['joints'][joint] = {
            'rms_position_error_rad': math.sqrt(
                sum(value * value for value in errors) / len(errors)) if errors else 0.0,
            'max_position_error_rad': max(map(abs, errors), default=0.0),
            'max_desired_velocity_rad_sec': max(map(abs, desired_velocity), default=0.0),
            'max_actual_velocity_rad_sec': max(map(abs, actual_velocity), default=0.0),
            # Encoder feedback is quantized/batched, so p95 is the useful
            # physical indicator; max is retained to expose isolated spikes.
            'p95_actual_acceleration_rad_sec2': percentile(actual_acceleration, 0.95),
            'max_actual_acceleration_rad_sec2': max(actual_acceleration, default=0.0),
            'max_desired_acceleration_rad_sec2': max(
                map(abs, desired_acceleration), default=0.0),
            'max_abs_effort_nm': max(map(abs, efforts), default=0.0),
        }
    return summary


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--speed-scale', type=float, default=1.0)
    parser.add_argument('--routes', nargs='+', required=True)
    args = parser.parse_args()
    if not 0.1 <= args.speed_scale <= 1.0:
        parser.error('--speed-scale must be in [0.1, 1.0]')
    return args


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init()
    node = TelemetryRecorder(args.speed_scale)
    error = None
    started = time.monotonic()
    node.started_at = started
    try:
        for route in args.routes:
            node.execute(route)
    except Exception as exc:  # Preserve partial telemetry for failure analysis.
        error = str(exc)
    elapsed = time.monotonic() - started
    node.destroy_node()
    rclpy.shutdown()

    csv_path = os.path.join(args.output_dir, 'joint_telemetry.csv')
    if node.rows:
        with open(csv_path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=node.rows[0].keys())
            writer.writeheader()
            writer.writerows(node.rows)
    summary = summarize(node.rows, args.routes, elapsed)
    summary['success'] = error is None
    summary['error'] = error or ''
    with open(os.path.join(args.output_dir, 'summary.json'), 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    raise SystemExit(0 if error is None else 1)


if __name__ == '__main__':
    main()
