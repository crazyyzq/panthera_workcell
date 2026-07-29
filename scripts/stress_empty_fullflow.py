#!/usr/bin/env python3
"""Run repeated empty-cup production trajectories through one persistent ROS node."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from panthera_interfaces.action import ExecuteMotion
from panthera_interfaces.srv import SetBrush
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


HOME = [0.0] * 6
JOINTS = [f'joint{index}' for index in range(1, 7)]


class StressRunner(Node):
    def __init__(self, speed_scale):
        super().__init__('empty_fullflow_stress_runner')
        self.speed_scale = speed_scale
        self.motion = ActionClient(self, ExecuteMotion, '/motion/execute')
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')
        self.brush = self.create_client(
            SetBrush, '/spectrometer_cell/debug/set_brush')
        self.recover = self.create_client(
            Trigger, '/spectrometer_cell/recover_home')
        self.last_joint = {}
        self.last_joint_change_time = {}
        self.last_sample_time = None
        self.reset_metrics()
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.on_controller_state,
            50)

    def reset_metrics(self):
        self.samples = 0
        self.max_gap = 0.0
        self.max_jump = {name: 0.0 for name in JOINTS}
        self.max_observed_rate = {name: 0.0 for name in JOINTS}
        self.max_tracking_error = {name: 0.0 for name in JOINTS}
        self.nonfinite = 0

    def on_joint_state(self, message):
        now = time.monotonic()
        if self.last_sample_time is not None:
            self.max_gap = max(self.max_gap, now - self.last_sample_time)
        self.last_sample_time = now
        positions = dict(zip(message.name, message.position))
        for name in JOINTS:
            if name not in positions:
                continue
            value = positions[name]
            if not math.isfinite(value):
                self.nonfinite += 1
                continue
            if name in self.last_joint:
                jump = abs(value - self.last_joint[name])
                self.max_jump[name] = max(self.max_jump[name], jump)
                if jump > 1e-9:
                    previous_change = self.last_joint_change_time.get(name)
                    if previous_change is not None and now > previous_change:
                        self.max_observed_rate[name] = max(
                            self.max_observed_rate[name],
                            jump / (now - previous_change))
                    self.last_joint_change_time[name] = now
            else:
                self.last_joint_change_time[name] = now
            self.last_joint[name] = value
        self.samples += 1

    def on_controller_state(self, message):
        for name, error in zip(message.joint_names, message.error.positions):
            if name not in self.max_tracking_error:
                continue
            if math.isfinite(error):
                self.max_tracking_error[name] = max(
                    self.max_tracking_error[name], abs(error))
            else:
                self.nonfinite += 1

    def wait_ready(self):
        if not self.motion.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('motion action unavailable')
        if not self.gripper.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('gripper action unavailable')
        if not self.brush.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('brush service unavailable')
        if not self.recover.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('recover_home service unavailable')

    def route(self, name):
        goal = ExecuteMotion.Goal()
        goal.route_name = name
        goal.speed_scale = self.speed_scale
        goal.dry_run = False
        sent = self.motion.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=8.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            raise RuntimeError(f'{name}: goal response timeout/rejected')
        result_future = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=45.0)
        if not result_future.done():
            sent.result().cancel_goal_async()
            raise RuntimeError(f'{name}: result timeout')
        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise RuntimeError(
                f'{name}: status={wrapped.status} code={result.error_code} '
                f'{result.message}')

    def grip(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['L_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        tolerance = JointTolerance()
        tolerance.name = 'L_finger_joint'
        tolerance.position = 0.003
        tolerance.velocity = 0.003
        goal.goal_tolerance = [tolerance]
        goal.goal_time_tolerance.sec = 2
        sent = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=5.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            raise RuntimeError(
                f'gripper {position}: goal response timeout/rejected')
        result_future = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=8.0)
        if not result_future.done():
            sent.result().cancel_goal_async()
            raise RuntimeError(f'gripper {position}: result timeout')
        wrapped = result_future.result()
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED or
            wrapped.result.error_code != 0
        ):
            raise RuntimeError(
                f'gripper {position}: status={wrapped.status} '
                f'code={wrapped.result.error_code} '
                f'{wrapped.result.error_string}')

    def set_brush(self, enabled):
        request = SetBrush.Request()
        request.enabled = enabled
        request.speed_percent = 100.0 if enabled else 0.0
        future = self.brush.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'brush {enabled}: response timeout')
        if not future.result().success:
            raise RuntimeError(f'brush {enabled}: {future.result().message}')

    def recover_home(self):
        future = self.recover.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=55.0)
        return (
            future.done() and future.result() is not None and
            future.result().success)

    def home_error(self):
        if any(name not in self.last_joint for name in JOINTS):
            return float('inf')
        return max(
            abs(self.last_joint[name] - target)
            for name, target in zip(JOINTS, HOME))

    def cycle(self):
        self.route('home_to_outlet_1_grasp_smooth')
        self.grip(0.0)
        self.route('outlet_1_grasp_to_spectrometer_place_continuous')
        self.grip(0.05)
        self.route('spectrometer_place_to_hover')
        self.route('spectrometer_hover_to_pick')
        self.grip(0.0)
        self.route('spectrometer_pick_to_brush_entry_continuous')
        self.set_brush(True)
        self.route('brush_entry_to_center')
        self.route('brush_center_to_entry')
        self.set_brush(False)
        self.route('brush_entry_to_outlet_1_return_continuous')
        self.grip(0.05)
        self.route('outlet_1_return_to_home_fast')
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.home_error() > 0.05:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.home_error() > 0.05:
            raise RuntimeError(
                f'Home encoder error {self.home_error():.6f}rad')
        if self.nonfinite:
            raise RuntimeError(
                f'non-finite telemetry samples={self.nonfinite}')
        if self.max_gap > 0.25:
            raise RuntimeError(
                f'joint-state sample gap {self.max_gap:.3f}s')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=50)
    parser.add_argument('--speed-scale', type=float, default=1.0)
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('--cycles must be positive')
    if not 0.1 <= args.speed_scale <= 1.0:
        parser.error('--speed-scale must be in [0.1, 1.0]')
    return args


def main():
    args = parse_args()
    rclpy.init()
    runner = StressRunner(args.speed_scale)
    passed = 0
    result = 0
    try:
        runner.wait_ready()
        for cycle in range(1, args.cycles + 1):
            started = time.monotonic()
            runner.reset_metrics()
            print(f'CYCLE_START {cycle}/{args.cycles}', flush=True)
            runner.cycle()
            passed += 1
            error_joint = max(
                JOINTS, key=lambda name: runner.max_tracking_error[name])
            rate_joint = max(
                JOINTS, key=lambda name: runner.max_observed_rate[name])
            print(
                f'CYCLE_PASS {cycle}/{args.cycles} '
                f'duration={time.monotonic() - started:.2f}s '
                f'samples={runner.samples} gap={runner.max_gap:.4f}s '
                f'home={runner.home_error():.5f}rad '
                f'track={error_joint}:'
                f'{runner.max_tracking_error[error_joint]:.5f}rad '
                f'state_rate={rate_joint}:'
                f'{runner.max_observed_rate[rate_joint]:.3f}rad/s',
                flush=True)
    except Exception as error:
        print(
            f'CYCLE_FAIL {passed + 1}/{args.cycles} '
            f'{type(error).__name__}: {error}',
            flush=True)
        try:
            runner.set_brush(False)
        except Exception as stop_error:
            print(f'BRUSH_STOP_FAIL {stop_error}', flush=True)
        recovered = runner.recover_home()
        print(
            f'SAFETY_RECOVERY home={recovered} '
            f'error={runner.home_error():.6f}rad',
            flush=True)
        result = 1
    finally:
        try:
            runner.set_brush(False)
        except Exception:
            pass
        print(
            f'STRESS_SUMMARY passed={passed}/{args.cycles} '
            f'home_error={runner.home_error():.6f}rad',
            flush=True)
        runner.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
