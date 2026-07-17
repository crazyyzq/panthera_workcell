#!/usr/bin/env python3
import json
import math
import os
import sys
import termios
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState, JointTolerance
from panthera_interfaces.action import ExecuteMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


HOME = [-0.006, 0.0, 0.012, -0.072, -0.006, 0.034]
ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
GRIPPER_JOINT = 'L_finger_joint'
MOTOR_DEVICE = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


class Cycle(Node):
    def __init__(self):
        super().__init__(f'industrial_full_cycle_{os.getpid()}')
        self.motion = ActionClient(self, ExecuteMotion, '/motion/execute')
        self.gripper = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.latest_joint_state = None
        self.route_samples = 0
        self.route_error_sq = 0.0
        self.route_error_count = 0
        self.route_max_error = 0.0
        self.route_max_reference_velocity = 0.0
        self.route_max_feedback_velocity = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state', self.on_controller_state, 50)
        self.motor_fd = None

    def on_joint_state(self, msg):
        self.latest_joint_state = msg

    def on_controller_state(self, msg):
        if msg.error.positions:
            self.route_samples += 1
            self.route_max_error = max(
                self.route_max_error, max(abs(v) for v in msg.error.positions))
            self.route_error_sq += sum(v * v for v in msg.error.positions)
            self.route_error_count += len(msg.error.positions)
        if msg.reference.velocities:
            self.route_max_reference_velocity = max(
                self.route_max_reference_velocity,
                max(abs(v) for v in msg.reference.velocities))
        if msg.feedback.velocities:
            self.route_max_feedback_velocity = max(
                self.route_max_feedback_velocity,
                max(abs(v) for v in msg.feedback.velocities))

    def spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.done()

    def route(self, name):
        self.route_samples = 0
        self.route_error_sq = 0.0
        self.route_error_count = 0
        self.route_max_error = 0.0
        self.route_max_reference_velocity = 0.0
        self.route_max_feedback_velocity = 0.0
        goal = ExecuteMotion.Goal()
        goal.route_name = name
        goal.speed_scale = 1.0
        goal.dry_run = False
        sent = self.motion.send_goal_async(goal)
        if not self.spin_until(sent, 5.0) or not sent.result() or not sent.result().accepted:
            raise RuntimeError(f'{name}: goal rejected')
        result_future = sent.result().get_result_async()
        if not self.spin_until(result_future, 120.0):
            sent.result().cancel_goal_async()
            raise RuntimeError(f'{name}: result timeout')
        result = result_future.result().result
        metrics = {
            'stage': name,
            'success': result.success,
            'elapsed_sec': round(result.elapsed_sec, 3),
            'max_error_rad': round(self.route_max_error, 4),
            'max_reference_rad_sec': round(self.route_max_reference_velocity, 4),
            'max_feedback_rad_sec': round(self.route_max_feedback_velocity, 4),
            'rms_error_rad': round(math.sqrt(
                self.route_error_sq / self.route_error_count)
                if self.route_error_count else 0.0, 4),
            'message': result.message,
        }
        print(json.dumps(metrics, ensure_ascii=False), flush=True)
        if not result.success:
            raise RuntimeError(f'{name}: {result.message}')

    def gripper_state(self):
        msg = self.latest_joint_state
        if msg is None or GRIPPER_JOINT not in msg.name:
            return None
        index = msg.name.index(GRIPPER_JOINT)
        if index >= len(msg.position) or index >= len(msg.velocity):
            return None
        return msg.position[index], msg.velocity[index]

    def command_gripper(self, target, label, allow_contact=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=0, nanosec=800_000_000)
        goal.trajectory.points = [point]
        if allow_contact:
            hold = JointTrajectoryPoint()
            hold.positions = [target]
            hold.velocities = [0.0]
            hold.time_from_start = Duration(sec=3600, nanosec=800_000_000)
            goal.trajectory.points.append(hold)
        tolerance = JointTolerance()
        tolerance.name = GRIPPER_JOINT
        tolerance.position = 0.040 if allow_contact else 0.003
        tolerance.velocity = 0.003
        goal.goal_tolerance = [tolerance]
        if allow_contact:
            path_tolerance = JointTolerance()
            path_tolerance.name = GRIPPER_JOINT
            path_tolerance.position = 0.10
            path_tolerance.velocity = 0.10
            goal.path_tolerance = [path_tolerance]
        goal.goal_time_tolerance = Duration(sec=2)
        sent = self.gripper.send_goal_async(goal)
        if not self.spin_until(sent, 3.0) or not sent.result() or not sent.result().accepted:
            raise RuntimeError(f'{label}: goal rejected')
        handle = sent.result()
        result_future = handle.get_result_async()
        contact_since = None
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            state = self.gripper_state()
            if state is not None:
                position, velocity = state
                if abs(position - target) <= 0.003 and abs(velocity) <= 0.003:
                    print(f'{label}: encoder target reached at {position:.4f}m', flush=True)
                    return
                contact = (
                    allow_contact and position <= 0.042 and position > 0.003 and
                    abs(velocity) <= 0.003)
                if contact:
                    now = time.monotonic()
                    contact_since = contact_since or now
                    if now - contact_since >= 0.08:
                        print(
                            f'{label}: stable cup contact at {position:.4f}m; close target retained',
                            flush=True)
                        return
                else:
                    contact_since = None
            if result_future.done():
                wrapped = result_future.result()
                if wrapped.status == 4 and wrapped.result.error_code == 0:
                    print(f'{label}: controller target reached', flush=True)
                    return
                if not allow_contact:
                    raise RuntimeError(
                        f'{label}: controller error {wrapped.result.error_string}')
        raise RuntimeError(f'{label}: encoder/contact timeout')

    def set_motor(self, enabled):
        if self.motor_fd is None:
            self.motor_fd = os.open(
                MOTOR_DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attr = termios.tcgetattr(self.motor_fd)
            attr[0] = attr[1] = attr[3] = 0
            attr[2] = (
                (attr[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) |
                termios.CS8 | termios.CLOCAL | termios.CREAD)
            attr[4] = attr[5] = termios.B115200
            attr[6][termios.VMIN] = 0
            attr[6][termios.VTIME] = 10
            termios.tcsetattr(self.motor_fd, termios.TCSANOW, attr)
        os.write(self.motor_fd, b'1' if enabled else b'0')
        termios.tcdrain(self.motor_fd)
        print(f'brush motor {"ON" if enabled else "OFF"}', flush=True)

    def recover_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = HOME
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = Duration(sec=10)
        goal.trajectory.points = [point]
        for name in ARM_JOINTS:
            path = JointTolerance(); path.name = name; path.position = 0.5
            final = JointTolerance(); final.name = name; final.position = 0.05
            goal.path_tolerance.append(path); goal.goal_tolerance.append(final)
        velocity = JointTolerance(); velocity.name = ''; velocity.velocity = 0.05
        goal.goal_tolerance.append(velocity)
        goal.goal_time_tolerance = Duration(sec=5)
        sent = self.arm.send_goal_async(goal)
        if not self.spin_until(sent, 3.0) or not sent.result() or not sent.result().accepted:
            raise RuntimeError('automatic Home recovery rejected; keep enabled')
        result = sent.result().get_result_async()
        if not self.spin_until(result, 17.0):
            raise RuntimeError('automatic Home recovery timeout; keep enabled')
        wrapped = result.result()
        if wrapped.status != 4 or wrapped.result.error_code != 0:
            raise RuntimeError('automatic Home recovery failed; keep enabled')
        print('automatic Home recovery completed', flush=True)

    def close(self):
        if self.motor_fd is not None:
            try:
                self.set_motor(False)
            finally:
                os.close(self.motor_fd)
                self.motor_fd = None


def main():
    rclpy.init()
    node = Cycle()
    try:
        if not node.motion.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('motion server unavailable')
        if not node.gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('gripper server unavailable')
        if not node.arm.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('arm controller unavailable')
        node.command_gripper(0.05, 'outlet open')
        node.route('home_to_outlet_1_grasp_smooth')
        node.command_gripper(0.0, 'outlet grasp', allow_contact=True)
        node.route('outlet_1_grasp_to_spectrometer_place_continuous')
        node.command_gripper(0.05, 'spectrometer release')
        node.route('spectrometer_place_to_pick_smooth')
        node.command_gripper(0.0, 'spectrometer grasp', allow_contact=True)
        # The server concatenates independently limited phases into one
        # controller goal. This removes action round-trip dwell without letting
        # the wrist reversal slow the pickup and lift phases.
        node.route('spectrometer_pick_to_brush_entry_continuous')
        node.set_motor(True)
        node.route('brush_entry_to_center')
        time.sleep(2.0)
        node.route('brush_center_to_entry')
        node.set_motor(False)
        node.route('brush_entry_to_outlet_1_return_smooth')
        node.command_gripper(0.05, 'outlet return release')
        node.route('outlet_1_return_to_home_fast')
        print('FULL_CYCLE_SUCCESS', flush=True)
    except Exception as exc:
        print(f'FULL_CYCLE_ERROR: {exc}', file=sys.stderr, flush=True)
        try:
            node.set_motor(False)
        except Exception as motor_exc:
            print(f'motor stop error: {motor_exc}', file=sys.stderr, flush=True)
        node.recover_home()
        raise
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
