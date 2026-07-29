import json
import hmac
import mimetypes
import os
import math
import re
import shutil
import struct
import subprocess
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml
from ament_index_python.packages import get_package_share_directory
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None
import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from panthera_interfaces.action import ExecuteMotion
from panthera_interfaces.msg import ExternalSignal, LaserDistance, WorkflowStatus
from panthera_interfaces.srv import RunWorkflow, SetBrush, SetSpeedScale, StageJog


DEBUG_POSITION_TOLERANCE_M = 0.0005

EXTERNAL_COMMANDS = {
    'PING': None,
    'OUTLET_1_DISCHARGE_DONE': 'actual_cycle_outlet_1',
    'OUTLET_2_DISCHARGE_DONE': 'actual_cycle_outlet_2',
    'DETECTION_DONE': 'detection_done',
}
EXTERNAL_REQUEST_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._:-]{0,63}$')
EXTERNAL_JOURNAL_LIMIT = 256


POINT_CONFIG_MOTION_KEYS = {
    'outlet_approach_y',
    'outlet_near_y',
    'outlet_high_z',
    'outlet_grip_z',
    'outlet_transfer_z',
    'spectrometer_approach_x_offset',
    'spectrometer_high_z',
    'clean_high_z',
    'clean_approach_y',
    'clean_pre_z',
    'clean_ready_z',
}

POINT_CONFIG_AXIS_KEYS = {
    'laser_min_mm',
    'laser_max_mm',
    'axis_zero_laser_mm',
    'axis_scale_m_per_mm',
    'axis',
    'place_offset_xyz',
    'pick_offset_xyz',
}

POINT_CONFIG_CLEANING_KEYS = {
    'pour_wrist_joint_index',
    'pour_direction',
    'pour_angle_rad',
    'pour_velocity_scale',
    'pour_acceleration_scale',
    'pour_hold_sec',
    'shake_count',
    'shake_angle_rad',
    'shake_hold_sec',
    'brush_enabled',
    'brush_pose',
    'brush_velocity_scale',
    'brush_acceleration_scale',
    'brush_approach_offset_xyz',
    'brush_upright_retreat_offset_xyz',
    'brush_stroke_count',
    'brush_stroke_offset_xyz',
    'brush_hold_sec',
    'brush_motor_stop_delay_sec',
    'motor_rs485_enabled',
    'motor_rs485_device',
    'motor_rs485_baudrate',
    'motor_rs485_slave_id',
    'motor_rs485_duty_permille',
    'motor_rs485_communication_timeout_ds',
}


def now_sec():
    return time.time()


def _parse_yaml_scalar(value):
    value = value.strip()
    if value in ('true', 'false'):
        return value == 'true'
    if value.startswith('[') and value.endswith(']'):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_yaml_scalar(item.strip()) for item in inner.split(',')]
    if re.fullmatch(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', value):
        number = float(value)
        if re.fullmatch(r'[-+]?\d+', value):
            return int(number)
        return number
    return value


def _parse_simple_yaml(text):
    root = {}
    stack = [(-1, root)]
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith('#'):
            continue
        indent = len(raw) - len(raw.lstrip(' '))
        line = raw.strip()
        match = re.match(r'^([^:#][^:]*):(?:\s*(.*))?$', line)
        if not match:
            continue
        key = match.group(1).strip()
        rest = (match.group(2) or '').strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            continue
        parent = stack[-1][1]
        if rest == '':
            parent[key] = {}
            stack.append((indent, parent[key]))
        else:
            parent[key] = _parse_yaml_scalar(rest)
    return root


def _format_yaml_scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_format_yaml_number(float(item)) for item in value) + ']'
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return _format_yaml_number(value)
    return str(value)


def _format_yaml_number(value):
    if not math.isfinite(float(value)):
        raise ValueError('number must be finite')
    text = f'{float(value):.6f}'.rstrip('0').rstrip('.')
    if text == '-0':
        return '0'
    if '.' not in text and 'e' not in text.lower():
        text += '.0'
    return text


def _replace_yaml_value(text, path, value):
    lines = text.splitlines()
    indices = []
    start = 0
    parent_indent = -1
    for depth, key in enumerate(path):
        pattern = re.compile(rf'^(\s*){re.escape(key)}:\s*(.*)$')
        found = None
        for idx in range(start, len(lines)):
            raw = lines[idx]
            stripped = raw.strip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(raw) - len(raw.lstrip(' '))
            if depth > 0 and indent <= parent_indent:
                break
            match = pattern.match(raw)
            if match and indent > parent_indent:
                found = (idx, indent)
                break
        if found is None:
            raise KeyError('.'.join(path))
        indices.append(found[0])
        parent_indent = found[1]
        start = found[0] + 1

    idx = indices[-1]
    indent = lines[idx][:len(lines[idx]) - len(lines[idx].lstrip(' '))]
    original_inline_value = lines[idx].partition(':')[2].strip()
    sequence_indent = indent
    if idx + 1 < len(lines) and lines[idx + 1].lstrip(' ').startswith('- '):
        sequence_indent = lines[idx + 1][
            :len(lines[idx + 1]) - len(lines[idx + 1].lstrip(' '))]
    end = idx + 1
    while end < len(lines):
        raw = lines[end]
        if raw.strip():
            child_indent = len(raw) - len(raw.lstrip(' '))
            indentless_sequence = (
                child_indent == len(indent) and raw.lstrip(' ').startswith('- '))
            if child_indent <= len(indent) and not indentless_sequence:
                break
        end += 1
    if isinstance(value, (list, tuple)) and not original_inline_value:
        replacement = [f'{indent}{path[-1]}:']
        replacement.extend(
            f'{sequence_indent}- {_format_yaml_number(float(item))}'
            for item in value)
    else:
        replacement = [f'{indent}{path[-1]}: {_format_yaml_scalar(value)}']
    lines[idx:end] = replacement
    return '\n'.join(lines) + '\n'


def _as_float_list(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{name} must be a {length}-element array')
    result = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f'{name} contains non-finite value')
        result.append(number)
    return result


def _as_bool(value, name):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('true', '1', 'yes', 'on'):
            return True
        if normalized in ('false', '0', 'no', 'off'):
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    raise ValueError(f'{name} must be boolean')


def _validate_motion_catalog_document(catalog):
    if not isinstance(catalog, dict):
        raise ValueError('motion catalog must be an object')
    if int(catalog.get('schema_version', 0)) != 1:
        raise ValueError('motion catalog schema_version must be 1')

    robot = catalog.get('robot')
    points = catalog.get('points')
    routes = catalog.get('routes')
    if not isinstance(robot, dict):
        raise ValueError('motion catalog robot must be an object')
    if not isinstance(points, dict) or not points:
        raise ValueError('motion catalog points must be a non-empty object')
    if not isinstance(routes, dict):
        raise ValueError('motion catalog routes must be an object')

    joint_names = robot.get('joint_names')
    if not isinstance(joint_names, list) or not joint_names:
        raise ValueError('robot.joint_names must be a non-empty array')
    id_pattern = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')

    for name, point in points.items():
        if not id_pattern.fullmatch(str(name)):
            raise ValueError(f'invalid point id: {name}')
        if not isinstance(point, dict):
            raise ValueError(f'points.{name} must be an object')
        has_joints = 'joints' in point
        has_pose = 'pose' in point
        if not has_joints and not has_pose:
            raise ValueError(f'points.{name} needs joints or pose')
        if has_joints:
            _as_float_list(point['joints'], len(joint_names), f'points.{name}.joints')
        if has_pose:
            pose = point['pose']
            if not isinstance(pose, dict):
                raise ValueError(f'points.{name}.pose must be an object')
            _as_float_list(pose.get('xyz'), 3, f'points.{name}.pose.xyz')
            _as_float_list(pose.get('rpy'), 3, f'points.{name}.pose.rpy')
        seed = str(point.get('ik_seed', '')).strip()
        if seed and seed not in points:
            raise ValueError(f'points.{name}.ik_seed references missing point: {seed}')

    references = {name: [] for name in points}
    for name, route in routes.items():
        if not id_pattern.fullmatch(str(name)):
            raise ValueError(f'invalid route id: {name}')
        if not isinstance(route, dict):
            raise ValueError(f'routes.{name} must be an object')
        start = str(route.get('start', '')).strip()
        if start not in points:
            raise ValueError(f'routes.{name}.start references missing point: {start}')
        references[start].append(f'routes.{name}.start')
        segments = route.get('segments')
        if not isinstance(segments, list) or not segments:
            raise ValueError(f'routes.{name}.segments must be a non-empty array')
        for index, segment in enumerate(segments):
            if not isinstance(segment, dict):
                raise ValueError(f'routes.{name}.segments[{index}] must be an object')
            segment_type = str(segment.get('type', 'joint'))
            if segment_type not in ('joint', 'linear'):
                raise ValueError(
                    f'routes.{name}.segments[{index}].type must be joint or linear')
            target = str(segment.get('to', '')).strip()
            if target not in points:
                raise ValueError(
                    f'routes.{name}.segments[{index}].to references missing point: {target}')
            if segment_type == 'linear' and 'pose' not in points[target]:
                raise ValueError(
                    f'linear target points.{target} must contain a pose')
            references[target].append(f'routes.{name}.segments[{index}].to')

    commissioning = catalog.get('commissioning', {})
    if commissioning is not None and not isinstance(commissioning, dict):
        raise ValueError('commissioning must be an object')
    safe_point = str((commissioning or {}).get('safe_point', '')).strip()
    if safe_point and safe_point not in points:
        raise ValueError(f'commissioning.safe_point references missing point: {safe_point}')
    followers = (commissioning or {}).get('translation_followers', {})
    if not isinstance(followers, dict):
        raise ValueError('commissioning.translation_followers must be an object')
    for target, entries in followers.items():
        if target not in points or not isinstance(entries, list):
            raise ValueError(f'invalid commissioning followers for {target}')
        for entry in entries:
            if not isinstance(entry, dict) or entry.get('point') not in points:
                raise ValueError(f'invalid commissioning follower for {target}: {entry}')
            axes = str(entry.get('axes', ''))
            if not axes or any(axis not in 'xyz' for axis in axes) or len(set(axes)) != len(axes):
                raise ValueError(f'invalid commissioning follower axes for {target}: {axes}')
    return references


class HmiStateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._tool_pose_candidate = None
        self._tool_pose_candidate_count = 0
        self._tool_pose_reject_count = 0
        self._data = {
            'server': {
                'started_at': now_sec(),
                'time': now_sec(),
            },
            'spectrometer_cell': {
                'state': 'UNKNOWN',
                'state_age_sec': None,
                'context': {},
                'context_raw': '',
                'error': '',
            },
            'laser': {
                'valid': False,
                'distance_mm': None,
                'distance_m': None,
                'source': '',
                'status': 'no data',
                'age_sec': None,
            },
            'workflow': {
                'name': '',
                'step': '',
                'step_index': 0,
                'step_count': 0,
                'state': 0,
                'message': '',
                'age_sec': None,
            },
            'external_signal': {
                'source': '',
                'name': '',
                'code': 0,
                'active': False,
                'workflow_name': '',
                'detail': '',
                'age_sec': None,
            },
            'joint_state': {
                'names': [],
                'positions': [],
                'velocities': [],
                'efforts': [],
                'age_sec': None,
            },
            'tool_pose': {
                'available': False,
                'base_frame': 'base_link',
                'tool_frame': 'gripper_center',
                'position_m': {'x': None, 'y': None, 'z': None},
                'position_mm': {'x': None, 'y': None, 'z': None},
                'quaternion_xyzw': {'x': None, 'y': None, 'z': None, 'w': None},
                'rpy_rad': {'roll': None, 'pitch': None, 'yaw': None},
                'rpy_deg': {'roll': None, 'pitch': None, 'yaw': None},
                'stamp_sec': None,
                'status': 'no tf',
                'filter_reject_count': 0,
                'age_sec': None,
            },
            'camera': {
                'rgb': {
                    'topic': '',
                    'available': False,
                    'width': 0,
                    'height': 0,
                    'encoding': '',
                    'age_sec': None,
                    'url': '/api/camera/rgb_stream.mjpg',
                },
                'depth': {
                    'topic': '',
                    'available': False,
                    'width': 0,
                    'height': 0,
                    'encoding': '',
                    'age_sec': None,
                    'url': '/api/camera/depth',
                },
            },
            'services': {},
            'motion': {
                'speed_scale': 1.0,
                'speed_percent': 100,
            },
        }
        self._timestamps = {}

    def set_cell_state(self, state):
        with self._lock:
            self._data['spectrometer_cell']['state'] = state
            self._timestamps['cell_state'] = now_sec()

    def set_cell_context(self, raw):
        context = {}
        try:
            context = json.loads(raw)
        except json.JSONDecodeError:
            context = {'raw': raw}

        with self._lock:
            self._data['spectrometer_cell']['context'] = context
            self._data['spectrometer_cell']['context_raw'] = raw
            self._timestamps['cell_context'] = now_sec()
            if isinstance(context, dict) and context.get('state'):
                self._data['spectrometer_cell']['state'] = context['state']
                self._timestamps['cell_state'] = now_sec()

    def set_cell_error(self, error):
        with self._lock:
            self._data['spectrometer_cell']['error'] = error
            self._timestamps['cell_error'] = now_sec()

    def set_laser(self, msg):
        with self._lock:
            self._data['laser'] = {
                'valid': bool(msg.valid),
                'distance_mm': float(msg.distance_mm),
                'distance_m': float(msg.distance_m),
                'source': msg.source,
                'status': msg.status,
                'age_sec': 0.0,
            }
            self._timestamps['laser'] = now_sec()

    def set_workflow(self, msg):
        with self._lock:
            self._data['workflow'] = {
                'name': msg.workflow_name,
                'step': msg.current_step,
                'step_index': int(msg.step_index),
                'step_count': int(msg.step_count),
                'state': int(msg.state),
                'message': msg.message,
                'age_sec': 0.0,
            }
            self._timestamps['workflow'] = now_sec()

    def set_external_signal(self, msg):
        with self._lock:
            self._data['external_signal'] = {
                'source': msg.source,
                'name': msg.name,
                'code': int(msg.code),
                'active': bool(msg.active),
                'workflow_name': msg.workflow_name,
                'detail': msg.detail,
                'age_sec': 0.0,
            }
            self._timestamps['external_signal'] = now_sec()

    def set_joint_state(self, msg):
        with self._lock:
            self._data['joint_state'] = {
                'names': list(msg.name),
                'positions': [float(v) for v in msg.position],
                'velocities': [float(v) for v in msg.velocity],
                'efforts': [float(v) for v in msg.effort],
                'age_sec': 0.0,
            }
            self._timestamps['joint_state'] = now_sec()

    def set_tool_pose(
            self,
            base_frame,
            tool_frame,
            transform,
            max_jump_m=None,
            max_quat_jump=None,
            accept_after_count=3):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rpy = quaternion_to_rpy(rotation.x, rotation.y, rotation.z, rotation.w)
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        current_pose = {
            'position_m': {
                'x': float(translation.x),
                'y': float(translation.y),
                'z': float(translation.z),
            },
            'quaternion_xyzw': {
                'x': float(rotation.x),
                'y': float(rotation.y),
                'z': float(rotation.z),
                'w': float(rotation.w),
            },
        }
        with self._lock:
            if self._should_reject_tool_pose_locked(
                    current_pose,
                    max_jump_m,
                    max_quat_jump,
                    accept_after_count):
                self._timestamps['tool_pose'] = now_sec()
                self._data['tool_pose'].update({
                    'available': True,
                    'base_frame': base_frame,
                    'tool_frame': tool_frame,
                    'status': 'filtered_spike',
                    'filter_reject_count': self._tool_pose_reject_count,
                    'age_sec': 0.0,
                })
                return

            self._tool_pose_candidate = None
            self._tool_pose_candidate_count = 0
            self._data['tool_pose'] = {
                'available': True,
                'base_frame': base_frame,
                'tool_frame': tool_frame,
                'position_m': current_pose['position_m'],
                'position_mm': {
                    'x': float(translation.x) * 1000.0,
                    'y': float(translation.y) * 1000.0,
                    'z': float(translation.z) * 1000.0,
                },
                'quaternion_xyzw': current_pose['quaternion_xyzw'],
                'rpy_rad': {
                    'roll': rpy[0],
                    'pitch': rpy[1],
                    'yaw': rpy[2],
                },
                'rpy_deg': {
                    'roll': math.degrees(rpy[0]),
                    'pitch': math.degrees(rpy[1]),
                    'yaw': math.degrees(rpy[2]),
                },
                'stamp_sec': stamp,
                'status': 'ok',
                'filter_reject_count': self._tool_pose_reject_count,
                'age_sec': 0.0,
            }
            self._timestamps['tool_pose'] = now_sec()

    def _should_reject_tool_pose_locked(
            self,
            current_pose,
            max_jump_m,
            max_quat_jump,
            accept_after_count):
        previous = self._data.get('tool_pose', {})
        if not previous.get('available'):
            return False
        prev_pos = previous.get('position_m') or {}
        prev_quat = previous.get('quaternion_xyzw') or {}
        if any(prev_pos.get(axis) is None for axis in ('x', 'y', 'z')):
            return False
        if any(prev_quat.get(axis) is None for axis in ('x', 'y', 'z', 'w')):
            return False

        pos = current_pose['position_m']
        quat = current_pose['quaternion_xyzw']
        dpos = math.sqrt(sum((pos[axis] - prev_pos[axis]) ** 2 for axis in ('x', 'y', 'z')))
        dquat = math.sqrt(sum((quat[axis] - prev_quat[axis]) ** 2 for axis in ('x', 'y', 'z', 'w')))
        if (max_jump_m is None or dpos <= max_jump_m) and (
                max_quat_jump is None or dquat <= max_quat_jump):
            return False

        if self._tool_pose_candidate is not None:
            cand_pos = self._tool_pose_candidate['position_m']
            cand_quat = self._tool_pose_candidate['quaternion_xyzw']
            dcand_pos = math.sqrt(
                sum((pos[axis] - cand_pos[axis]) ** 2 for axis in ('x', 'y', 'z')))
            dcand_quat = math.sqrt(
                sum((quat[axis] - cand_quat[axis]) ** 2 for axis in ('x', 'y', 'z', 'w')))
            if dcand_pos <= (max_jump_m or 0.0) and dcand_quat <= (max_quat_jump or 0.0):
                self._tool_pose_candidate_count += 1
            else:
                self._tool_pose_candidate = current_pose
                self._tool_pose_candidate_count = 1
        else:
            self._tool_pose_candidate = current_pose
            self._tool_pose_candidate_count = 1

        if self._tool_pose_candidate_count >= max(1, int(accept_after_count)):
            return False

        self._tool_pose_reject_count += 1
        return True

    def set_tool_pose_error(self, base_frame, tool_frame, status):
        with self._lock:
            self._data['tool_pose'].update({
                'available': False,
                'base_frame': base_frame,
                'tool_frame': tool_frame,
                'status': status,
            })

    def set_camera_topic(self, channel, topic):
        with self._lock:
            self._data['camera'][channel]['topic'] = topic

    def set_camera_info(self, channel, msg):
        with self._lock:
            self._data['camera'][channel]['width'] = int(msg.width)
            self._data['camera'][channel]['height'] = int(msg.height)

    def set_camera_image(self, channel, msg):
        with self._lock:
            self._data['camera'][channel].update({
                'available': True,
                'width': int(msg.width),
                'height': int(msg.height),
                'encoding': msg.encoding,
                'age_sec': 0.0,
            })
            self._timestamps[f'camera_{channel}'] = now_sec()

    def set_camera_compressed(self, channel, msg):
        with self._lock:
            self._data['camera'][channel].update({
                'available': True,
                'encoding': msg.format or 'compressed',
                'age_sec': 0.0,
            })
            self._timestamps[f'camera_{channel}'] = now_sec()

    def set_services(self, services):
        with self._lock:
            self._data['services'] = dict(services)

    def set_motion_speed_scale(self, scale):
        scale = max(0.20, min(1.20, float(scale)))
        with self._lock:
            self._data['motion']['speed_scale'] = scale
            self._data['motion']['speed_percent'] = int(round(scale * 100.0))

    def snapshot(self):
        with self._lock:
            data = json.loads(json.dumps(self._data))
            current = now_sec()
            data['server']['time'] = current

            age_targets = {
                ('spectrometer_cell', 'state_age_sec'): 'cell_state',
                ('laser', 'age_sec'): 'laser',
                ('workflow', 'age_sec'): 'workflow',
                ('external_signal', 'age_sec'): 'external_signal',
                ('joint_state', 'age_sec'): 'joint_state',
                ('tool_pose', 'age_sec'): 'tool_pose',
                ('camera', 'rgb_age_sec'): 'camera_rgb',
                ('camera', 'depth_age_sec'): 'camera_depth',
            }
            for (section, field), key in age_targets.items():
                if key in self._timestamps:
                    if section == 'camera' and field == 'rgb_age_sec':
                        data['camera']['rgb']['age_sec'] = current - self._timestamps[key]
                    elif section == 'camera' and field == 'depth_age_sec':
                        data['camera']['depth']['age_sec'] = current - self._timestamps[key]
                    else:
                        data[section][field] = current - self._timestamps[key]
            return data


def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_product(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def cartesian_pose_target(current_xyz, current_rpy, deltas):
    target_xyz = [current + delta for current, delta in zip(current_xyz, deltas[:3])]
    target_quaternion = _quaternion_product(
        rpy_to_quaternion(deltas[3:]),
        rpy_to_quaternion(current_rpy))
    target_rpy = quaternion_to_rpy(*target_quaternion)
    return target_xyz, list(target_rpy)


def cartesian_pose_delta(current_xyz, current_rpy, target_xyz, target_rpy):
    translation = [target - current for current, target in zip(current_xyz, target_xyz)]
    current_quaternion = rpy_to_quaternion(current_rpy)
    target_quaternion = rpy_to_quaternion(target_rpy)
    inverse_current = (
        -current_quaternion[0],
        -current_quaternion[1],
        -current_quaternion[2],
        current_quaternion[3],
    )
    rotation = quaternion_to_rpy(
        *_quaternion_product(target_quaternion, inverse_current))
    return translation + list(rotation)


def compensated_command_target(
        measured_xyz, measured_rpy, commanded_xyz, commanded_rpy,
        physical_target_xyz, physical_target_rpy):
    physical_delta = cartesian_pose_delta(
        measured_xyz, measured_rpy, physical_target_xyz, physical_target_rpy)
    return cartesian_pose_target(commanded_xyz, commanded_rpy, physical_delta)


def rpy_orientation_error(first, second):
    first_quaternion = rpy_to_quaternion(first)
    second_quaternion = rpy_to_quaternion(second)
    dot = abs(sum(a * b for a, b in zip(first_quaternion, second_quaternion)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


class CameraImageStore:
    def __init__(self, max_width=480, target_fps=2.0, worker_count=2):
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._messages = {}
        self._payloads = {}
        self._pending_raw = {}
        self._last_convert_time = {}
        self._stats = {}
        self._shutdown = False
        self.max_width = max_width
        self.min_convert_interval = 1.0 / max(0.2, float(target_fps))
        self.worker_count = max(1, int(worker_count))
        self._workers = [
            threading.Thread(
                target=self._raw_worker,
                name=f'camera-image-worker-{idx + 1}',
                daemon=True,
            )
            for idx in range(self.worker_count)
        ]
        for worker in self._workers:
            worker.start()

    def _record_locked(self, channel, source, content_type=None, detail=None):
        key = f'{channel}_{source}'
        stat = self._stats.setdefault(key, {
            'count': 0,
            'last_age_sec': None,
            'last_content_type': '',
            'last_detail': '',
        })
        stat['count'] += 1
        stat['last_time'] = now_sec()
        if content_type:
            stat['last_content_type'] = content_type
        if detail is not None:
            stat['last_detail'] = str(detail)[:240]

    def set_raw(self, channel, msg):
        current = now_sec()
        with self._condition:
            self._messages[channel] = msg
            self._record_locked(channel, 'raw_received')
            cached = self._payloads.get(channel)
            if (
                channel == 'rgb'
                and cached is not None
                and cached[1] in ('image/jpeg', 'image/png')
                and current - cached[2] < 2.0
            ):
                return
            last = self._last_convert_time.get(channel, 0.0)
            if current - last < self.min_convert_interval:
                return
            self._last_convert_time[channel] = current
            self._pending_raw[channel] = (msg, current)
            self._record_locked(channel, 'raw_queued')
            self._condition.notify_all()

    def set_compressed(self, channel, msg):
        fmt = (msg.format or '').lower()
        content_type = 'image/png' if 'png' in fmt else 'image/jpeg'
        with self._condition:
            self._payloads[channel] = (bytes(msg.data), content_type, now_sec())
            self._record_locked(channel, 'compressed_received', content_type)
            self._condition.notify_all()

    def get_payload(self, channel):
        with self._lock:
            payload = self._payloads.get(channel)
            msg = self._messages.get(channel)
        if payload is not None:
            return payload[0], payload[1], None

        if msg is None:
            return None, 'image/bmp', 'no image received'

        return None, 'image/bmp', 'image conversion pending'

    def wait_for_payload(self, channel, last_stamp, timeout=1.0):
        deadline = now_sec() + timeout
        with self._condition:
            while True:
                payload = self._payloads.get(channel)
                if payload is not None and payload[2] != last_stamp:
                    return payload[0], payload[1], payload[2], None
                remaining = deadline - now_sec()
                if remaining <= 0.0:
                    if payload is not None:
                        return payload[0], payload[1], payload[2], None
                    return None, 'image/bmp', last_stamp, 'no image received'
                self._condition.wait(timeout=remaining)

    def _raw_worker(self):
        while True:
            with self._condition:
                while not self._shutdown and not self._pending_raw:
                    self._condition.wait()
                if self._shutdown:
                    return
                channel, (msg, queued_at) = self._pending_raw.popitem()
                self._record_locked(channel, 'raw_worker_started')

            try:
                if channel == 'depth':
                    payload = image_to_depth_bmp(msg, self.max_width)
                else:
                    payload = image_to_color_bmp(msg, self.max_width)
                content_type = 'image/bmp'
                detail = f'latency_sec={now_sec() - queued_at:.3f}'
            except Exception as exc:
                payload = _placeholder_bmp(
                    int(getattr(msg, 'width', self.max_width)),
                    int(getattr(msg, 'height', 360)),
                    f'image conversion failed: {exc}',
                )
                content_type = 'image/bmp'
                detail = exc

            with self._condition:
                self._payloads[channel] = (payload, content_type, now_sec())
                self._record_locked(channel, 'raw_payload', content_type, detail)
                self._condition.notify_all()

    def stats(self):
        with self._lock:
            current = now_sec()
            result = {}
            for key, stat in self._stats.items():
                result[key] = {
                    'count': int(stat.get('count', 0)),
                    'last_age_sec': (
                        current - stat['last_time']
                        if 'last_time' in stat else None
                    ),
                    'last_content_type': stat.get('last_content_type', ''),
                    'last_detail': stat.get('last_detail', ''),
                }
            for channel, payload in self._payloads.items():
                result[f'{channel}_active_payload'] = {
                    'last_age_sec': current - payload[2],
                    'content_type': payload[1],
                    'bytes': len(payload[0]),
                }
            result['raw_worker_pool'] = {
                'workers': self.worker_count,
                'alive': sum(1 for worker in self._workers if worker.is_alive()),
                'pending_channels': sorted(self._pending_raw.keys()),
            }
            return result

    def shutdown(self):
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
        for worker in self._workers:
            worker.join(timeout=1.0)


def _scaled_dimensions(width, height, max_width):
    if max_width <= 0 or width <= max_width:
        return width, height, 1
    stride = max(1, int((width + max_width - 1) / max_width))
    return max(1, width // stride), max(1, height // stride), stride


def _pack_bmp(width, height, rgb_rows_top_down):
    row_size = width * 3
    padding = (4 - (row_size % 4)) % 4
    pixel_size = (row_size + padding) * height
    file_size = 14 + 40 + pixel_size

    header = bytearray()
    header.extend(b'BM')
    header.extend(struct.pack('<IHHI', file_size, 0, 0, 54))
    header.extend(struct.pack('<IiiHHIIiiII', 40, width, height, 1, 24, 0, pixel_size, 2835, 2835, 0, 0))

    payload = bytearray(header)
    pad = b'\x00' * padding
    for row in reversed(rgb_rows_top_down):
        # BMP stores BGR bytes.
        for r, g, b in row:
            payload.extend((b & 0xFF, g & 0xFF, r & 0xFF))
        payload.extend(pad)

    return bytes(payload)


def _numpy_rgb_to_bmp(rgb):
    if cv2 is None or np is None:
        return None
    if rgb.size == 0:
        return None
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    ok, encoded = cv2.imencode('.bmp', bgr)
    if not ok:
        return None
    return encoded.tobytes()


def _gray_to_rgb(gray):
    gray = max(0, min(255, int(gray)))
    return gray, gray, gray


def _depth_color(norm):
    norm = max(0.0, min(1.0, float(norm)))
    if norm < 0.25:
        t = norm / 0.25
        return 0, int(255 * t), 255
    if norm < 0.50:
        t = (norm - 0.25) / 0.25
        return 0, 255, int(255 * (1.0 - t))
    if norm < 0.75:
        t = (norm - 0.50) / 0.25
        return int(255 * t), 255, 0
    t = (norm - 0.75) / 0.25
    return 255, int(255 * (1.0 - t)), 0


def image_to_color_bmp(msg, max_width=640):
    width = int(msg.width)
    height = int(msg.height)
    out_w, out_h, stride = _scaled_dimensions(width, height, max_width)
    data = bytes(msg.data)
    encoding = msg.encoding.lower()

    fast_payload = _image_to_color_bmp_fast(msg, out_w, out_h, stride, encoding)
    if fast_payload is not None:
        return fast_payload

    rows = []

    if encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        channels = 4 if 'a8' in encoding else 3
        for y in range(0, height, stride):
            if len(rows) >= out_h:
                break
            row = []
            base = y * int(msg.step)
            for x in range(0, width, stride):
                if len(row) >= out_w:
                    break
                idx = base + x * channels
                if idx + channels > len(data):
                    row.append((0, 0, 0))
                    continue
                if encoding.startswith('rgb'):
                    row.append((data[idx], data[idx + 1], data[idx + 2]))
                else:
                    row.append((data[idx + 2], data[idx + 1], data[idx]))
            rows.append(row)
        return _pack_bmp(out_w, len(rows), rows)

    if encoding in ('mono8', '8uc1'):
        for y in range(0, height, stride):
            if len(rows) >= out_h:
                break
            row = []
            base = y * int(msg.step)
            for x in range(0, width, stride):
                if len(row) >= out_w:
                    break
                idx = base + x
                gray = data[idx] if idx < len(data) else 0
                row.append(_gray_to_rgb(gray))
            rows.append(row)
        return _pack_bmp(out_w, len(rows), rows)

    return _placeholder_bmp(out_w, out_h, f'unsupported color encoding: {msg.encoding}')


def _image_to_color_bmp_fast(msg, out_w, out_h, stride, encoding):
    if cv2 is None or np is None:
        return None

    width = int(msg.width)
    height = int(msg.height)
    data = msg.data

    if encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        channels = 4 if 'a8' in encoding else 3
        row_bytes = width * channels
        if int(msg.step) < row_bytes:
            return None
        raw = np.frombuffer(data, dtype=np.uint8)
        needed = int(msg.step) * height
        if raw.size < needed:
            return None
        rows = raw[:needed].reshape(height, int(msg.step))
        image = rows[:, :row_bytes].reshape(height, width, channels)
        sample = image[::stride, ::stride][:out_h, :out_w, :3]
        if encoding.startswith('bgr'):
            sample = sample[:, :, ::-1]
        return _numpy_rgb_to_bmp(np.ascontiguousarray(sample))

    if encoding in ('mono8', '8uc1'):
        if int(msg.step) < width:
            return None
        raw = np.frombuffer(data, dtype=np.uint8)
        needed = int(msg.step) * height
        if raw.size < needed:
            return None
        rows = raw[:needed].reshape(height, int(msg.step))
        sample = rows[:, :width][::stride, ::stride][:out_h, :out_w]
        rgb = np.repeat(sample[:, :, None], 3, axis=2)
        return _numpy_rgb_to_bmp(rgb)

    if encoding in ('yuyv', 'yuyv422', 'yuv422', 'yuv422_yuy2', 'yuv422_yuyv'):
        row_bytes = width * 2
        if int(msg.step) < row_bytes:
            return None
        raw = np.frombuffer(data, dtype=np.uint8)
        needed = int(msg.step) * height
        if raw.size < needed:
            return None
        rows = raw[:needed].reshape(height, int(msg.step))
        image = rows[:, :row_bytes].reshape(height, width, 2)
        rgb = cv2.cvtColor(image, cv2.COLOR_YUV2RGB_YUY2)
        sample = rgb[::stride, ::stride][:out_h, :out_w]
        return _numpy_rgb_to_bmp(np.ascontiguousarray(sample))

    return None


def image_to_depth_bmp(msg, max_width=640):
    width = int(msg.width)
    height = int(msg.height)
    out_w, out_h, stride = _scaled_dimensions(width, height, max_width)
    data = bytes(msg.data)
    encoding = msg.encoding.lower()

    fast_payload = _image_to_depth_bmp_fast(msg, out_w, out_h, stride, encoding)
    if fast_payload is not None:
        return fast_payload

    values = []
    rows_values = []

    for y in range(0, height, stride):
        if len(rows_values) >= out_h:
            break
        row = []
        base = y * int(msg.step)
        for x in range(0, width, stride):
            if len(row) >= out_w:
                break
            value = 0.0
            if encoding in ('16uc1', 'mono16'):
                idx = base + x * 2
                if idx + 2 <= len(data):
                    value = int.from_bytes(data[idx:idx + 2], 'big' if msg.is_bigendian else 'little')
            elif encoding in ('32fc1',):
                idx = base + x * 4
                if idx + 4 <= len(data):
                    value = struct.unpack('>f' if msg.is_bigendian else '<f', data[idx:idx + 4])[0]
            elif encoding in ('mono8', '8uc1'):
                idx = base + x
                if idx < len(data):
                    value = data[idx]
            else:
                return _placeholder_bmp(out_w, out_h, f'unsupported depth encoding: {msg.encoding}')

            if value > 0 and value == value:
                values.append(value)
            row.append(value)
        rows_values.append(row)

    if not values:
        return _placeholder_bmp(out_w, len(rows_values) or out_h, 'no valid depth')

    min_v = min(values)
    max_v = max(values)
    span = max(max_v - min_v, 1.0)
    rows = []
    for row_values in rows_values:
        row = []
        for value in row_values:
            if value <= 0 or value != value:
                row.append((20, 24, 32))
            else:
                row.append(_depth_color((value - min_v) / span))
        rows.append(row)

    return _pack_bmp(out_w, len(rows), rows)


def _image_to_depth_bmp_fast(msg, out_w, out_h, stride, encoding):
    if cv2 is None or np is None:
        return None

    width = int(msg.width)
    height = int(msg.height)
    data = msg.data

    if encoding in ('16uc1', 'mono16'):
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        elem_size = 2
    elif encoding in ('32fc1',):
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        elem_size = 4
    elif encoding in ('mono8', '8uc1'):
        dtype = np.dtype('u1')
        elem_size = 1
    else:
        return None

    if int(msg.step) < width * elem_size or int(msg.step) % elem_size != 0:
        return None
    row_stride = int(msg.step) // elem_size
    raw = np.frombuffer(data, dtype=dtype)
    needed = row_stride * height
    if raw.size < needed:
        return None

    image = raw[:needed].reshape(height, row_stride)[:, :width]
    sample = image[::stride, ::stride][:out_h, :out_w].astype(np.float32, copy=False)
    valid = np.isfinite(sample) & (sample > 0.0)
    if not np.any(valid):
        return _placeholder_bmp(out_w, out_h, 'no valid depth')

    valid_values = sample[valid]
    min_v = float(np.min(valid_values))
    max_v = float(np.max(valid_values))
    span = max(max_v - min_v, 1.0)
    norm = np.clip((sample - min_v) / span, 0.0, 1.0)

    rgb = np.zeros((sample.shape[0], sample.shape[1], 3), dtype=np.uint8)
    rgb[:, :] = (20, 24, 32)

    mask = valid & (norm < 0.25)
    t = norm[mask] / 0.25
    rgb[mask, 0] = 0
    rgb[mask, 1] = (255.0 * t).astype(np.uint8)
    rgb[mask, 2] = 255

    mask = valid & (norm >= 0.25) & (norm < 0.50)
    t = (norm[mask] - 0.25) / 0.25
    rgb[mask, 0] = 0
    rgb[mask, 1] = 255
    rgb[mask, 2] = (255.0 * (1.0 - t)).astype(np.uint8)

    mask = valid & (norm >= 0.50) & (norm < 0.75)
    t = (norm[mask] - 0.50) / 0.25
    rgb[mask, 0] = (255.0 * t).astype(np.uint8)
    rgb[mask, 1] = 255
    rgb[mask, 2] = 0

    mask = valid & (norm >= 0.75)
    t = (norm[mask] - 0.75) / 0.25
    rgb[mask, 0] = 255
    rgb[mask, 1] = (255.0 * (1.0 - t)).astype(np.uint8)
    rgb[mask, 2] = 0

    return _numpy_rgb_to_bmp(rgb)


def _placeholder_bmp(width, height, message):
    width = max(320, min(width or 320, 640))
    height = max(180, min(height or 180, 360))
    rows = [[(31, 41, 55) for _ in range(width)] for _ in range(height)]
    # Minimal visual marker: red top bar for unavailable/unsupported images.
    for y in range(min(8, height)):
        for x in range(width):
            rows[y][x] = (185, 28, 28)
    return _pack_bmp(width, height, rows)


class HmiHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, bridge_node):
        super().__init__(server_address, handler_class)
        self.bridge_node = bridge_node


class HmiRequestHandler(BaseHTTPRequestHandler):
    server_version = 'PantheraWebHMI/0.1'

    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args):
        try:
            message = fmt % args
        except TypeError:
            message = fmt
        self.server.bridge_node.get_logger().debug(message)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header(
            'Access-Control-Allow-Headers',
            'Content-Type, X-Panthera-Token')
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/api/external/status':
            if not self.server.bridge_node.external_command_authorized(
                    self.headers.get('X-Panthera-Token', '')):
                self._send_json(
                    {'success': False, 'message': 'external command token rejected'},
                    status=401)
                return
            request_id = urllib.parse.parse_qs(parsed.query).get(
                'request_id', [''])[0]
            result, status = self.server.bridge_node.external_command_status(
                request_id)
            self._send_json(result, status=status)
            return
        if parsed.path == '/api/status':
            self._send_json(self.server.bridge_node.snapshot())
            return
        if parsed.path == '/api/point_config':
            self._send_json(self.server.bridge_node.get_point_config())
            return
        if parsed.path == '/api/motion_catalog':
            self._send_json(self.server.bridge_node.get_motion_catalog())
            return
        if parsed.path == '/api/events':
            self._send_events()
            return
        if parsed.path == '/api/camera/rgb_stream.mjpg':
            self._send_mjpeg('rgb')
            return
        if parsed.path in ('/api/camera/rgb', '/api/camera/rgb.bmp', '/api/camera/rgb.jpg'):
            payload, content_type = self.server.bridge_node.get_camera_payload('rgb')
            self._send_bytes(payload, content_type)
            return
        if parsed.path in ('/api/camera/depth', '/api/camera/depth.bmp'):
            payload, content_type = self.server.bridge_node.get_camera_payload('depth')
            self._send_bytes(payload, content_type)
            return
        self._send_static(parsed.path)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path not in (
            '/api/command',
            '/api/speed_scale',
            '/api/camera/restart',
            '/api/point_config',
            '/api/point_config/reload',
            '/api/motion_catalog',
            '/api/motion_catalog/reload',
            '/api/debug/enter',
            '/api/debug/jog',
            '/api/debug/move_to',
            '/api/debug/save',
            '/api/debug/exit',
            '/api/debug/gripper',
            '/api/debug/brush',
            '/api/external/command',
        ):
            self._send_json({'success': False, 'message': 'unknown endpoint'}, status=404)
            return

        try:
            length = int(self.headers.get('Content-Length', '0'))
        except (TypeError, ValueError):
            self._send_json({'success': False, 'message': 'invalid content length'}, status=400)
            return
        if length < 0 or length > 1024 * 1024:
            self._send_json({'success': False, 'message': 'request body is too large'}, status=413)
            return
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        try:
            body = json.loads(raw)
        except json.JSONDecodeError:
            self._send_json({'success': False, 'message': 'invalid json'}, status=400)
            return
        if not isinstance(body, dict):
            self._send_json({'success': False, 'message': 'json body must be an object'}, status=400)
            return

        if parsed.path == '/api/external/command':
            if not self.server.bridge_node.external_command_authorized(
                    self.headers.get('X-Panthera-Token', '')):
                self._send_json(
                    {'success': False, 'message': 'external command token rejected'},
                    status=401)
                return
            result, status = self.server.bridge_node.call_external_command(body)
            self._send_json(result, status=status)
            return
        if parsed.path == '/api/speed_scale':
            result = self.server.bridge_node.call_speed_scale(body)
        elif parsed.path == '/api/camera/restart':
            result = self.server.bridge_node.restart_camera()
        elif parsed.path == '/api/point_config':
            result = self.server.bridge_node.save_point_config(body)
        elif parsed.path == '/api/point_config/reload':
            result = self.server.bridge_node.reload_point_config()
        elif parsed.path == '/api/motion_catalog':
            result = self.server.bridge_node.save_motion_catalog(body)
        elif parsed.path == '/api/motion_catalog/reload':
            result = self.server.bridge_node.reload_motion_catalog()
        elif parsed.path == '/api/debug/enter':
            result = self.server.bridge_node.enter_debug(body)
        elif parsed.path == '/api/debug/jog':
            result = self.server.bridge_node.debug_jog(body)
        elif parsed.path == '/api/debug/move_to':
            result = self.server.bridge_node.debug_move_to(body)
        elif parsed.path == '/api/debug/save':
            result = self.server.bridge_node.save_debug_point()
        elif parsed.path == '/api/debug/exit':
            result = self.server.bridge_node.exit_debug()
        elif parsed.path == '/api/debug/gripper':
            result = self.server.bridge_node.debug_gripper(body)
        elif parsed.path == '/api/debug/brush':
            result = self.server.bridge_node.debug_brush(body)
        else:
            command = body.get('command', '')
            result = self.server.bridge_node.call_command(command)
        self._send_json(result)

    def _send_json(self, data, status=200):
        payload = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(payload)

    def _send_mjpeg(self, channel):
        boundary = 'panthera_frame'
        self.send_response(200)
        self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={boundary}')
        self.send_header('Cache-Control', 'no-cache, no-store, max-age=0')
        self.send_header('Connection', 'close')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        last_stamp = None
        while True:
            try:
                payload, content_type, last_stamp, error = (
                    self.server.bridge_node.wait_for_camera_payload(channel, last_stamp))
                if payload is None:
                    payload = _placeholder_bmp(640, 360, error or 'no image received')
                    content_type = 'image/bmp'
                self.wfile.write(f'--{boundary}\r\n'.encode('ascii'))
                self.wfile.write(f'Content-Type: {content_type}\r\n'.encode('ascii'))
                self.wfile.write(f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(payload)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception as exc:
                self.server.bridge_node.get_logger().warn(f'camera stream closed: {exc}')
                break

    def _send_bytes(self, payload, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store, max-age=0')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(payload)

    def _send_events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        while True:
            try:
                snapshot = self.server.bridge_node.snapshot()
                payload = json.dumps(snapshot, ensure_ascii=False)
                self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                self.wfile.flush()
                time.sleep(1.0)
            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception as exc:
                self.server.bridge_node.get_logger().warn(f'SSE client closed: {exc}')
                break

    def _send_static(self, path):
        if path == '/':
            path = '/index.html'

        normalized = os.path.normpath(urllib.parse.unquote(path).lstrip('/'))
        if normalized.startswith('..'):
            self.send_error(403)
            return

        full_path = os.path.join(self.server.bridge_node.static_dir, normalized)
        if not os.path.isfile(full_path):
            self.send_error(404)
            return

        mime_type, _ = mimetypes.guess_type(full_path)
        mime_type = mime_type or 'application/octet-stream'
        with open(full_path, 'rb') as stream:
            payload = stream.read()

        self.send_response(200)
        self.send_header('Content-Type', mime_type)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


class WebHmiNode(Node):
    def __init__(self):
        super().__init__('panthera_web_hmi')
        self.state_store = HmiStateStore()

        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8080).value)
        default_static = os.path.join(
            get_package_share_directory('panthera_web_hmi'),
            'static',
        )
        self.static_dir = self.declare_parameter('static_dir', default_static).value
        default_point_config_path = os.path.join(
            os.path.expanduser('~'),
            'panthera_workcell_ws',
            'src',
            'panthera_spectrometer_cell',
            'config',
            'spectrometer_cell.yaml')
        if not os.path.isfile(default_point_config_path):
            try:
                default_point_config_path = os.path.join(
                    get_package_share_directory('panthera_spectrometer_cell'),
                    'config',
                    'spectrometer_cell.yaml')
            except Exception:
                pass
        self.point_config_path = self.declare_parameter(
            'point_config_path',
            default_point_config_path).value
        default_motion_catalog_path = os.path.join(
            os.path.expanduser('~'),
            'panthera_workcell_ws',
            'src',
            'panthera_motion',
            'config',
            'motion_catalog.yaml')
        if not os.path.isfile(default_motion_catalog_path):
            try:
                default_motion_catalog_path = os.path.join(
                    get_package_share_directory('panthera_motion'),
                    'config',
                    'motion_catalog.yaml')
            except Exception:
                pass
        self.motion_catalog_path = self.declare_parameter(
            'motion_catalog_path',
            default_motion_catalog_path).value
        default_external_journal = os.path.join(
            os.path.expanduser('~'),
            'panthera_workcell_ws',
            '.runtime',
            'external_commands.json')
        self.external_command_journal_path = self.declare_parameter(
            'external_command_journal_path',
            default_external_journal).value
        self.external_command_token = self.declare_parameter(
            'external_command_token',
            os.environ.get('PANTHERA_EXTERNAL_COMMAND_TOKEN', '')).value
        self._external_command_lock = threading.Lock()
        self._external_command_records = self._load_external_command_journal()
        self.rgb_topic = self.declare_parameter('rgb_topic', '/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/camera/depth/image_raw').value
        self.rgb_info_topic = self.declare_parameter(
            'rgb_info_topic',
            '/camera/color/camera_info').value
        self.depth_info_topic = self.declare_parameter(
            'depth_info_topic',
            '/camera/depth/camera_info').value
        self.compressed_rgb_topic = self.declare_parameter(
            'compressed_rgb_topic',
            '/camera/color/image_raw/compressed').value
        self.subscribe_raw_rgb = bool(self.declare_parameter('subscribe_raw_rgb', False).value)
        self.camera_max_width = int(self.declare_parameter('camera_max_width', 480).value)
        self.camera_target_fps = float(self.declare_parameter('camera_target_fps', 30.0).value)
        self.camera_worker_threads = int(self.declare_parameter('camera_worker_threads', 3).value)
        self.tool_pose_base_frame = self.declare_parameter('tool_pose_base_frame', 'base_link').value
        self.tool_pose_frame = self.declare_parameter('tool_pose_frame', 'gripper_center').value
        self.tool_pose_rate_hz = float(self.declare_parameter('tool_pose_rate_hz', 10.0).value)
        self.tool_pose_max_jump_m = float(
            self.declare_parameter('tool_pose_max_jump_m', 0.08).value)
        self.tool_pose_max_quat_jump = float(
            self.declare_parameter('tool_pose_max_quat_jump', 0.25).value)
        self.tool_pose_filter_accept_after_count = int(
            self.declare_parameter('tool_pose_filter_accept_after_count', 3).value)
        self.camera_store = CameraImageStore(
            max_width=max(160, self.camera_max_width),
            target_fps=self.camera_target_fps,
            worker_count=self.camera_worker_threads)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.camera_callback_group = ReentrantCallbackGroup()
        self.state_store.set_camera_topic(
            'rgb',
            self.compressed_rgb_topic if self.compressed_rgb_topic else self.rgb_topic)
        self.state_store.set_camera_topic('depth', self.depth_topic)

        self.command_service_names = {
            'outlet_1_done': self.declare_parameter(
                'service_outlet_1_done',
                '/spectrometer_cell/simulate_outlet_1_done').value,
            'actual_cycle_outlet_1': self.declare_parameter(
                'service_actual_cycle_outlet_1',
                '/spectrometer_cell/simulate_outlet_1_done').value,
            'outlet_2_done': self.declare_parameter(
                'service_outlet_2_done',
                '/spectrometer_cell/simulate_outlet_2_done').value,
            'actual_cycle_outlet_2': self.declare_parameter(
                'service_actual_cycle_outlet_2',
                '/spectrometer_cell/simulate_outlet_2_done').value,
            'simulate_estop': self.declare_parameter(
                'service_simulate_estop',
                '/spectrometer_cell/simulate_estop').value,
            'clear_estop': self.declare_parameter(
                'service_clear_estop',
                '/spectrometer_cell/clear_estop').value,
            'request_reset': self.declare_parameter(
                'service_request_reset',
                '/spectrometer_cell/request_reset').value,
            'restart_cleaning_motor': self.declare_parameter(
                'service_restart_cleaning_motor',
                '/spectrometer_cell/restart_cleaning_motor').value,
            'manual_mode': self.declare_parameter(
                'service_manual_mode',
                '/spectrometer_cell/manual_mode').value,
            'auto_mode': self.declare_parameter(
                'service_auto_mode',
                '/spectrometer_cell/auto_mode').value,
            'detection_done': self.declare_parameter(
                'service_detection_done',
                '/spectrometer_cell/simulate_detection_done').value,
        }
        self.command_clients = {
            name: self.create_client(Trigger, service_name)
            for name, service_name in self.command_service_names.items()
        }
        self.workflow_service_name = self.declare_parameter('workflow_service', '/run_workflow').value
        self.workflow_client = self.create_client(RunWorkflow, self.workflow_service_name)
        self.workflow_commands = {
            'workflow_fixed_large_motion_demo': 'fixed_large_motion_demo',
            'workflow_visible_motion_check': 'visible_motion_check',
            'workflow_cup_pick_place': 'cup_pick_place',
            'workflow_outlet_1_pick_check_actual': 'outlet_1_pick_check_actual',
            'workflow_outlet_2_pick_check_actual': 'outlet_2_pick_check_actual',
            'workflow_spectrometer_place_check_actual': 'spectrometer_place_check_actual',
            'workflow_clean_dump_check_actual': 'clean_dump_check_actual',
            'workflow_arm_home': 'arm_home',
            'workflow_gripper_open': 'gripper_open',
            'workflow_gripper_close': 'gripper_close',
        }
        self.spectrometer_stop_motion_service_name = self.declare_parameter(
            'spectrometer_stop_motion_service',
            '/spectrometer_cell/stop_motion').value
        self.spectrometer_stop_motion_client = self.create_client(
            Trigger,
            self.spectrometer_stop_motion_service_name)
        self.spectrometer_recover_home_service_name = self.declare_parameter(
            'spectrometer_recover_home_service',
            '/spectrometer_cell/recover_home').value
        self.spectrometer_recover_home_client = self.create_client(
            Trigger,
            self.spectrometer_recover_home_service_name)
        self.workflow_stop_service_name = self.declare_parameter(
            'workflow_stop_service',
            '/stop_workflow').value
        self.workflow_stop_client = self.create_client(Trigger, self.workflow_stop_service_name)
        self.speed_scale_service_name = self.declare_parameter(
            'speed_scale_service',
            '/spectrometer_cell/set_speed_scale').value
        self.speed_scale_client = self.create_client(SetSpeedScale, self.speed_scale_service_name)
        self.reload_config_service_name = self.declare_parameter(
            'reload_config_service',
            '/spectrometer_cell/reload_config').value
        self.reload_config_client = self.create_client(Trigger, self.reload_config_service_name)
        self.motion_reload_service_name = self.declare_parameter(
            'motion_reload_service',
            '/motion/reload').value
        self.motion_reload_client = self.create_client(
            Trigger,
            self.motion_reload_service_name)
        self.debug_callback_group = ReentrantCallbackGroup()
        self.motion_execute_client = ActionClient(
            self,
            ExecuteMotion,
            '/motion/execute',
            callback_group=self.debug_callback_group)
        self.stage_jog_client = self.create_client(
            StageJog,
            '/motion/stage_jog',
            callback_group=self.debug_callback_group)
        self.debug_gripper_open_client = self.create_client(
            Trigger,
            '/spectrometer_cell/debug/gripper_open',
            callback_group=self.debug_callback_group)
        self.debug_gripper_close_client = self.create_client(
            Trigger,
            '/spectrometer_cell/debug/gripper_close',
            callback_group=self.debug_callback_group)
        self.debug_brush_client = self.create_client(
            SetBrush,
            '/spectrometer_cell/debug/set_brush',
            callback_group=self.debug_callback_group)
        self._debug_lock = threading.RLock()
        self._debug_operation_lock = threading.Lock()
        self._debug = {
            'active': False,
            'phase': 'inactive',
            'selected_point': '',
            'dirty': False,
            'history': [],
            'commanded_pose': None,
            'last_message': '',
            'brush_enabled': False,
            'brush_speed_percent': 50.0,
        }
        default_camera_restart_script = os.path.join(
            os.path.expanduser('~'),
            'panthera_workcell_ws',
            'scripts',
            'restart_camera.sh')
        self.camera_restart_script = self.declare_parameter(
            'camera_restart_script',
            default_camera_restart_script).value

        self.create_subscription(String, '/spectrometer_cell/state', self._on_cell_state, 10)
        self.create_subscription(String, '/spectrometer_cell/context', self._on_cell_context, 10)
        self.create_subscription(String, '/spectrometer_cell/error', self._on_cell_error, 10)
        self.create_subscription(
            LaserDistance, '/sensors/laser/distance', self._on_laser, qos_profile_sensor_data)
        self.create_subscription(WorkflowStatus, '/workflow/status', self._on_workflow, 10)
        self.create_subscription(ExternalSignal, '/workflow/external_signal', self._on_external_signal, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(
            CameraInfo,
            self.rgb_info_topic,
            self._on_rgb_camera_info,
            self.camera_qos,
            callback_group=self.camera_callback_group)
        self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self._on_depth_camera_info,
            self.camera_qos,
            callback_group=self.camera_callback_group)
        if self.subscribe_raw_rgb or not self.compressed_rgb_topic:
            self.create_subscription(
                Image,
                self.rgb_topic,
                self._on_rgb_image,
                self.camera_qos,
                callback_group=self.camera_callback_group)
        self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth_image,
            self.camera_qos,
            callback_group=self.camera_callback_group)
        if self.compressed_rgb_topic:
            self.create_subscription(
                CompressedImage,
                self.compressed_rgb_topic,
                self._on_compressed_rgb_image,
                self.camera_qos,
                callback_group=self.camera_callback_group)

        self.service_timer = self.create_timer(1.0, self._update_service_status)
        self.tool_pose_timer = self.create_timer(
            1.0 / max(1.0, self.tool_pose_rate_hz),
            self._update_tool_pose)
        self._update_service_status()

        self.http_server = HmiHttpServer((self.host, self.port), HmiRequestHandler, self)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(
            f'Panthera Web HMI started: http://{self.host}:{self.port} static={self.static_dir}')

    def _on_cell_state(self, msg):
        self.state_store.set_cell_state(msg.data)

    def _on_cell_context(self, msg):
        self.state_store.set_cell_context(msg.data)

    def _on_cell_error(self, msg):
        self.state_store.set_cell_error(msg.data)

    def _on_laser(self, msg):
        self.state_store.set_laser(msg)

    def _on_workflow(self, msg):
        self.state_store.set_workflow(msg)

    def _on_external_signal(self, msg):
        self.state_store.set_external_signal(msg)

    def _on_joint_state(self, msg):
        self.state_store.set_joint_state(msg)

    def _on_rgb_camera_info(self, msg):
        self.state_store.set_camera_info('rgb', msg)

    def _on_depth_camera_info(self, msg):
        self.state_store.set_camera_info('depth', msg)

    def _on_rgb_image(self, msg):
        self.state_store.set_camera_image('rgb', msg)
        self.camera_store.set_raw('rgb', msg)

    def _on_depth_image(self, msg):
        self.state_store.set_camera_image('depth', msg)
        self.camera_store.set_raw('depth', msg)

    def _on_compressed_rgb_image(self, msg):
        self.state_store.set_camera_compressed('rgb', msg)
        self.camera_store.set_compressed('rgb', msg)

    def _update_tool_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tool_pose_base_frame,
                self.tool_pose_frame,
                Time(),
                timeout=Duration(seconds=0.02))
            self.state_store.set_tool_pose(
                self.tool_pose_base_frame,
                self.tool_pose_frame,
                transform,
                self.tool_pose_max_jump_m,
                self.tool_pose_max_quat_jump,
                self.tool_pose_filter_accept_after_count)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            self.state_store.set_tool_pose_error(
                self.tool_pose_base_frame,
                self.tool_pose_frame,
                str(exc))

    def _update_service_status(self):
        services = {}
        for command, client in self.command_clients.items():
            services[command] = {
                'service': self.command_service_names[command],
                'ready': bool(client.service_is_ready()),
            }
        workflow_ready = bool(self.workflow_client.service_is_ready())
        for command, workflow_name in self.workflow_commands.items():
            services[command] = {
                'service': self.workflow_service_name,
                'workflow': workflow_name,
                'ready': workflow_ready,
            }
        services['spectrometer_stop_motion'] = {
            'service': self.spectrometer_stop_motion_service_name,
            'ready': bool(self.spectrometer_stop_motion_client.service_is_ready()),
        }
        services['spectrometer_recover_home'] = {
            'service': self.spectrometer_recover_home_service_name,
            'ready': bool(self.spectrometer_recover_home_client.service_is_ready()),
        }
        services['workflow_stop'] = {
            'service': self.workflow_stop_service_name,
            'ready': bool(self.workflow_stop_client.service_is_ready()),
        }
        services['speed_scale'] = {
            'service': self.speed_scale_service_name,
            'ready': bool(self.speed_scale_client.service_is_ready()),
        }
        services['reload_config'] = {
            'service': self.reload_config_service_name,
            'ready': bool(self.reload_config_client.service_is_ready()),
        }
        services['camera_restart'] = {
            'script': self.camera_restart_script,
            'ready': bool(os.path.isfile(self.camera_restart_script) and os.access(self.camera_restart_script, os.X_OK)),
        }
        services['point_config'] = {
            'path': self.point_config_path,
            'ready': bool(os.path.isfile(self.point_config_path) and os.access(self.point_config_path, os.R_OK)),
            'writable': bool(os.path.isfile(self.point_config_path) and os.access(self.point_config_path, os.W_OK)),
        }
        services['motion_catalog'] = {
            'path': self.motion_catalog_path,
            'ready': bool(
                os.path.isfile(self.motion_catalog_path) and
                os.access(self.motion_catalog_path, os.R_OK)),
            'writable': bool(
                os.path.isfile(self.motion_catalog_path) and
                os.access(self.motion_catalog_path, os.W_OK)),
            'reload_ready': bool(self.motion_reload_client.service_is_ready()),
        }
        services['debug_motion'] = {
            'execute_ready': bool(self.motion_execute_client.server_is_ready()),
            'jog_ready': bool(self.stage_jog_client.service_is_ready()),
            'gripper_ready': bool(
                self.debug_gripper_open_client.service_is_ready() and
                self.debug_gripper_close_client.service_is_ready()),
            'brush_ready': bool(self.debug_brush_client.service_is_ready()),
        }
        self.state_store.set_services(services)

    def snapshot(self):
        self._update_service_status()
        data = self.state_store.snapshot()
        data['camera']['debug'] = self.camera_store.stats()
        data['debug'] = self.debug_snapshot()
        return data

    def debug_snapshot(self):
        with self._debug_lock:
            result = {key: value for key, value in self._debug.items() if key != 'history'}
            result['jog_count'] = len(self._debug['history'])
        return result

    @staticmethod
    def _wait_future(future, timeout_sec):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError('ROS request timed out')
        return future.result()

    def _call_service_request(self, client, request, service_name, timeout_sec=10.0):
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=2.0):
            return None, f'ROS service not ready: {service_name}'
        try:
            return self._wait_future(client.call_async(request), timeout_sec), ''
        except Exception as exc:
            return None, f'ROS service call failed: {service_name}: {exc}'

    def _execute_motion_route(self, route_name, speed_scale=1.0, timeout_sec=90.0):
        if not self.motion_execute_client.wait_for_server(timeout_sec=3.0):
            return {'success': False, 'message': 'motion action server is unavailable'}
        goal = ExecuteMotion.Goal()
        goal.route_name = route_name
        goal.speed_scale = float(speed_scale)
        goal.dry_run = False
        try:
            handle = self._wait_future(
                self.motion_execute_client.send_goal_async(goal), 5.0)
            if not handle or not handle.accepted:
                return {'success': False, 'message': f'motion route rejected: {route_name}'}
            wrapped = self._wait_future(handle.get_result_async(), timeout_sec)
            result = wrapped.result
            return {
                'success': bool(result.success),
                'message': result.message,
                'error_code': int(result.error_code),
                'route_name': route_name,
            }
        except Exception as exc:
            return {'success': False, 'message': f'motion route failed: {route_name}: {exc}'}

    @staticmethod
    def _debug_point_routes(point_name):
        return f'debug_safe_to_{point_name}', f'debug_{point_name}_to_safe'

    def _debug_catalog_target(self, point_name):
        catalog, references = self._load_motion_catalog()
        point = catalog.get('points', {}).get(point_name)
        if not point or 'tunable' not in point.get('tags', []) or 'pose' not in point:
            raise ValueError(f'point is not a tunable Cartesian target: {point_name}')
        entry_route, exit_route = self._debug_point_routes(point_name)
        routes = catalog.get('routes', {})
        if entry_route not in routes or exit_route not in routes:
            raise ValueError(f'point has no validated debug route: {point_name}')
        return catalog, references, point, entry_route, exit_route

    def _debug_verify_home(self, catalog):
        snapshot = self.state_store.snapshot()
        joint_state = snapshot.get('joint_state', {})
        if joint_state.get('age_sec') is None or joint_state.get('age_sec') > 0.5:
            return False, 'joint state is stale'
        names = joint_state.get('names', [])
        positions = joint_state.get('positions', [])
        home = catalog.get('points', {}).get('home_near', {}).get('joints', [])
        robot_names = catalog.get('robot', {}).get('joint_names', [])
        if len(home) != len(robot_names):
            return False, 'home_near joints are invalid'
        errors = []
        for name, target in zip(robot_names, home):
            if name not in names:
                return False, f'joint state is missing {name}'
            index = names.index(name)
            if index >= len(positions):
                return False, f'joint state position is missing {name}'
            errors.append(abs(float(positions[index]) - float(target)))
        maximum = max(errors, default=float('inf'))
        return maximum <= 0.05, f'Home max joint error={maximum:.4f}rad'

    def _debug_require_paused(self):
        snapshot = self.state_store.snapshot()
        cell = snapshot.get('spectrometer_cell', {})
        context = cell.get('context', {}) if isinstance(cell.get('context'), dict) else {}
        state = cell.get('state') or context.get('state') or 'UNKNOWN'
        if state != 'PAUSED':
            return False, f'debug command requires PAUSED state, current={state}'
        if context.get('active_command_id') or context.get('active_action_name'):
            return False, 'robot action is active'
        return True, 'debug interlock ready'

    def enter_debug(self, body):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'another debug operation is running'}

        def fail(result):
            message = result.get('message', 'debug entry failed')
            with self._debug_lock:
                self._debug.update(
                    active=False, phase='error', selected_point='', dirty=False,
                    history=[], commanded_pose=None, last_message=message)
            return result

        try:
            point_name = str(body.get('point_name', '')).strip()
            catalog, _, point, entry_route, _ = self._debug_catalog_target(point_name)
            snapshot = self.state_store.snapshot()
            cell = snapshot.get('spectrometer_cell', {})
            context = cell.get('context', {}) if isinstance(cell.get('context'), dict) else {}
            state = cell.get('state') or context.get('state') or 'UNKNOWN'
            if context.get('has_active_task') or context.get('cup_in_gripper') or context.get('spectrometer_occupied'):
                return {'success': False, 'message': 'debug entry rejected: active task/cup exists'}
            if state not in ('IDLE', 'WAIT_DISCHARGE', 'PAUSED'):
                return {'success': False, 'message': f'debug entry rejected from state {state}'}
            with self._debug_lock:
                if self._debug['active']:
                    return {'success': False, 'message': 'exit the current debug point first'}
                self._debug.update(phase='pausing', last_message='正在暂停自动流程')

            if state != 'PAUSED':
                paused = self.call_command('manual_mode')
                if not paused.get('success'):
                    return fail(paused)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    current = self.state_store.snapshot().get('spectrometer_cell', {}).get('state')
                    if current == 'PAUSED':
                        break
                    time.sleep(0.05)
                else:
                    return fail({'success': False, 'message': 'state machine did not enter PAUSED'})

            at_home, home_message = self._debug_verify_home(catalog)
            if not at_home:
                with self._debug_lock:
                    self._debug.update(phase='recovering_home', last_message=home_message)
                recovery = self._call_trigger_client(
                    self.spectrometer_recover_home_client,
                    self.spectrometer_recover_home_service_name,
                    timeout_sec=45.0)
                if not recovery.get('success'):
                    return fail(recovery)
                at_home, home_message = self._debug_verify_home(catalog)
                if not at_home:
                    return fail({
                        'success': False,
                        'message': f'Home verification failed: {home_message}',
                    })

            with self._debug_lock:
                self._debug.update(phase='moving_safe', last_message='正在前往安全调试点')
            result = self._execute_motion_route('home_to_safe_center')
            if not result.get('success'):
                return fail(result)
            with self._debug_lock:
                self._debug.update(phase='moving_target', last_message=f'正在前往 {point_name}')
            result = self._execute_motion_route(entry_route)
            if not result.get('success'):
                return fail(result)
            # The trajectory controller can finish before MIT position error
            # reaches its steady loaded value. Do not count that settling as
            # the operator's first Cartesian jog.
            time.sleep(1.0)
            with self._debug_lock:
                self._debug.update(
                    active=True,
                    phase='ready',
                    selected_point=point_name,
                    dirty=False,
                    history=[],
                    commanded_pose={
                        'xyz': [float(value) for value in point['pose']['xyz']],
                        'rpy': [float(value) for value in point['pose']['rpy']],
                    },
                    last_message='点位已到达，可以点动')
            return {'success': True, 'message': 'debug point reached', 'point_name': point_name}
        except Exception as exc:
            return fail({'success': False, 'message': str(exc)})
        finally:
            self._debug_operation_lock.release()

    def _stage_and_execute_jog(self, deltas, absolute_target=None):
        started = time.monotonic()
        request = StageJog.Request()
        request.delta_x_m, request.delta_y_m, request.delta_z_m = deltas[:3]
        request.delta_roll_rad, request.delta_pitch_rad, request.delta_yaw_rad = deltas[3:]
        request.process_profile = False
        request.base_point_name = ''
        request.point_offset_xyz_m = [0.0, 0.0, 0.0]
        request.use_absolute_target = absolute_target is not None
        if absolute_target is not None:
            request.absolute_target_xyz_m = list(absolute_target[0])
            request.absolute_target_rpy_rad = list(absolute_target[1])
        response, error = self._call_service_request(
            self.stage_jog_client, request, '/motion/stage_jog', timeout_sec=20.0)
        stage_elapsed = time.monotonic() - started
        if not response or not response.success:
            return {'success': False, 'message': error or response.message}
        result = self._execute_motion_route(response.route_name, speed_scale=1.0, timeout_sec=30.0)
        result['stage_elapsed_sec'] = stage_elapsed
        result['execute_elapsed_sec'] = time.monotonic() - started - stage_elapsed
        result['target_xyz_m'] = list(response.target_xyz_m)
        result['target_rpy_rad'] = list(response.target_rpy_rad)
        return result

    def debug_jog(self, body):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'previous jog is still running'}
        try:
            ready, message = self._debug_require_paused()
            with self._debug_lock:
                active = self._debug['active'] and self._debug['phase'] == 'ready'
            if not ready or not active:
                return {'success': False, 'message': message if not ready else 'debug point is not ready'}
            translation = [float(value) for value in body.get('translation_m', [0, 0, 0])]
            rotation = [float(value) for value in body.get('rotation_rad', [0, 0, 0])]
            if len(translation) != 3 or len(rotation) != 3:
                return {'success': False, 'message': 'jog vectors must contain three values'}
            if not all(math.isfinite(value) for value in translation + rotation):
                return {'success': False, 'message': 'jog values must be finite'}
            deltas = translation + rotation
            active_deltas = [value for value in deltas if abs(value) > 1e-9]
            if len(active_deltas) != 1:
                return {'success': False, 'message': 'jog must move exactly one axis'}
            if any(
                    abs(value) > 1e-9 and not 0.002 <= abs(value) <= 0.020
                    for value in translation):
                return {
                    'success': False,
                    'message': 'MIT Cartesian translation step must be between 2mm and 20mm',
                }
            min_rotation = math.radians(0.1)
            max_rotation = math.radians(10.0)
            if any(
                    abs(value) > 1e-9 and not min_rotation <= abs(value) <= max_rotation
                    for value in rotation):
                return {
                    'success': False,
                    'message': 'rotation step must be between 0.1 and 10 degrees',
                }
            start_xyz, start_rpy = self._fresh_tool_pose()
            target_xyz, target_rpy = cartesian_pose_target(
                start_xyz, start_rpy, deltas)
            with self._debug_lock:
                self._debug.update(phase='jogging', last_message='正在执行点动')
            return self._execute_debug_target(
                start_xyz, start_rpy, target_xyz, target_rpy, deltas,
                max_corrections=0)
        except Exception as exc:
            with self._debug_lock:
                self._debug.update(phase='error', last_message=f'点动失败: {exc}')
            return {'success': False, 'message': f'jog failed: {exc}'}
        finally:
            self._debug_operation_lock.release()

    def debug_move_to(self, body):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'previous debug motion is still running'}
        try:
            ready, message = self._debug_require_paused()
            with self._debug_lock:
                active = self._debug['active'] and self._debug['phase'] == 'ready'
            if not ready or not active:
                return {
                    'success': False,
                    'message': message if not ready else 'debug point is not ready',
                }
            try:
                target_xyz = [float(value) for value in body.get('target_xyz_m', [])]
                target_rpy = [float(value) for value in body.get('target_rpy_rad', [])]
            except (TypeError, ValueError):
                return {'success': False, 'message': '目标坐标必须是有效数字'}
            if len(target_xyz) != 3 or len(target_rpy) != 3:
                return {'success': False, 'message': '目标坐标必须包含 XYZ 和 Roll/Pitch/Yaw'}
            if not all(math.isfinite(value) for value in target_xyz + target_rpy):
                return {'success': False, 'message': '目标坐标不能包含无穷或空值'}
            start_xyz, start_rpy = self._settled_tool_pose()
            deltas = cartesian_pose_delta(start_xyz, start_rpy, target_xyz, target_rpy)
            with self._debug_lock:
                self._debug.update(phase='jogging', last_message='正在执行输入坐标')
            return self._execute_debug_target(
                start_xyz, start_rpy, target_xyz, target_rpy, deltas)
        except ValueError as exc:
            with self._debug_lock:
                self._debug.update(phase='ready', last_message=str(exc))
            return {'success': False, 'message': str(exc)}
        except Exception as exc:
            with self._debug_lock:
                self._debug.update(phase='error', last_message=f'坐标执行失败: {exc}')
            return {'success': False, 'message': f'坐标执行失败: {exc}'}
        finally:
            self._debug_operation_lock.release()

    def _execute_debug_target(
            self, start_xyz, start_rpy, target_xyz, target_rpy, requested_delta,
            max_corrections=2):
        translation_distance = math.sqrt(sum(value * value for value in requested_delta[:3]))
        rotation_distance = rpy_orientation_error(start_rpy, target_rpy)
        if translation_distance > 0.0205:
            raise ValueError(
                f'单次输入位置距离为 {translation_distance * 1000.0:.1f} mm，'
                '不得超过 20 mm')
        if rotation_distance > math.radians(10.0) + 1e-9:
            raise ValueError(
                f'单次输入姿态变化为 {math.degrees(rotation_distance):.1f}°，'
                '不得超过 10°')
        if translation_distance < 0.0001 and rotation_distance < math.radians(0.05):
            message = '实测位置已经在输入目标内，无需运动'
            with self._debug_lock:
                self._debug.update(phase='ready', last_message=message)
            return {'success': True, 'message': message, 'already_at_target': True}

        with self._debug_lock:
            previous_command = self._debug.get('commanded_pose')
        if previous_command:
            commanded_xyz, commanded_rpy = compensated_command_target(
                start_xyz,
                start_rpy,
                previous_command['xyz'],
                previous_command['rpy'],
                target_xyz,
                target_rpy)
        else:
            commanded_xyz, commanded_rpy = list(target_xyz), list(target_rpy)
        measured_to_command = cartesian_pose_delta(
            start_xyz, start_rpy, commanded_xyz, commanded_rpy)
        result = self._stage_and_execute_jog(
            measured_to_command, (commanded_xyz, commanded_rpy))
        if not result.get('success'):
            raise RuntimeError(result.get('message', 'Cartesian motion failed'))
        commanded_xyz = list(result['target_xyz_m'])
        commanded_rpy = list(result['target_rpy_rad'])
        if max_corrections:
            measured_xyz, measured_rpy = self._settled_tool_pose()
        else:
            time.sleep(0.05)
            measured_xyz, measured_rpy = self._fresh_tool_pose()
        position_error = math.sqrt(sum(
            (target - measured) ** 2
            for measured, target in zip(measured_xyz, target_xyz)))
        orientation_error = rpy_orientation_error(measured_rpy, target_rpy)
        correction_count = 0
        convergence_stopped = False
        error_trace = [{
            'position_error_m': position_error,
            'orientation_error_rad': orientation_error,
            'position_residual_m': [
                target - measured
                for measured, target in zip(measured_xyz, target_xyz)],
        }]

        for _ in range(max_corrections):
            if (
                    position_error <= DEBUG_POSITION_TOLERANCE_M and
                    orientation_error <= math.radians(0.5)):
                break
            previous_normalized_error = max(
                position_error / DEBUG_POSITION_TOLERANCE_M,
                orientation_error / math.radians(0.5))
            correction = cartesian_pose_delta(
                measured_xyz, measured_rpy, target_xyz, target_rpy)
            if position_error > DEBUG_POSITION_TOLERANCE_M:
                translation_scale = (
                    min(0.70, 0.003 / position_error)
                    if position_error > 0.003 else 0.35)
                correction[:3] = [
                    value * translation_scale for value in correction[:3]]
            else:
                correction[:3] = [0.0, 0.0, 0.0]
            if orientation_error > math.radians(0.5):
                rotation_scale = min(
                    0.60 if orientation_error > math.radians(1.0) else 0.30,
                    math.radians(0.75) / orientation_error)
                correction[3:] = [
                    value * rotation_scale for value in correction[3:]]
            else:
                correction[3:] = [0.0, 0.0, 0.0]
            commanded_xyz, commanded_rpy = cartesian_pose_target(
                commanded_xyz, commanded_rpy, correction)
            measured_to_command = cartesian_pose_delta(
                measured_xyz, measured_rpy, commanded_xyz, commanded_rpy)
            correction_result = self._stage_and_execute_jog(
                measured_to_command, (commanded_xyz, commanded_rpy))
            if not correction_result.get('success'):
                raise RuntimeError(
                    f'bounded Cartesian correction failed: '
                    f'{correction_result.get("message", "unknown error")}')
            result = correction_result
            correction_count += 1
            measured_xyz, measured_rpy = self._settled_tool_pose()
            position_error = math.sqrt(sum(
                (target - measured) ** 2
                for measured, target in zip(measured_xyz, target_xyz)))
            orientation_error = rpy_orientation_error(measured_rpy, target_rpy)
            error_trace.append({
                'position_error_m': position_error,
                'orientation_error_rad': orientation_error,
                'position_residual_m': [
                    target - measured
                    for measured, target in zip(measured_xyz, target_xyz)],
            })
            normalized_error = max(
                position_error / DEBUG_POSITION_TOLERANCE_M,
                orientation_error / math.radians(0.5))
            if normalized_error >= previous_normalized_error:
                convergence_stopped = True
                break

        within_tolerance = (
            position_error <= DEBUG_POSITION_TOLERANCE_M and
            orientation_error <= math.radians(0.5))
        command_bias = math.sqrt(sum(
            (commanded - target) ** 2
            for commanded, target in zip(commanded_xyz, target_xyz)))
        if within_tolerance:
            warning = ''
        elif convergence_stopped:
            warning = '，⚠ 误差开始变差，已停止同向修正'
        else:
            warning = '，⚠ 请检查实测值后再决定是否重试'
        message = (
            f'目标 XYZ [{", ".join(f"{value * 1000.0:.2f}" for value in target_xyz)}] mm，'
            f'实测 [{", ".join(f"{value * 1000.0:.2f}" for value in measured_xyz)}] mm，'
            f'位置误差 {position_error * 1000.0:.2f} mm，'
            f'姿态误差 {math.degrees(orientation_error):.2f}°，'
            f'负载补偿 {command_bias * 1000.0:.2f} mm，'
            f'稳定修正 {correction_count} 次{warning}'
        )
        result.update({
            'message': message,
            'requested_target_xyz_m': list(target_xyz),
            'requested_target_rpy_rad': list(target_rpy),
            'measured_xyz_m': list(measured_xyz),
            'measured_rpy_rad': list(measured_rpy),
            'position_error_m': position_error,
            'orientation_error_rad': orientation_error,
            'command_bias_m': command_bias,
            'correction_count': correction_count,
            'convergence_stopped': convergence_stopped,
            'error_trace': error_trace,
            'within_step_tolerance': within_tolerance,
            'warning': not within_tolerance,
        })
        with self._debug_lock:
            self._debug['commanded_pose'] = {
                'xyz': list(result['target_xyz_m'] if within_tolerance else measured_xyz),
                'rpy': list(result['target_rpy_rad'] if within_tolerance else measured_rpy),
            }
            self._debug['history'].append(list(requested_delta))
            self._debug['dirty'] = True
            self._debug.update(phase='ready', last_message=message)
        return result

    def _fresh_tool_pose(self):
        tool = self.state_store.snapshot().get('tool_pose', {})
        if not tool.get('available') or tool.get('age_sec') is None or tool.get('age_sec') > 0.5:
            raise ValueError('fresh measured tool pose is unavailable')
        position = tool.get('position_m', {})
        rpy = tool.get('rpy_rad', {})
        xyz = [float(position[axis]) for axis in ('x', 'y', 'z')]
        angles = [float(rpy[axis]) for axis in ('roll', 'pitch', 'yaw')]
        if not all(math.isfinite(value) for value in xyz + angles):
            raise ValueError('measured tool pose contains a non-finite value')
        return xyz, angles

    def _settled_tool_pose(self, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        samples = []
        while time.monotonic() < deadline:
            xyz, rpy = self._fresh_tool_pose()
            samples.append((xyz, rpy))
            samples = samples[-4:]
            if len(samples) == 4:
                position_span = max(
                    max(sample[0][axis] for sample in samples) -
                    min(sample[0][axis] for sample in samples)
                    for axis in range(3))
                orientation_span = max(
                    rpy_orientation_error(sample[1], samples[-1][1])
                    for sample in samples)
                if position_span <= 0.0005 and orientation_span <= math.radians(0.25):
                    averaged_xyz = [
                        sum(sample[0][axis] for sample in samples) / len(samples)
                        for axis in range(3)
                    ]
                    return averaged_xyz, samples[-1][1]
            time.sleep(0.05)
        raise ValueError('TCP is still settling; wait a moment and retry')

    def save_debug_point(self):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'another debug operation is running'}
        try:
            with self._debug_lock:
                if not self._debug['active'] or self._debug['phase'] != 'ready':
                    return {'success': False, 'message': 'debug point is not ready'}
                point_name = self._debug['selected_point']
                commanded_pose = self._debug.get('commanded_pose')
                self._debug.update(phase='saving', last_message='正在校验并热重载')
            catalog, _, point, _, _ = self._debug_catalog_target(point_name)
            if not commanded_pose:
                raise ValueError('commanded debug pose is unavailable')
            xyz = [float(value) for value in commanded_pose['xyz']]
            rpy = [float(value) for value in commanded_pose['rpy']]
            old_xyz = [float(value) for value in point['pose']['xyz']]
            delta = [new - old for new, old in zip(xyz, old_xyz)]
            point['pose']['xyz'] = xyz
            point['pose']['rpy'] = rpy
            pose_updates = {point_name: point['pose']}
            followers = catalog.get('commissioning', {}).get(
                'translation_followers', {}).get(point_name, [])
            for follower in followers:
                follower_point = catalog.get('points', {}).get(follower.get('point', ''))
                if not follower_point or 'pose' not in follower_point:
                    raise ValueError(f'invalid calibration follower: {follower}')
                axes = str(follower.get('axes', ''))
                follower_xyz = [float(value) for value in follower_point['pose']['xyz']]
                for index, axis in enumerate('xyz'):
                    if axis in axes:
                        follower_xyz[index] += delta[index]
                follower_point['pose']['xyz'] = follower_xyz
                follower_point['pose']['rpy'] = list(rpy)
                pose_updates[follower['point']] = follower_point['pose']
            result = self.save_motion_catalog(
                {'catalog': catalog},
                allow_debug=True,
                pose_updates=pose_updates)
            with self._debug_lock:
                if result.get('success'):
                    self._debug.update(
                        phase='ready', dirty=False, history=[],
                        last_message='点位已保存并热重载')
                else:
                    self._debug.update(phase='ready', last_message=result.get('message', '保存失败'))
            result['point_name'] = point_name
            result['delta_xyz_mm'] = [value * 1000.0 for value in delta]
            return result
        except Exception as exc:
            with self._debug_lock:
                self._debug.update(phase='ready', last_message=str(exc))
            return {'success': False, 'message': str(exc)}
        finally:
            self._debug_operation_lock.release()

    def exit_debug(self):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'another debug operation is running'}

        def recover_home(failure):
            recovery = self._call_trigger_client(
                self.spectrometer_recover_home_client,
                self.spectrometer_recover_home_service_name,
                timeout_sec=45.0)
            if recovery.get('success'):
                catalog, _ = self._load_motion_catalog()
                at_home, home_message = self._debug_verify_home(catalog)
                if at_home:
                    with self._debug_lock:
                        self._debug.update(
                            active=False, phase='inactive', selected_point='', dirty=False,
                            history=[], commanded_pose=None, brush_enabled=False,
                            last_message='安全退出轨迹失败，已自动恢复到 Home')
                    return {
                        'success': True,
                        'message': (
                            f"safe exit route failed ({failure.get('message')}); "
                            f'recovered Home: {home_message}'
                        ),
                        'recovered_home': True,
                    }
            message = (
                f"{failure.get('message', 'safe exit failed')}; "
                f"Home recovery failed: {recovery.get('message', 'unknown error')}"
            )
            with self._debug_lock:
                self._debug.update(phase='error', last_message=message)
            return {'success': False, 'message': message}

        try:
            with self._debug_lock:
                if not self._debug['active']:
                    return {'success': True, 'message': 'debug mode is already inactive'}
                point_name = self._debug['selected_point']
                self._debug.update(phase='returning', last_message='正在安全退出调试')
            stop = SetBrush.Request()
            stop.enabled = False
            stop.speed_percent = 0.0
            self._call_service_request(
                self.debug_brush_client, stop, '/spectrometer_cell/debug/set_brush', 5.0)
            rewind = self._rewind_debug_history()
            if not rewind.get('success'):
                with self._debug_lock:
                    self._debug.update(phase='error', last_message=rewind['message'])
                return rewind
            _, _, _, _, exit_route = self._debug_catalog_target(point_name)
            result = self._execute_motion_route(exit_route)
            if not result.get('success'):
                return recover_home(result)
            result = self._execute_motion_route('safe_center_to_home')
            if not result.get('success'):
                return recover_home(result)
            with self._debug_lock:
                self._debug.update(
                    active=False, phase='inactive', selected_point='', dirty=False,
                    history=[], commanded_pose=None, brush_enabled=False,
                    last_message='已安全回到 Home，自动流程仍暂停')
            return {'success': True, 'message': 'debug mode exited at Home; automatic mode remains paused'}
        except Exception as exc:
            with self._debug_lock:
                self._debug.update(phase='error', last_message=f'安全退出失败: {exc}')
            return {'success': False, 'message': f'debug exit failed: {exc}'}
        finally:
            self._debug_operation_lock.release()

    def _rewind_debug_history(self):
        while True:
            with self._debug_lock:
                if not self._debug['history']:
                    return {'success': True, 'message': 'debug jog path rewound'}
                delta = list(self._debug['history'][-1])
                self._debug.update(
                    phase='returning',
                    last_message=f'正在沿点动原路返回，剩余 {len(self._debug["history"])} 步')
            result = self._stage_and_execute_jog([-value for value in delta])
            if not result.get('success'):
                return {
                    'success': False,
                    'message': (
                        'debug jog rewind failed; robot remains enabled: '
                        f'{result.get("message", "unknown error")}'
                    ),
                }
            with self._debug_lock:
                self._debug['history'].pop()
                self._debug['commanded_pose'] = {
                    'xyz': list(result['target_xyz_m']),
                    'rpy': list(result['target_rpy_rad']),
                }

    def debug_gripper(self, body):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'another debug operation is running'}
        try:
            return self._debug_gripper_locked(body)
        finally:
            self._debug_operation_lock.release()

    def _debug_gripper_locked(self, body):
        with self._debug_lock:
            if not self._debug['active'] or self._debug['phase'] != 'ready':
                return {'success': False, 'message': 'debug point is not ready'}
        command = str(body.get('command', '')).strip().lower()
        client = self.debug_gripper_open_client if command == 'open' else self.debug_gripper_close_client
        if command not in ('open', 'close'):
            return {'success': False, 'message': 'gripper command must be open or close'}
        return self._call_trigger_client(
            client,
            f'/spectrometer_cell/debug/gripper_{command}',
            timeout_sec=15.0)

    def debug_brush(self, body):
        if not self._debug_operation_lock.acquire(blocking=False):
            return {'success': False, 'message': 'another debug operation is running'}
        try:
            return self._debug_brush_locked(body)
        finally:
            self._debug_operation_lock.release()

    def _debug_brush_locked(self, body):
        with self._debug_lock:
            if not self._debug['active']:
                return {'success': False, 'message': 'debug mode is not active'}
        enabled = body.get('enabled', False)
        if not isinstance(enabled, bool):
            return {'success': False, 'message': 'brush enabled must be true or false'}
        try:
            speed = float(body.get('speed_percent', 0.0))
        except (TypeError, ValueError):
            return {'success': False, 'message': 'brush speed is invalid'}
        if not math.isfinite(speed) or speed < 0.0 or speed > 100.0:
            return {'success': False, 'message': 'brush speed must be between 0% and 100%'}
        if enabled and speed < 1.0:
            return {'success': False, 'message': 'brush speed must be at least 1% when enabled'}
        request = SetBrush.Request()
        request.enabled = enabled
        request.speed_percent = speed
        response, error = self._call_service_request(
            self.debug_brush_client,
            request,
            '/spectrometer_cell/debug/set_brush',
            timeout_sec=8.0)
        if not response:
            return {'success': False, 'message': error}
        result = {
            'success': bool(response.success),
            'message': response.message,
            'enabled': bool(response.enabled),
            'applied_speed_percent': float(response.applied_speed_percent),
        }
        if result['success']:
            with self._debug_lock:
                self._debug['brush_enabled'] = result['enabled']
                self._debug['brush_speed_percent'] = speed
            if body.get('persist_default') and speed >= 1.0:
                _, config = self._load_point_config()
                old_duty = int(config.get('cleaning', {}).get('motor_rs485_duty_permille', 500))
                duty = (-1 if old_duty < 0 else 1) * int(round(speed * 10.0))
                saved = self.save_point_config(
                    {'config': {'cleaning': {'motor_rs485_duty_permille': duty}}},
                    allow_debug=True)
                result['persist_result'] = saved
                if not saved.get('success'):
                    result['message'] += f"; default save failed: {saved.get('message')}"
        return result

    def _load_motion_catalog(self):
        if not os.path.isfile(self.motion_catalog_path):
            raise FileNotFoundError(self.motion_catalog_path)
        with open(self.motion_catalog_path, 'r', encoding='utf-8') as file:
            catalog = yaml.safe_load(file)
        references = _validate_motion_catalog_document(catalog)
        return catalog, references

    @staticmethod
    def _atomic_write_text(path, text):
        directory = os.path.dirname(os.path.abspath(path))
        temporary_path = os.path.join(
            directory,
            f'.{os.path.basename(path)}.tmp.{os.getpid()}.{threading.get_ident()}')
        try:
            with open(temporary_path, 'w', encoding='utf-8', newline='\n') as file:
                file.write(text)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary_path, path)
            try:
                directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except (AttributeError, OSError):
                pass
        finally:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)

    def get_motion_catalog(self):
        try:
            catalog, references = self._load_motion_catalog()
            return {
                'success': True,
                'message': 'motion catalog loaded',
                'path': self.motion_catalog_path,
                'catalog': catalog,
                'references': references,
                'reload_available': bool(self.motion_reload_client.service_is_ready()),
            }
        except Exception as exc:
            return {
                'success': False,
                'message': f'load motion catalog failed: {exc}',
                'path': self.motion_catalog_path,
                'catalog': {},
                'references': {},
            }

    def _debug_session_active(self):
        with self._debug_lock:
            return bool(self._debug['active'])

    def save_motion_catalog(self, body, allow_debug=False, pose_updates=None):
        if self._debug_session_active() and not allow_debug:
            return {
                'success': False,
                'message': 'point catalog changes are locked during an active debug session',
            }
        catalog = body.get('catalog', body)
        try:
            references = _validate_motion_catalog_document(catalog)
        except Exception as exc:
            return {
                'success': False,
                'message': f'motion catalog validation failed: {exc}',
            }

        if not self.motion_reload_client.service_is_ready():
            return {
                'success': False,
                'message': (
                    'motion server reload service is not ready; '
                    'catalog was not changed because compile validation is required'),
            }

        try:
            write_path = os.path.realpath(self.motion_catalog_path)
            with open(write_path, 'r', encoding='utf-8') as file:
                previous_text = file.read()
            if pose_updates:
                candidate_text = previous_text
                for point_name, pose in pose_updates.items():
                    candidate_text = _replace_yaml_value(
                        candidate_text,
                        ['points', point_name, 'pose', 'xyz'],
                        pose['xyz'])
                    candidate_text = _replace_yaml_value(
                        candidate_text,
                        ['points', point_name, 'pose', 'rpy'],
                        pose['rpy'])
                catalog = yaml.safe_load(candidate_text)
                references = _validate_motion_catalog_document(catalog)
            else:
                candidate_text = yaml.safe_dump(
                    catalog,
                    allow_unicode=True,
                    sort_keys=False,
                    default_flow_style=False)
            backup_path = (
                f'{write_path}.bak_'
                f'{time.strftime("%Y%m%d_%H%M%S")}')
            shutil.copy2(write_path, backup_path)
            self._atomic_write_text(write_path, candidate_text)

            reload_result = self._call_trigger_client(
                self.motion_reload_client,
                self.motion_reload_service_name,
                timeout_sec=30.0)
            if not reload_result.get('success'):
                self._atomic_write_text(write_path, previous_text)
                rollback_result = self._call_trigger_client(
                    self.motion_reload_client,
                    self.motion_reload_service_name,
                    timeout_sec=30.0)
                return {
                    'success': False,
                    'message': (
                        'candidate compile/reload failed; previous catalog restored: '
                        f"{reload_result.get('message', 'unknown error')}"),
                    'path': write_path,
                    'backup_path': backup_path,
                    'rollback_result': rollback_result,
                }

            self.get_logger().warn(
                f'motion catalog saved and compiled path={write_path} '
                f'backup={backup_path}')
            return {
                'success': True,
                'message': 'motion catalog saved, compiled, and atomically activated',
                'path': write_path,
                'backup_path': backup_path,
                'references': references,
                'reload_result': reload_result,
            }
        except Exception as exc:
            return {
                'success': False,
                'message': f'save motion catalog failed: {exc}',
                'path': self.motion_catalog_path,
            }

    def reload_motion_catalog(self):
        if self._debug_session_active():
            return {
                'success': False,
                'message': 'point catalog reload is locked during an active debug session',
            }
        return self._call_trigger_client(
            self.motion_reload_client,
            self.motion_reload_service_name,
            timeout_sec=30.0)

    def _load_point_config(self):
        if not os.path.isfile(self.point_config_path):
            raise FileNotFoundError(self.point_config_path)
        with open(self.point_config_path, 'r', encoding='utf-8') as file:
            text = file.read()
        return text, _parse_simple_yaml(text)

    def get_point_config(self):
        try:
            _, config = self._load_point_config()
            return {
                'success': True,
                'message': 'point config loaded',
                'path': self.point_config_path,
                'config': {
                    'named_poses': config.get('named_poses', {}),
                    'motion': {
                        key: config.get('motion', {}).get(key)
                        for key in sorted(POINT_CONFIG_MOTION_KEYS)
                        if key in config.get('motion', {})
                    },
                    'spectrometer_axis': {
                        key: config.get('spectrometer_axis', {}).get(key)
                        for key in sorted(POINT_CONFIG_AXIS_KEYS)
                        if key in config.get('spectrometer_axis', {})
                    },
                    'cleaning': {
                        key: config.get('cleaning', {}).get(key)
                        for key in sorted(POINT_CONFIG_CLEANING_KEYS)
                        if key in config.get('cleaning', {})
                    },
                },
                'reload_available': bool(self.reload_config_client.service_is_ready()),
            }
        except Exception as exc:
            return {
                'success': False,
                'message': f'load point config failed: {exc}',
                'path': self.point_config_path,
                'config': {},
            }

    def save_point_config(self, body, allow_debug=False):
        if self._debug_session_active() and not allow_debug:
            return {
                'success': False,
                'message': 'process config changes are locked during an active debug session',
            }
        try:
            updates = body.get('config', body)
            if not isinstance(updates, dict):
                return {'success': False, 'message': 'invalid point config payload'}

            text, current = self._load_point_config()
            updated_text = text
            changed = []

            named_poses = updates.get('named_poses', {})
            if named_poses is not None and not isinstance(named_poses, dict):
                return {'success': False, 'message': 'named_poses must be an object'}
            for pose_name, pose in (named_poses or {}).items():
                if pose_name not in current.get('named_poses', {}):
                    return {'success': False, 'message': f'unknown named pose: {pose_name}'}
                if not isinstance(pose, dict):
                    return {'success': False, 'message': f'named pose must be object: {pose_name}'}
                if 'xyz' in pose:
                    xyz = _as_float_list(pose['xyz'], 3, f'named_poses.{pose_name}.xyz')
                    updated_text = _replace_yaml_value(updated_text, ['named_poses', pose_name, 'xyz'], xyz)
                    changed.append(f'named_poses.{pose_name}.xyz')
                if 'rpy' in pose:
                    rpy = _as_float_list(pose['rpy'], 3, f'named_poses.{pose_name}.rpy')
                    updated_text = _replace_yaml_value(updated_text, ['named_poses', pose_name, 'rpy'], rpy)
                    changed.append(f'named_poses.{pose_name}.rpy')

            motion = updates.get('motion', {})
            if motion is not None and not isinstance(motion, dict):
                return {'success': False, 'message': 'motion must be an object'}
            for key, value in (motion or {}).items():
                if key not in POINT_CONFIG_MOTION_KEYS:
                    return {'success': False, 'message': f'motion key is not editable from HMI: {key}'}
                number = float(value)
                if not math.isfinite(number):
                    return {'success': False, 'message': f'motion.{key} must be finite'}
                updated_text = _replace_yaml_value(updated_text, ['motion', key], number)
                changed.append(f'motion.{key}')

            axis = updates.get('spectrometer_axis', {})
            if axis is not None and not isinstance(axis, dict):
                return {'success': False, 'message': 'spectrometer_axis must be an object'}
            for key, value in (axis or {}).items():
                if key not in POINT_CONFIG_AXIS_KEYS:
                    return {'success': False, 'message': f'spectrometer_axis key is not editable from HMI: {key}'}
                if key in ('place_offset_xyz', 'pick_offset_xyz'):
                    value = _as_float_list(value, 3, f'spectrometer_axis.{key}')
                elif key == 'axis':
                    value = str(value).strip().lower()
                    if value not in ('x', 'y', 'z'):
                        return {'success': False, 'message': 'spectrometer_axis.axis must be x, y, or z'}
                else:
                    value = float(value)
                    if not math.isfinite(value):
                        return {'success': False, 'message': f'spectrometer_axis.{key} must be finite'}
                updated_text = _replace_yaml_value(updated_text, ['spectrometer_axis', key], value)
                changed.append(f'spectrometer_axis.{key}')

            cleaning = updates.get('cleaning', {})
            if cleaning is not None and not isinstance(cleaning, dict):
                return {'success': False, 'message': 'cleaning must be an object'}
            for key, value in (cleaning or {}).items():
                if key not in POINT_CONFIG_CLEANING_KEYS:
                    return {'success': False, 'message': f'cleaning key is not editable from HMI: {key}'}
                if key in (
                    'pour_wrist_joint_index',
                    'pour_direction',
                    'shake_count',
                    'brush_stroke_count',
                    'motor_rs485_baudrate',
                    'motor_rs485_slave_id',
                    'motor_rs485_duty_permille',
                    'motor_rs485_communication_timeout_ds',
                ):
                    value = int(value)
                    if key == 'pour_direction' and value not in (-1, 0, 1):
                        return {'success': False, 'message': 'cleaning.pour_direction must be -1, 0, or 1'}
                    if key == 'motor_rs485_baudrate' and value <= 0:
                        return {
                            'success': False,
                            'message': 'cleaning.motor_rs485_baudrate must be > 0',
                        }
                    if key == 'motor_rs485_slave_id' and value not in range(1, 128):
                        return {
                            'success': False,
                            'message': 'cleaning.motor_rs485_slave_id must be in [1, 127]',
                        }
                    if key == 'motor_rs485_duty_permille' and (
                        value == 0 or not -1000 <= value <= 1000
                    ):
                        return {
                            'success': False,
                            'message': (
                                'cleaning.motor_rs485_duty_permille must be non-zero '
                                'in [-1000, 1000]'
                            ),
                        }
                    if key == 'motor_rs485_communication_timeout_ds' and (
                        value not in range(1, 256)
                    ):
                        return {
                            'success': False,
                            'message': (
                                'cleaning.motor_rs485_communication_timeout_ds must '
                                'be in [1, 255]'
                            ),
                        }
                elif key in ('brush_enabled', 'motor_rs485_enabled'):
                    value = _as_bool(value, f'cleaning.{key}')
                elif key in ('brush_pose', 'motor_rs485_device'):
                    value = str(value).strip()
                    if not value:
                        return {'success': False, 'message': f'cleaning.{key} must not be empty'}
                    if key == 'brush_pose' and value not in current.get('named_poses', {}):
                        return {'success': False, 'message': f'cleaning.brush_pose is not a named pose: {value}'}
                elif key in (
                    'brush_approach_offset_xyz',
                    'brush_upright_retreat_offset_xyz',
                    'brush_stroke_offset_xyz',
                ):
                    value = _as_float_list(value, 3, f'cleaning.{key}')
                else:
                    value = float(value)
                    if not math.isfinite(value):
                        return {'success': False, 'message': f'cleaning.{key} must be finite'}
                    if key in ('brush_velocity_scale', 'brush_acceleration_scale') and value <= 0:
                        return {'success': False, 'message': f'cleaning.{key} must be > 0'}
                    if key in ('brush_hold_sec', 'brush_motor_stop_delay_sec') and value < 0:
                        return {'success': False, 'message': f'cleaning.{key} must be >= 0'}
                updated_text = _replace_yaml_value(updated_text, ['cleaning', key], value)
                changed.append(f'cleaning.{key}')

            if not changed:
                return {
                    'success': True,
                    'message': 'no point config changes',
                    'path': self.point_config_path,
                    'changed': [],
                    'reload_available': bool(self.reload_config_client.service_is_ready()),
                }

            write_path = os.path.realpath(self.point_config_path)
            backup_path = f'{write_path}.bak_{time.strftime("%Y%m%d_%H%M%S")}'
            shutil.copy2(write_path, backup_path)
            self._atomic_write_text(write_path, updated_text)

            self.get_logger().warn(
                f'point config saved by HMI path={write_path} backup={backup_path} '
                f'changed={",".join(changed)}')
            reload_result = self.reload_point_config(allow_debug=allow_debug)
            reload_success = bool(reload_result.get('success'))
            if reload_success:
                message = f'saved {len(changed)} fields and reloaded runtime config'
            else:
                self._atomic_write_text(write_path, text)
                rollback_reload = self.reload_point_config(allow_debug=allow_debug)
                return {
                    'success': False,
                    'message': (
                        'runtime reload failed; restored previous config: '
                        f"{reload_result.get('message', 'unknown error')}"),
                    'path': write_path,
                    'backup_path': backup_path,
                    'changed': changed,
                    'reload_result': reload_result,
                    'rollback_reload_result': rollback_reload,
                    'runtime_reloaded': False,
                }
            return {
                'success': True,
                'message': message,
                'path': write_path,
                'backup_path': backup_path,
                'changed': changed,
                'reload_result': reload_result,
                'runtime_reloaded': reload_success,
                'reload_available': bool(self.reload_config_client.service_is_ready()),
            }
        except Exception as exc:
            return {
                'success': False,
                'message': f'save point config failed: {exc}',
                'path': self.point_config_path,
            }

    def reload_point_config(self, allow_debug=False):
        if self._debug_session_active() and not allow_debug:
            return {
                'success': False,
                'message': 'process config reload is locked during an active debug session',
            }
        result = self._call_trigger_client(
            self.reload_config_client,
            self.reload_config_service_name,
            timeout_sec=5.0)
        self.get_logger().warn(
            f"point config reload requested success={result.get('success')} "
            f"message={result.get('message')}")
        return result

    def call_command(self, command):
        if command == 'simulate_estop':
            return self.call_emergency_stop()

        if self._debug_session_active() and command not in (
                'manual_mode', 'clear_estop', 'request_reset',
                'restart_cleaning_motor'):
            return {
                'success': False,
                'message': 'production commands are locked during an active point-debug session',
            }

        state_error = self._production_command_state_error(command)
        if state_error:
            return {'success': False, 'message': state_error}

        if command in self.workflow_commands:
            return self.call_workflow(self.workflow_commands[command])

        if command == 'restart_cleaning_motor':
            return self.restart_cleaning_motor()

        if command not in self.command_clients:
            return {'success': False, 'message': f'unknown command: {command}'}

        client = self.command_clients[command]
        service_name = self.command_service_names[command]
        if not client.wait_for_service(timeout_sec=3.0):
            return {
                'success': False,
                'message': f'ROS service not ready: {service_name}',
            }

        event = threading.Event()
        result_holder = {}
        future = client.call_async(Trigger.Request())

        def _done(done_future):
            try:
                response = done_future.result()
                result_holder['result'] = {
                    'success': bool(response.success),
                    'message': response.message,
                    'service': service_name,
                }
            except Exception as exc:
                result_holder['result'] = {
                    'success': False,
                    'message': f'ROS service call failed: {exc}',
                    'service': service_name,
                }
            event.set()

        future.add_done_callback(_done)

        if not event.wait(timeout=5.0):
            return {
                'success': False,
                'message': f'ROS service call timeout: {service_name}',
            }

        return result_holder['result']

    def restart_cleaning_motor(self):
        result = self._call_trigger_client(
            self.command_clients['restart_cleaning_motor'],
            self.command_service_names['restart_cleaning_motor'],
            timeout_sec=10.0)
        if not result.get('success'):
            return result

        cell = self.state_store.snapshot().get('spectrometer_cell', {})
        context = cell.get('context') or {}
        empty_error = (
            cell.get('state') == 'ERROR' and
            not context.get('has_active_task') and
            not context.get('cup_in_gripper') and
            not context.get('spectrometer_occupied'))
        if not empty_error:
            return result

        reset = self._call_trigger_client(
            self.command_clients['request_reset'],
            self.command_service_names['request_reset'],
            timeout_sec=5.0)
        return {
            'success': bool(reset.get('success')),
            'message': (
                f"{result.get('message', 'cleaning motor restored')}; "
                f"{reset.get('message', 'state reset failed')}"),
            'service': result.get('service'),
        }

    def external_command_authorized(self, supplied_token):
        required = str(self.external_command_token)
        return not required or hmac.compare_digest(required, str(supplied_token))

    def _load_external_command_journal(self):
        try:
            with open(
                    self.external_command_journal_path,
                    encoding='utf-8') as stream:
                document = json.load(stream)
            records = document.get('records', [])
            if document.get('version') != 1 or not isinstance(records, list):
                raise ValueError('unsupported external command journal')
            loaded = {}
            for record in records[-EXTERNAL_JOURNAL_LIMIT:]:
                if not isinstance(record, dict):
                    continue
                request_id = record.get('request_id')
                command = record.get('command')
                if (not isinstance(request_id, str) or
                        not EXTERNAL_REQUEST_ID_PATTERN.fullmatch(request_id) or
                        command not in EXTERNAL_COMMANDS):
                    continue
                item = dict(record)
                if item.get('status') == 'processing':
                    item.update({
                        'accepted': False,
                        'status': 'uncertain',
                        'message': (
                            'previous process stopped before acknowledgement; '
                            'inspect live workcell state and use a new request_id'),
                    })
                loaded[request_id] = item
            return loaded
        except FileNotFoundError:
            return {}
        except Exception as exc:
            self.get_logger().error(
                f'external command journal ignored: {exc}')
            return {}

    def _save_external_command_journal(self):
        os.makedirs(
            os.path.dirname(os.path.abspath(self.external_command_journal_path)),
            exist_ok=True)
        while len(self._external_command_records) > EXTERNAL_JOURNAL_LIMIT:
            del self._external_command_records[next(iter(
                self._external_command_records))]
        document = {
            'version': 1,
            'records': list(self._external_command_records.values()),
        }
        self._atomic_write_text(
            self.external_command_journal_path,
            json.dumps(document, ensure_ascii=False, indent=2) + '\n')

    def _external_live_state(self):
        cell = self.state_store.snapshot().get('spectrometer_cell', {})
        context = cell.get('context') or {}
        return {
            'state': cell.get('state', 'UNKNOWN'),
            'cycle_id': context.get('cycle_id'),
            'active_outlet': context.get('outlet', 'NONE'),
            'has_active_task': bool(context.get('has_active_task')),
        }

    def external_command_status(self, request_id=''):
        live = self._external_live_state()
        if not request_id:
            return {
                'success': True,
                'api_version': '1.0',
                'commands': list(EXTERNAL_COMMANDS),
                'idempotency': 'request_id is retained for the latest 256 commands',
                'workcell': live,
            }, 200
        if not EXTERNAL_REQUEST_ID_PATTERN.fullmatch(str(request_id)):
            return {
                'success': False,
                'message': 'request_id must be 1-64 letters, digits, dot, colon, dash or underscore',
            }, 400
        with self._external_command_lock:
            record = self._external_command_records.get(request_id)
            if record is None:
                return {
                    'success': False,
                    'request_id': request_id,
                    'message': 'request_id not found',
                    'workcell': live,
                }, 404
            result = dict(record)
        result.update({'success': True, 'duplicate': True, 'workcell': live})
        return result, 200

    def call_external_command(self, body):
        request_id = str(body.get('request_id', '')).strip()
        command = str(body.get('command', '')).strip().upper()
        if not EXTERNAL_REQUEST_ID_PATTERN.fullmatch(request_id):
            return {
                'success': False,
                'message': 'request_id must be 1-64 letters, digits, dot, colon, dash or underscore',
            }, 400
        if command not in EXTERNAL_COMMANDS:
            return {
                'success': False,
                'request_id': request_id,
                'message': f'unsupported command: {command}',
                'commands': list(EXTERNAL_COMMANDS),
            }, 400

        with self._external_command_lock:
            previous = self._external_command_records.get(request_id)
            if previous is not None:
                if previous.get('command') != command:
                    return {
                        'success': False,
                        'request_id': request_id,
                        'message': 'request_id was already used for another command',
                        'workcell': self._external_live_state(),
                    }, 409
                result = dict(previous)
                accepted = bool(previous.get('accepted'))
                result.update({
                    'success': accepted,
                    'duplicate': True,
                    'workcell': self._external_live_state(),
                })
                return result, 200 if accepted else 409

            record = {
                'api_version': '1.0',
                'request_id': request_id,
                'command': command,
                'accepted': False,
                'duplicate': False,
                'status': 'processing',
                'received_time_ms': int(time.time() * 1000),
            }
            self._external_command_records[request_id] = record
            try:
                self._save_external_command_journal()
            except Exception as exc:
                del self._external_command_records[request_id]
                return {
                    'success': False,
                    'request_id': request_id,
                    'message': f'cannot persist request before execution: {exc}',
                }, 503

            try:
                ros_command = EXTERNAL_COMMANDS[command]
                command_result = (
                    {'success': True, 'message': 'external command interface ready'}
                    if ros_command is None else self.call_command(ros_command))
            except Exception as exc:
                command_result = {
                    'success': False,
                    'message': f'external command execution failed: {exc}',
                }

            record.update({
                'accepted': bool(command_result.get('success')),
                'status': (
                    'accepted' if command_result.get('success') else 'rejected'),
                'message': command_result.get('message', ''),
                'completed_time_ms': int(time.time() * 1000),
            })
            if command_result.get('service'):
                record['service'] = command_result['service']
            journal_error = ''
            try:
                self._save_external_command_journal()
            except Exception as exc:
                journal_error = str(exc)
                record['journal_error'] = journal_error
                self.get_logger().error(
                    f'external command result journal failed: {exc}')
            result = dict(record)
            result.update({
                'success': bool(record['accepted']),
                'workcell': self._external_live_state(),
            })
            if journal_error:
                result['message'] += (
                    '; result persistence failed, do not retry automatically')
            return result, 200 if record['accepted'] else 409

    def _production_command_state_error(self, command):
        cycle_commands = {
            'outlet_1_done',
            'outlet_2_done',
            'actual_cycle_outlet_1',
            'actual_cycle_outlet_2',
        }
        if command not in cycle_commands and command != 'detection_done':
            return ''

        cell = self.state_store.snapshot().get('spectrometer_cell', {})
        state = cell.get('state', 'UNKNOWN')
        context = cell.get('context') or {}
        active_task = bool(context.get('has_active_task'))
        paused_from = context.get('paused_from_state', 'UNKNOWN')

        if command in cycle_commands:
            allowed = state in ('IDLE', 'WAIT_DISCHARGE') or (
                state == 'PAUSED' and not active_task and
                paused_from in ('IDLE', 'WAIT_DISCHARGE'))
            if not allowed:
                return (
                    f'{command} rejected in {state}: '
                    'a new cycle may only start while waiting for discharge')
            return ''

        allowed = state == 'WAIT_DETECTION_DONE' or (
            state == 'PAUSED' and active_task and
            paused_from == 'WAIT_DETECTION_DONE')
        if not allowed:
            return (
                f'detection_done rejected in {state}: '
                'the cell is not waiting for detection completion')
        return ''

    def call_emergency_stop(self):
        calls = [
            ('state_estop', self.command_clients.get('simulate_estop'), self.command_service_names.get('simulate_estop')),
            ('spectrometer_stop_motion', self.spectrometer_stop_motion_client, self.spectrometer_stop_motion_service_name),
            ('workflow_stop', self.workflow_stop_client, self.workflow_stop_service_name),
        ]
        pending = []
        results = {}
        for label, client, service_name in calls:
            if client is None or not service_name:
                results[label] = 'FAIL client missing'
                continue

            if not client.wait_for_service(timeout_sec=0.05):
                results[label] = f'FAIL service not ready: {service_name}'
                continue

            try:
                future = client.call_async(Trigger.Request())
                pending.append((label, service_name, future))
                results[label] = 'PENDING'
            except Exception as exc:
                results[label] = f'FAIL call failed: {exc}'

        deadline = time.time() + 1.0
        for label, service_name, future in pending:
            while not future.done() and time.time() < deadline:
                time.sleep(0.01)
            if not future.done():
                results[label] = f'SENT no response yet: {service_name}'
                continue
            try:
                response = future.result()
                results[label] = f"{'OK' if response.success else 'FAIL'} {response.message}"
            except Exception as exc:
                results[label] = f'FAIL response error: {exc}'

        any_success = any(value.startswith('OK') or value.startswith('SENT') for value in results.values())
        message = '; '.join(f'{label}: {result}' for label, result in results.items())
        self.get_logger().error(f'EMERGENCY STOP requested from HMI: {message}')
        return {
            'success': any_success,
            'message': message,
            'service': 'multi_stop',
            'results': results,
        }

    def call_workflow(self, workflow_name):
        if not self.workflow_client.wait_for_service(timeout_sec=3.0):
            return {
                'success': False,
                'message': f'ROS service not ready: {self.workflow_service_name}',
            }

        event = threading.Event()
        result_holder = {}
        request = RunWorkflow.Request()
        request.workflow_name = workflow_name
        request.dry_run = False
        future = self.workflow_client.call_async(request)

        def _done(done_future):
            try:
                response = done_future.result()
                result_holder['result'] = {
                    'success': bool(response.success),
                    'message': response.message,
                    'service': self.workflow_service_name,
                    'workflow': workflow_name,
                }
            except Exception as exc:
                result_holder['result'] = {
                    'success': False,
                    'message': f'ROS workflow service call failed: {exc}',
                    'service': self.workflow_service_name,
                    'workflow': workflow_name,
                }
            event.set()

        future.add_done_callback(_done)

        if not event.wait(timeout=5.0):
            return {
                'success': False,
                'message': f'ROS workflow service call timeout: {workflow_name}',
                'service': self.workflow_service_name,
                'workflow': workflow_name,
            }

        return result_holder['result']

    def call_speed_scale(self, body):
        try:
            scale = float(body.get('scale', body.get('speed_scale', 1.0)))
        except (TypeError, ValueError):
            return {'success': False, 'message': 'invalid speed scale'}

        if not math.isfinite(scale) or scale < 0.20 or scale > 1.00:
            return {'success': False, 'message': 'speed scale must be between 20% and 100%'}
        if self._debug_session_active():
            return {
                'success': False,
                'message': 'speed changes are locked during an active point-debug session',
            }
        if not self.speed_scale_client.wait_for_service(timeout_sec=2.0):
            return {
                'success': False,
                'message': f'ROS service not ready: {self.speed_scale_service_name}',
            }

        event = threading.Event()
        result_holder = {}
        request = SetSpeedScale.Request()
        request.scale = scale
        future = self.speed_scale_client.call_async(request)

        def _done(done_future):
            try:
                response = done_future.result()
                applied = float(response.applied_scale)
                self.state_store.set_motion_speed_scale(applied)
                result_holder['result'] = {
                    'success': bool(response.success),
                    'message': response.message,
                    'service': self.speed_scale_service_name,
                    'applied_scale': applied,
                    'applied_percent': int(round(applied * 100.0)),
                }
            except Exception as exc:
                result_holder['result'] = {
                    'success': False,
                    'message': f'ROS speed scale call failed: {exc}',
                    'service': self.speed_scale_service_name,
                }
            event.set()

        future.add_done_callback(_done)
        if not event.wait(timeout=3.0):
            return {
                'success': False,
                'message': f'ROS speed scale call timeout: {self.speed_scale_service_name}',
                'service': self.speed_scale_service_name,
            }
        return result_holder['result']

    def restart_camera(self):
        script = self.camera_restart_script
        if not os.path.isfile(script):
            return {'success': False, 'message': f'camera restart script not found: {script}'}
        if not os.access(script, os.X_OK):
            return {'success': False, 'message': f'camera restart script is not executable: {script}'}

        log_dir = os.path.join(os.path.expanduser('~'), 'panthera_workcell_ws', 'validation_logs')
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f'camera_restart_{time.strftime("%Y%m%d_%H%M%S")}.log')
        try:
            log_file = open(log_path, 'ab')
            subprocess.Popen(
                [script],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True)
            self.get_logger().warn(f'camera restart requested, log={log_path}')
            return {'success': True, 'message': f'camera restart started: {log_path}', 'log': log_path}
        except Exception as exc:
            return {'success': False, 'message': f'camera restart failed: {exc}', 'log': log_path}

    def _call_trigger_client(self, client, service_name, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {
                'success': False,
                'message': f'ROS service not ready: {service_name}',
                'service': service_name,
            }

        event = threading.Event()
        result_holder = {}
        future = client.call_async(Trigger.Request())

        def _done(done_future):
            try:
                response = done_future.result()
                result_holder['result'] = {
                    'success': bool(response.success),
                    'message': response.message,
                    'service': service_name,
                }
            except Exception as exc:
                result_holder['result'] = {
                    'success': False,
                    'message': f'ROS service call failed: {exc}',
                    'service': service_name,
                }
            event.set()

        future.add_done_callback(_done)
        if not event.wait(timeout=timeout_sec):
            return {
                'success': False,
                'message': f'ROS service call timeout: {service_name}',
                'service': service_name,
            }
        return result_holder['result']

    def get_camera_payload(self, channel):
        payload, content_type, error = self.camera_store.get_payload(channel)
        if payload is None:
            return _placeholder_bmp(640, 360, error or 'no image received'), 'image/bmp'
        return payload, content_type

    def wait_for_camera_payload(self, channel, last_stamp):
        return self.camera_store.wait_for_payload(channel, last_stamp, timeout=1.0)

    def shutdown_http(self):
        self.camera_store.shutdown()
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WebHmiNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.shutdown_http()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
