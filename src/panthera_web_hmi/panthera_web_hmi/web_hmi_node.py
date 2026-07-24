import json
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

from panthera_interfaces.msg import ExternalSignal, LaserDistance, WorkflowStatus
from panthera_interfaces.srv import RunWorkflow, SetSpeedScale


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
    lines[idx] = f'{indent}{path[-1]}: {_format_yaml_scalar(value)}'
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
            'pose_tuner': {
                'targets': [],
                'last_result': None,
                'log': [],
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

    def set_pose_tuner_targets(self, targets):
        with self._lock:
            self._data['pose_tuner']['targets'] = list(targets)

    def append_pose_tuner_log(self, entry):
        with self._lock:
            log = self._data['pose_tuner'].setdefault('log', [])
            log.insert(0, dict(entry))
            del log[80:]
            self._data['pose_tuner']['last_result'] = dict(entry)

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

    def log_message(self, fmt, *args):
        try:
            message = fmt % args
        except TypeError:
            message = fmt
        self.server.bridge_node.get_logger().debug(message)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/api/status':
            self._send_json(self.server.bridge_node.snapshot())
            return
        if parsed.path == '/api/point_config':
            self._send_json(self.server.bridge_node.get_point_config())
            return
        if parsed.path == '/api/motion_catalog':
            self._send_json(self.server.bridge_node.get_motion_catalog())
            return
        if parsed.path == '/api/pose_tuner/list':
            self._send_json(self.server.bridge_node.call_pose_tuner_list())
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
            '/api/state_request',
            '/api/speed_scale',
            '/api/camera/restart',
            '/api/point_config',
            '/api/point_config/reload',
            '/api/motion_catalog',
            '/api/motion_catalog/reload',
            '/api/pose_tuner/run',
            '/api/pose_tuner/stop',
        ):
            self._send_json({'success': False, 'message': 'unknown endpoint'}, status=404)
            return

        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        try:
            body = json.loads(raw)
        except json.JSONDecodeError:
            self._send_json({'success': False, 'message': 'invalid json'}, status=400)
            return

        if parsed.path == '/api/state_request':
            result = self.server.bridge_node.call_state_request(body)
        elif parsed.path == '/api/speed_scale':
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
        elif parsed.path == '/api/pose_tuner/run':
            result = self.server.bridge_node.call_pose_tuner_run(body)
        elif parsed.path == '/api/pose_tuner/stop':
            result = self.server.bridge_node.call_pose_tuner_stop(body)
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
            'manual_mode': self.declare_parameter(
                'service_manual_mode',
                '/spectrometer_cell/manual_mode').value,
            'auto_mode': self.declare_parameter(
                'service_auto_mode',
                '/spectrometer_cell/auto_mode').value,
            'step_once': self.declare_parameter(
                'service_step_once',
                '/spectrometer_cell/step_once').value,
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
        self.pose_tuner_list_service_name = self.declare_parameter(
            'pose_tuner_list_service',
            '/pose_tuner/list_targets').value
        self.pose_tuner_run_service_name = self.declare_parameter(
            'pose_tuner_run_service',
            '/pose_tuner/run_target').value
        self.pose_tuner_stop_service_name = self.declare_parameter(
            'pose_tuner_stop_service',
            '/pose_tuner/stop').value
        self.pose_tuner_list_client = self.create_client(Trigger, self.pose_tuner_list_service_name)
        self.pose_tuner_run_client = self.create_client(RunWorkflow, self.pose_tuner_run_service_name)
        self.pose_tuner_stop_client = self.create_client(Trigger, self.pose_tuner_stop_service_name)
        self.spectrometer_stop_motion_service_name = self.declare_parameter(
            'spectrometer_stop_motion_service',
            '/spectrometer_cell/stop_motion').value
        self.spectrometer_stop_motion_client = self.create_client(
            Trigger,
            self.spectrometer_stop_motion_service_name)
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
        services['pose_tuner_list'] = {
            'service': self.pose_tuner_list_service_name,
            'ready': bool(self.pose_tuner_list_client.service_is_ready()),
        }
        services['pose_tuner_run'] = {
            'service': self.pose_tuner_run_service_name,
            'ready': bool(self.pose_tuner_run_client.service_is_ready()),
        }
        services['pose_tuner_stop'] = {
            'service': self.pose_tuner_stop_service_name,
            'ready': bool(self.pose_tuner_stop_client.service_is_ready()),
        }
        services['spectrometer_stop_motion'] = {
            'service': self.spectrometer_stop_motion_service_name,
            'ready': bool(self.spectrometer_stop_motion_client.service_is_ready()),
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
        self.state_store.set_services(services)

    def snapshot(self):
        self._update_service_status()
        data = self.state_store.snapshot()
        data['camera']['debug'] = self.camera_store.stats()
        return data

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

    def save_motion_catalog(self, body):
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
            with open(self.motion_catalog_path, 'r', encoding='utf-8') as file:
                previous_text = file.read()
            candidate_text = yaml.safe_dump(
                catalog,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False)
            backup_path = (
                f'{self.motion_catalog_path}.bak_'
                f'{time.strftime("%Y%m%d_%H%M%S")}')
            shutil.copy2(self.motion_catalog_path, backup_path)
            self._atomic_write_text(self.motion_catalog_path, candidate_text)

            reload_result = self._call_trigger_client(
                self.motion_reload_client,
                self.motion_reload_service_name,
                timeout_sec=30.0)
            if not reload_result.get('success'):
                self._atomic_write_text(self.motion_catalog_path, previous_text)
                rollback_result = self._call_trigger_client(
                    self.motion_reload_client,
                    self.motion_reload_service_name,
                    timeout_sec=30.0)
                return {
                    'success': False,
                    'message': (
                        'candidate compile/reload failed; previous catalog restored: '
                        f"{reload_result.get('message', 'unknown error')}"),
                    'path': self.motion_catalog_path,
                    'backup_path': backup_path,
                    'rollback_result': rollback_result,
                }

            self.get_logger().warn(
                f'motion catalog saved and compiled path={self.motion_catalog_path} '
                f'backup={backup_path}')
            return {
                'success': True,
                'message': 'motion catalog saved, compiled, and atomically activated',
                'path': self.motion_catalog_path,
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

    def save_point_config(self, body):
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

            backup_path = f'{self.point_config_path}.bak_{time.strftime("%Y%m%d_%H%M%S")}'
            shutil.copy2(self.point_config_path, backup_path)
            with open(self.point_config_path, 'w', encoding='utf-8', newline='\n') as file:
                file.write(updated_text)

            self.get_logger().warn(
                f'point config saved by HMI path={self.point_config_path} backup={backup_path} '
                f'changed={",".join(changed)}')
            reload_result = self.reload_point_config()
            reload_success = bool(reload_result.get('success'))
            if reload_success:
                message = f'saved {len(changed)} fields and reloaded runtime config'
            else:
                message = (
                    f'saved {len(changed)} fields, but runtime reload failed: '
                    f"{reload_result.get('message', 'unknown error')}")
            return {
                'success': True,
                'message': message,
                'path': self.point_config_path,
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

    def reload_point_config(self):
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

        if command in self.workflow_commands:
            return self.call_workflow(self.workflow_commands[command])

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

    def call_emergency_stop(self):
        calls = [
            ('state_estop', self.command_clients.get('simulate_estop'), self.command_service_names.get('simulate_estop')),
            ('spectrometer_stop_motion', self.spectrometer_stop_motion_client, self.spectrometer_stop_motion_service_name),
            ('workflow_stop', self.workflow_stop_client, self.workflow_stop_service_name),
            ('pose_tuner_stop', self.pose_tuner_stop_client, self.pose_tuner_stop_service_name),
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

    def call_state_request(self, body):
        target_state = str(body.get('target_state', '')).strip().upper()
        reason = str(body.get('reason', '')).strip()
        force = bool(body.get('force', False))
        source = str(body.get('source', 'web_hmi')).strip() or 'web_hmi'

        known_states = {
            'INIT',
            'IDLE',
            'WAIT_DISCHARGE',
            'SELECT_TASK',
            'PICK_FROM_OUTLET',
            'MEASURE_SPECTROMETER_BEFORE_PLACE',
            'PLACE_TO_SPECTROMETER',
            'START_DETECTION',
            'WAIT_DETECTION_DONE',
            'MEASURE_SPECTROMETER_BEFORE_PICK',
            'PICK_FROM_SPECTROMETER',
            'CLEAN_CUP',
            'RETURN_CUP',
            'COMPLETE_CYCLE',
            'PAUSED',
            'ESTOP',
            'ERROR',
            'RESET',
        }
        action_states = {
            'PICK_FROM_OUTLET',
            'MEASURE_SPECTROMETER_BEFORE_PLACE',
            'PLACE_TO_SPECTROMETER',
            'START_DETECTION',
            'MEASURE_SPECTROMETER_BEFORE_PICK',
            'PICK_FROM_SPECTROMETER',
            'CLEAN_CUP',
            'RETURN_CUP',
        }

        if target_state not in known_states:
            return {
                'success': False,
                'accepted': False,
                'message': f'unknown target_state: {target_state}',
            }
        if not reason:
            return {
                'success': False,
                'accepted': False,
                'message': 'state request rejected: reason is required',
            }

        snapshot = self.snapshot()
        cell = snapshot.get('spectrometer_cell', {})
        context = cell.get('context', {}) if isinstance(cell.get('context', {}), dict) else {}
        current_state = cell.get('state') or context.get('state') or 'UNKNOWN'
        has_active_task = bool(context.get('has_active_task', False))

        self.get_logger().warn(
            f'state request from={source} current={current_state} target={target_state} '
            f'force={force} reason={reason}')

        if target_state in action_states:
            return {
                'success': False,
                'accepted': False,
                'from_state': current_state,
                'target_state': target_state,
                'message': (
                    'state request rejected: action states must be reached by the state machine; '
                    'the HMI cannot jump directly into a robot action'
                ),
            }

        if current_state == 'ESTOP' and target_state not in ('RESET', 'ESTOP'):
            return {
                'success': False,
                'accepted': False,
                'from_state': current_state,
                'target_state': target_state,
                'message': 'state request rejected: ESTOP requires clear_estop then RESET first',
            }

        command = None
        if target_state == 'ESTOP':
            command = 'simulate_estop'
        elif target_state == 'RESET':
            command = 'request_reset'
        elif target_state == 'PAUSED':
            command = 'manual_mode'
        elif target_state == 'WAIT_DISCHARGE':
            command = 'auto_mode'
        elif target_state == 'IDLE':
            if current_state in ('ERROR', 'ESTOP', 'RESET'):
                command = 'request_reset'
            elif has_active_task and not force:
                return {
                    'success': False,
                    'accepted': False,
                    'from_state': current_state,
                    'target_state': target_state,
                    'message': 'state request rejected: active task exists; use force only after manual safety check',
                }
            else:
                command = 'manual_mode'
        elif target_state in ('INIT', 'ERROR', 'COMPLETE_CYCLE', 'SELECT_TASK', 'WAIT_DETECTION_DONE'):
            return {
                'success': False,
                'accepted': False,
                'from_state': current_state,
                'target_state': target_state,
                'message': f'state request rejected: {target_state} is not exposed as a manual HMI transition',
            }

        if not command:
            return {
                'success': False,
                'accepted': False,
                'from_state': current_state,
                'target_state': target_state,
                'message': f'state request rejected: no command mapping for {target_state}',
            }

        result = self.call_command(command)
        result.update({
            'accepted': bool(result.get('success')),
            'from_state': current_state,
            'target_state': target_state,
            'mapped_command': command,
            'source': source,
            'reason': reason,
            'force': force,
        })
        return result

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

        scale = max(0.20, min(1.20, scale))
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

    def call_pose_tuner_list(self):
        if not self.pose_tuner_list_client.wait_for_service(timeout_sec=2.0):
            result = {
                'success': False,
                'message': f'ROS service not ready: {self.pose_tuner_list_service_name}',
                'targets': [],
            }
            self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
            return result

        call_result = self._call_trigger_client(
            self.pose_tuner_list_client,
            self.pose_tuner_list_service_name,
            timeout_sec=5.0)
        if not call_result.get('success'):
            result = {**call_result, 'targets': []}
            self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
            return result

        targets = self._parse_pose_tuner_targets(call_result.get('message', ''))
        self.state_store.set_pose_tuner_targets(targets)
        result = {
            'success': True,
            'message': f'loaded {len(targets)} pose tuner targets',
            'targets': targets,
            'service': self.pose_tuner_list_service_name,
        }
        self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
        return result

    def call_pose_tuner_run(self, body):
        target_name = str(body.get('target_name') or body.get('target') or '').strip()
        dry_run = bool(body.get('dry_run', True))
        debug_mode = bool(body.get('debug_mode', False))
        source = str(body.get('source', 'web_hmi')).strip() or 'web_hmi'

        validation = self._validate_pose_tuner_request(target_name, dry_run, debug_mode)
        if not validation.get('success'):
            result = {
                **validation,
                'target_name': target_name,
                'dry_run': dry_run,
                'debug_mode': debug_mode,
                'source': source,
            }
            self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
            return result

        if not self.pose_tuner_run_client.wait_for_service(timeout_sec=2.0):
            result = {
                'success': False,
                'message': f'ROS service not ready: {self.pose_tuner_run_service_name}',
                'target_name': target_name,
                'dry_run': dry_run,
                'debug_mode': debug_mode,
                'source': source,
            }
            self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
            return result

        event = threading.Event()
        result_holder = {}
        request = RunWorkflow.Request()
        request.workflow_name = target_name
        request.dry_run = dry_run
        future = self.pose_tuner_run_client.call_async(request)

        def _done(done_future):
            try:
                response = done_future.result()
                result_holder['result'] = {
                    'success': bool(response.success),
                    'message': response.message,
                    'service': self.pose_tuner_run_service_name,
                    'target_name': target_name,
                    'dry_run': dry_run,
                    'debug_mode': debug_mode,
                    'source': source,
                }
            except Exception as exc:
                result_holder['result'] = {
                    'success': False,
                    'message': f'ROS pose tuner call failed: {exc}',
                    'service': self.pose_tuner_run_service_name,
                    'target_name': target_name,
                    'dry_run': dry_run,
                    'debug_mode': debug_mode,
                    'source': source,
                }
            event.set()

        future.add_done_callback(_done)
        if not event.wait(timeout=8.0 if dry_run else 60.0):
            result = {
                'success': False,
                'message': f'ROS pose tuner call timeout: {target_name}',
                'service': self.pose_tuner_run_service_name,
                'target_name': target_name,
                'dry_run': dry_run,
                'debug_mode': debug_mode,
                'source': source,
            }
        else:
            result = result_holder['result']

        self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
        self.get_logger().warn(
            f"pose_tuner_run source={source} debug_mode={debug_mode} target={target_name} "
            f"dry_run={dry_run} success={result.get('success')} message={result.get('message')}")
        return result

    def call_pose_tuner_stop(self, body):
        source = str(body.get('source', 'web_hmi')).strip() or 'web_hmi'
        result = self._call_trigger_client(
            self.pose_tuner_stop_client,
            self.pose_tuner_stop_service_name,
            timeout_sec=3.0)
        result.update({'source': source, 'target_name': 'STOP', 'dry_run': False})
        self.state_store.append_pose_tuner_log(self._pose_tuner_log_entry(result))
        self.get_logger().warn(
            f"pose_tuner_stop source={source} success={result.get('success')} "
            f"message={result.get('message')}")
        return result

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

    def _validate_pose_tuner_request(self, target_name, dry_run, debug_mode):
        if not debug_mode:
            return {
                'success': False,
                'accepted': False,
                'message': 'pose tuner rejected: debug mode is off',
            }
        if not target_name:
            return {
                'success': False,
                'accepted': False,
                'message': 'pose tuner rejected: target_name is required',
            }

        snapshot = self.snapshot()
        cell = snapshot.get('spectrometer_cell', {})
        context = cell.get('context', {}) if isinstance(cell.get('context', {}), dict) else {}
        state = cell.get('state') or context.get('state') or 'UNKNOWN'
        workflow = snapshot.get('workflow', {})
        active_action = str(context.get('active_action_name') or '')
        active_command_id = int(context.get('active_command_id') or 0)
        has_active_task = bool(context.get('has_active_task', False))
        auto_mode = bool(context.get('auto_mode', False))
        workflow_state = int(workflow.get('state') or 0)
        workflow_busy = workflow_state in (1, 2)

        if state in ('ESTOP', 'ERROR'):
            return {
                'success': False,
                'accepted': False,
                'message': f'pose tuner rejected: state={state}',
                'state': state,
            }
        if active_action or active_command_id:
            return {
                'success': False,
                'accepted': False,
                'message': f'pose tuner rejected: active action {active_action or active_command_id}',
                'state': state,
            }
        if not dry_run and (auto_mode or has_active_task or workflow_busy or state not in ('IDLE', 'PAUSED', 'WAIT_DISCHARGE')):
            return {
                'success': False,
                'accepted': False,
                'message': (
                    'pose tuner real execution rejected: robot is not idle/manual-safe '
                    f'(state={state}, auto_mode={auto_mode}, has_active_task={has_active_task})'
                ),
                'state': state,
            }
        return {'success': True, 'accepted': True, 'message': 'pose tuner request accepted'}

    def _parse_pose_tuner_targets(self, text):
        targets = []
        pattern = re.compile(
            r'^(?P<name>[^|]+)\|\s*(?P<kind>pose|joint)\s+'
            r'(?P<body>.*?)\s*\|\s*(?P<description>.*)$')
        number = r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?'
        xyz_re = re.compile(r'xyz_mm=\[(?P<xyz>[^\]]+)\]')
        rpy_re = re.compile(r'rpy_deg=\[(?P<rpy>[^\]]+)\]')
        joint_re = re.compile(r'rad=\[(?P<joints>[^\]]+)\]')

        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            match = pattern.match(line)
            if not match:
                targets.append({
                    'name': line.split('|', 1)[0].strip(),
                    'kind': 'unknown',
                    'description': line,
                })
                continue
            body = match.group('body')
            target = {
                'name': match.group('name').strip(),
                'kind': match.group('kind').strip(),
                'description': match.group('description').strip(),
            }
            xyz_match = xyz_re.search(body)
            rpy_match = rpy_re.search(body)
            joint_match = joint_re.search(body)
            if xyz_match:
                values = [float(v) for v in re.findall(number, xyz_match.group('xyz'))]
                target['xyz_mm'] = values[:3]
            if rpy_match:
                values = [float(v) for v in re.findall(number, rpy_match.group('rpy'))]
                target['rpy_deg'] = values[:3]
            if joint_match:
                target['joints_rad'] = [float(v) for v in re.findall(number, joint_match.group('joints'))]
            targets.append(target)
        return targets

    def _pose_tuner_log_entry(self, result):
        snapshot = self.state_store.snapshot()
        cell = snapshot.get('spectrometer_cell', {})
        context = cell.get('context', {}) if isinstance(cell.get('context', {}), dict) else {}
        return {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'source': result.get('source', 'web_hmi'),
            'debug_mode': bool(result.get('debug_mode', False)),
            'target_name': result.get('target_name', ''),
            'dry_run': bool(result.get('dry_run', False)),
            'success': bool(result.get('success', False)),
            'message': result.get('message', ''),
            'robot_state': cell.get('state') or context.get('state') or 'UNKNOWN',
            'active_action': context.get('active_action_name', ''),
        }

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
