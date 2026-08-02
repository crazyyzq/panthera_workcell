from copy import deepcopy
from http.server import BaseHTTPRequestHandler
import inspect
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml

from panthera_web_hmi.web_hmi_node import WebHmiNode
from panthera_web_hmi.web_hmi_node import HmiRequestHandler
from panthera_web_hmi.web_hmi_node import _validate_motion_catalog_document
from panthera_web_hmi.web_hmi_node import cartesian_pose_delta
from panthera_web_hmi.web_hmi_node import cartesian_pose_target
from panthera_web_hmi.web_hmi_node import compensated_command_target
from panthera_web_hmi.web_hmi_node import rpy_orientation_error


CATALOG_PATH = (
    Path(__file__).parents[2] /
    'panthera_motion' /
    'config' /
    'motion_catalog.yaml'
)


def load_catalog():
    with CATALOG_PATH.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


@pytest.mark.parametrize('disconnect', [BrokenPipeError, ConnectionResetError])
def test_http_client_disconnect_is_ignored(monkeypatch, disconnect):
    handler = HmiRequestHandler.__new__(HmiRequestHandler)
    monkeypatch.setattr(
        BaseHTTPRequestHandler,
        'handle',
        lambda _self: (_ for _ in ()).throw(disconnect()))

    handler.handle()


def test_hmi_exposes_no_zero_control():
    static_root = Path(__file__).parents[1] / 'static'
    assert 'reference_zero' not in inspect.getsource(HmiRequestHandler.do_POST)
    assert 'debugReferenceZero' not in (
        static_root / 'index.html').read_text(encoding='utf-8')
    assert 'reference_zero' not in (
        static_root / 'assets' / 'app.js').read_text(encoding='utf-8')


def test_hmi_separates_debug_entry_from_point_motion():
    static_root = Path(__file__).parents[1] / 'static'
    html = (static_root / 'index.html').read_text(encoding='utf-8')
    javascript = (static_root / 'assets' / 'app.js').read_text(encoding='utf-8')

    assert '进入调试（不移动）' in html
    assert 'id="debugGoto"' in html
    assert "'/api/debug/enter', {}" in javascript
    assert "'/api/debug/goto', {point_name: point}" in javascript


def test_debug_entry_enables_tools_without_arm_motion():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {'active': False, 'phase': 'inactive'}
    node.state_store = SimpleNamespace(snapshot=lambda: {
        'spectrometer_cell': {'state': 'PAUSED', 'context': {}},
    })
    node._execute_motion_route = lambda _route: pytest.fail('enter moved the arm')

    result = node.enter_debug({})

    assert result == {
        'success': True,
        'message': 'debug mode entered without robot motion',
        'robot_moved': False,
    }
    assert node._debug['active'] is True
    assert node._debug['selected_point'] == ''
    assert node._debug['target_reached'] is False


def test_debug_tools_do_not_require_a_selected_point():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True,
        'phase': 'ready',
        'selected_point': '',
        'target_reached': False,
        'brush_enabled': False,
    }
    node.debug_gripper_open_client = object()
    node.debug_gripper_close_client = object()
    node.debug_brush_client = object()
    node._call_trigger_client = lambda *_args, **_kwargs: {'success': True}
    node._call_service_request = lambda *_args, **_kwargs: (
        SimpleNamespace(
            success=True, message='brush started', enabled=True,
            applied_speed_percent=50.0), '')

    assert node._debug_gripper_locked({'command': 'close'})['success'] is True
    assert node._debug_brush_locked(
        {'enabled': True, 'speed_percent': 50})['success'] is True


def test_debug_exit_without_a_point_does_not_move_the_arm():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True,
        'phase': 'ready',
        'selected_point': '',
        'target_reached': False,
        'history': [],
        'brush_enabled': True,
    }
    node.debug_brush_client = object()
    node._call_service_request = lambda *_args, **_kwargs: (None, '')
    node._execute_motion_route = lambda _route: pytest.fail('exit moved the arm')

    result = node.exit_debug()

    assert result['success'] is True
    assert result['robot_moved'] is False
    assert node._debug['active'] is False
    assert node._debug['brush_enabled'] is False


def test_every_tunable_point_has_a_safe_round_trip():
    catalog = load_catalog()
    _validate_motion_catalog_document(catalog)

    safe_point = catalog['commissioning']['safe_point']
    tunable = {
        name for name, point in catalog['points'].items()
        if 'tunable' in point.get('tags', [])
    }
    assert tunable == {
        'outlet_1_grasp',
        'outlet_2_grasp',
        'spectrometer_place',
        'spectrometer_pick',
        'spectrometer_wait',
        'clean_dump',
        'brush_center',
    }

    for name in tunable:
        entry = catalog['routes'][f'debug_safe_to_{name}']
        exit_route = catalog['routes'][f'debug_{name}_to_safe']
        assert entry['start'] == safe_point
        assert entry['segments'][-1]['to'] == name
        assert exit_route['start'] == name
        assert exit_route['segments'][-1]['to'] == safe_point


def test_invalid_follower_axis_is_rejected():
    catalog = deepcopy(load_catalog())
    catalog['commissioning']['translation_followers']['outlet_1_grasp'][0]['axes'] = 'xq'

    with pytest.raises(ValueError, match='follower axes'):
        _validate_motion_catalog_document(catalog)


def test_atomic_config_write_replaces_complete_file(tmp_path):
    target = tmp_path / 'config.yaml'
    target.write_text('old: true\n', encoding='utf-8')

    WebHmiNode._atomic_write_text(str(target), 'new: true\n')

    assert target.read_text(encoding='utf-8') == 'new: true\n'
    assert not list(tmp_path.glob('.config.yaml.tmp.*'))


@pytest.mark.parametrize('scale', [-0.1, 0.19, 1.01, float('nan'), 'bad'])
def test_hmi_rejects_speed_outside_operator_range(scale):
    node = WebHmiNode.__new__(WebHmiNode)

    result = node.call_speed_scale({'scale': scale})

    assert result['success'] is False


def test_cleaning_motor_restart_resets_empty_error():
    node = WebHmiNode.__new__(WebHmiNode)
    node.command_clients = {
        'restart_cleaning_motor': object(),
        'request_reset': object(),
    }
    node.command_service_names = {
        'restart_cleaning_motor': '/spectrometer_cell/restart_cleaning_motor',
        'request_reset': '/spectrometer_cell/request_reset',
    }
    node.state_store = SimpleNamespace(snapshot=lambda: {
        'spectrometer_cell': {
            'state': 'ERROR',
            'context': {
                'has_active_task': False,
                'cup_in_gripper': False,
                'spectrometer_occupied': False,
            },
        },
    })
    calls = []

    def call(_client, service, timeout_sec):
        calls.append((service, timeout_sec))
        return {'success': True, 'message': f'{service} ok', 'service': service}

    node._call_trigger_client = call

    result = node.restart_cleaning_motor()

    assert result['success'] is True
    assert calls == [
        ('/spectrometer_cell/restart_cleaning_motor', 10.0),
        ('/spectrometer_cell/request_reset', 5.0),
    ]


def test_cleaning_motor_restart_preserves_active_cycle_error():
    node = WebHmiNode.__new__(WebHmiNode)
    node.command_clients = {
        'restart_cleaning_motor': object(),
        'request_reset': object(),
    }
    node.command_service_names = {
        'restart_cleaning_motor': '/spectrometer_cell/restart_cleaning_motor',
        'request_reset': '/spectrometer_cell/request_reset',
    }
    node.state_store = SimpleNamespace(snapshot=lambda: {
        'spectrometer_cell': {
            'state': 'ERROR',
            'context': {
                'has_active_task': True,
                'cup_in_gripper': True,
                'spectrometer_occupied': False,
            },
        },
    })
    calls = []
    node._call_trigger_client = lambda _client, service, timeout_sec: (
        calls.append((service, timeout_sec)) or
        {'success': True, 'message': 'motor restored', 'service': service})

    result = node.restart_cleaning_motor()

    assert result['success'] is True
    assert calls == [('/spectrometer_cell/restart_cleaning_motor', 10.0)]


def test_equivalent_rpy_representations_have_no_orientation_error():
    assert rpy_orientation_error(
        (0.0, 0.0, 0.0),
        (-3.141592653589793, 3.141592653589793, 3.141592653589793),
    ) == pytest.approx(0.0, abs=1e-12)


def test_cartesian_pose_delta_round_trips_multi_axis_target():
    start_xyz = [0.42, -0.08, 0.19]
    start_rpy = [0.2, -0.3, 1.1]
    requested = [0.004, -0.003, 0.002, 0.03, -0.02, 0.04]

    target_xyz, target_rpy = cartesian_pose_target(
        start_xyz, start_rpy, requested)
    recovered = cartesian_pose_delta(
        start_xyz, start_rpy, target_xyz, target_rpy)

    assert recovered[:3] == pytest.approx(requested[:3], abs=1e-12)
    assert rpy_orientation_error(recovered[3:], requested[3:]) == pytest.approx(
        0.0, abs=1e-7)


def test_compensated_target_preserves_loaded_command_bias():
    measured_xyz = [0.473, -0.097, 0.197]
    commanded_xyz = [0.469, -0.096, 0.191]
    measured_rpy = [-0.004, -0.014, 0.001]
    commanded_rpy = [-0.001, -0.005, 0.000]
    physical_target_xyz = [0.474, -0.098, 0.198]
    physical_target_rpy = [-0.004, -0.014, 0.009]

    target_xyz, target_rpy = compensated_command_target(
        measured_xyz,
        measured_rpy,
        commanded_xyz,
        commanded_rpy,
        physical_target_xyz,
        physical_target_rpy,
    )

    assert target_xyz == pytest.approx([0.470, -0.097, 0.192], abs=1e-12)
    command_delta = cartesian_pose_delta(
        commanded_xyz, commanded_rpy, target_xyz, target_rpy)
    physical_delta = cartesian_pose_delta(
        measured_xyz, measured_rpy, physical_target_xyz, physical_target_rpy)
    assert command_delta[:3] == pytest.approx(physical_delta[:3], abs=1e-12)
    assert rpy_orientation_error(command_delta[3:], physical_delta[3:]) == pytest.approx(
        0.0, abs=1e-7)


def test_direct_coordinate_move_accepts_current_pose_without_motion():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True, 'phase': 'ready', 'last_message': '',
        'selected_point': 'clean_dump', 'target_reached': True,
    }
    node._debug_require_paused = lambda: (True, 'ready')
    node._settled_tool_pose = lambda: ([0.42, -0.08, 0.19], [0.2, -0.3, 1.1])

    result = node.debug_move_to({
        'target_xyz_m': [0.42, -0.08, 0.19],
        'target_rpy_rad': [0.2, -0.3, 1.1],
    })

    assert result['success'] is True
    assert result['already_at_target'] is True
    assert node._debug['phase'] == 'ready'


def test_debug_target_corrects_residual_once_when_it_converges():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'commanded_pose': {'xyz': [0.42, -0.08, 0.19], 'rpy': [0.0, 0.0, 0.0]},
        'history': [],
    }
    measured = iter([
        ([0.42, -0.08, 0.19025], [0.0, 0.0, 0.0]),
        ([0.42, -0.08, 0.1907], [0.0, 0.0, 0.0]),
    ])
    node._settled_tool_pose = lambda: next(measured)
    node._stage_and_execute_jog = lambda _delta, target: {
        'success': True,
        'target_xyz_m': list(target[0]),
        'target_rpy_rad': list(target[1]),
    }

    result = node._execute_debug_target(
        [0.42, -0.08, 0.19],
        [0.0, 0.0, 0.0],
        [0.42, -0.08, 0.191],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
    )

    assert result['correction_count'] == 1
    assert result['within_step_tolerance'] is True


def test_debug_target_stops_after_two_consecutive_regressions():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'commanded_pose': {'xyz': [0.42, -0.08, 0.19], 'rpy': [0.0, 0.0, 0.0]},
        'history': [],
    }
    measured = iter([
        ([0.42, -0.08, 0.19025], [0.0, 0.0, 0.0]),
        ([0.42, -0.08, 0.1900], [0.0, 0.0, 0.0]),
        ([0.42, -0.08, 0.1898], [0.0, 0.0, 0.0]),
    ])
    node._settled_tool_pose = lambda: next(measured)
    node._stage_and_execute_jog = lambda _delta, target: {
        'success': True,
        'target_xyz_m': list(target[0]),
        'target_rpy_rad': list(target[1]),
    }

    result = node._execute_debug_target(
        [0.42, -0.08, 0.19],
        [0.0, 0.0, 0.0],
        [0.42, -0.08, 0.191],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
    )

    assert result['correction_count'] == 2
    assert result['convergence_stopped'] is True


def test_debug_jog_rejects_unrepeatable_one_millimeter_step():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True, 'phase': 'ready',
        'selected_point': 'clean_dump', 'target_reached': True,
    }
    node._debug_require_paused = lambda: (True, 'ready')

    result = node.debug_jog({
        'translation_m': [0.0, 0.0, 0.001],
        'rotation_rad': [0.0, 0.0, 0.0],
    })

    assert result['success'] is False
    assert '2mm' in result['message']


def test_debug_jog_requires_a_selected_point():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True, 'phase': 'ready',
        'selected_point': '', 'target_reached': False,
    }
    node._debug_require_paused = lambda: (True, 'ready')

    result = node.debug_jog({
        'translation_m': [0.002, 0.0, 0.0],
        'rotation_rad': [0.0, 0.0, 0.0],
    })

    assert result['success'] is False
    assert result['message'] == 'debug point is not ready'


def test_debug_exit_rewinds_successful_jogs_in_reverse_order():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'phase': 'error',
        'history': [
            [0.005, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.020, 0.0, 0.0, 0.0, 0.0],
        ],
    }
    calls = []

    def execute(delta):
        calls.append(delta)
        return {
            'success': True,
            'target_xyz_m': [0.0, 0.0, 0.0],
            'target_rpy_rad': [0.0, 0.0, 0.0],
        }

    node._stage_and_execute_jog = execute

    result = node._rewind_debug_history()

    assert result['success'] is True
    assert calls == [
        [0.0, -0.020, 0.0, 0.0, 0.0, 0.0],
        [-0.005, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    assert node._debug['history'] == []


def test_debug_save_preserves_translation_follower_orientation():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True,
        'phase': 'ready',
        'selected_point': 'clean_dump',
        'target_reached': True,
        'commanded_pose': {
            'xyz': [0.10, -0.34, 0.26],
            'rpy': [0.1, 0.0, -1.57],
        },
    }
    catalog = {
        'points': {
            'clean_dump': {
                'pose': {'xyz': [0.09, -0.34, 0.25], 'rpy': [0.0, 0.0, -1.57]},
            },
            'clean_dump_pour': {
                'pose': {'xyz': [0.09, -0.34, 0.25], 'rpy': [-2.5, 0.0, -1.57]},
            },
            'clean_dump_hover': {
                'pose': {'xyz': [0.09, -0.34, 0.45], 'rpy': [0.0, 0.0, -1.57]},
            },
        },
        'commissioning': {
            'translation_followers': {
                'clean_dump': [
                    {'point': 'clean_dump_pour', 'axes': 'xyz'},
                    {'point': 'clean_dump_hover', 'axes': 'xy'},
                ],
            },
        },
    }
    node._debug_catalog_target = lambda _name: (
        deepcopy(catalog), None,
        deepcopy(catalog['points']['clean_dump']), None, None)
    saved = {}

    def save(body, **kwargs):
        saved.update(body['catalog'])
        return {'success': True, 'message': 'saved'}

    node.save_motion_catalog = save

    result = node.save_debug_point()

    assert result['success'] is True
    follower = saved['points']['clean_dump_pour']['pose']
    assert follower['xyz'] == pytest.approx([0.10, -0.34, 0.26])
    assert follower['rpy'] == pytest.approx([-2.5, 0.0, -1.57])
    hover = saved['points']['clean_dump_hover']['pose']
    assert hover['xyz'] == pytest.approx([0.10, -0.34, 0.45])
    assert hover['rpy'] == pytest.approx([0.1, 0.0, -1.57])


def test_spectrometer_place_save_keeps_laser_exit_vertical():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    catalog = load_catalog()
    place = catalog['points']['spectrometer_place']['pose']
    target_xyz = [place['xyz'][0] + 0.015, place['xyz'][1] - 0.010,
                  place['xyz'][2] + 0.003]
    node._debug = {
        'active': True,
        'phase': 'ready',
        'selected_point': 'spectrometer_place',
        'target_reached': True,
        'commanded_pose': {'xyz': target_xyz, 'rpy': list(place['rpy'])},
    }
    node._debug_catalog_target = lambda _name: (
        deepcopy(catalog), None,
        deepcopy(catalog['points']['spectrometer_place']), None, None)
    saved = {}
    node.save_motion_catalog = lambda body, **_kwargs: (
        saved.update(body['catalog']) or {'success': True, 'message': 'saved'})

    result = node.save_debug_point()

    assert result['success'] is True
    points = saved['points']
    assert points['spectrometer_preplace']['pose']['xyz'][:2] == pytest.approx(
        target_xyz[:2])
    assert points['spectrometer_hover']['pose']['xyz'][:2] == pytest.approx(
        target_xyz[:2])
    assert points['spectrometer_sensor_hover_template']['pose']['xyz'][:2] == (
        pytest.approx(target_xyz[:2]))


def test_debug_save_retries_transient_motion_reload_failure(tmp_path):
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {'active': True}
    node.motion_catalog_path = str(tmp_path / 'motion_catalog.yaml')
    catalog = load_catalog()
    Path(node.motion_catalog_path).write_text(
        yaml.safe_dump(catalog, allow_unicode=True, sort_keys=False),
        encoding='utf-8')
    node.motion_reload_client = object()
    node.motion_reload_service_name = '/motion/reload'
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    calls = []

    def reload(_client, service_name, timeout_sec):
        calls.append((service_name, timeout_sec))
        if len(calls) == 1:
            return {
                'success': False,
                'message': 'ROS service not ready: /motion/reload',
            }
        return {'success': True, 'message': 'motion catalog reloaded'}

    node._call_trigger_client = reload
    result = node.save_motion_catalog(
        {'catalog': catalog},
        allow_debug=True,
        pose_updates={
            'spectrometer_place': catalog['points']['spectrometer_place']['pose'],
        })

    assert result['success'] is True
    assert calls == [('/motion/reload', 30.0), ('/motion/reload', 30.0)]


def test_failed_debug_entry_stays_available_for_manual_jog():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {}
    node._settled_tool_pose = lambda timeout_sec: (
        [0.31, -0.22, 0.27], [-2.5, 0.0, -1.57])

    result = node._continue_debug_from_current_pose(
        'clean_dump', {'success': False, 'message': 'entry route not reached'})

    assert result['success'] is True
    assert result['warning'] is True
    assert node._debug['active'] is True
    assert node._debug['phase'] == 'ready'
    assert node._debug['selected_point'] == 'clean_dump'
    assert node._debug['target_reached'] is False
    assert node._debug['commanded_pose']['xyz'] == pytest.approx([0.31, -0.22, 0.27])


def test_debug_jog_executes_one_load_compensated_axis_route():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True, 'phase': 'ready',
        'selected_point': 'clean_dump', 'target_reached': False,
    }
    node._debug_require_paused = lambda: (True, 'ready')
    node._fresh_tool_pose = lambda: ([0.42, -0.08, 0.19], [0.0, 0.0, 0.0])
    calls = []
    node._execute_debug_target = lambda *args, **kwargs: (
        calls.append((args, kwargs)) or {'success': True})

    result = node.debug_jog({
        'translation_m': [0.002, 0.0, 0.0],
        'rotation_rad': [0.0, 0.0, 0.0],
    })

    assert result['success'] is True
    assert len(calls) == 1
    assert calls[0][1]['max_corrections'] == 0
    assert calls[0][1]['strict_axis'] is True


def test_strict_axis_jog_uses_relative_holding_path():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'commanded_pose': {
            'xyz': [0.420, -0.080, 0.190],
            'rpy': [0.0, 0.0, 0.0],
        },
        'history': [],
    }
    calls = []

    def stage(delta, absolute_target=None):
        calls.append((list(delta), absolute_target))
        return {
            'success': True,
            'target_xyz_m': [0.419, -0.082, 0.187],
            'target_rpy_rad': [0.001, -0.002, 0.0],
        }

    node._stage_and_execute_jog = stage
    node._fresh_tool_pose = lambda: (
        [0.419, -0.082, 0.187], [0.001, -0.002, 0.0])

    result = node._execute_debug_target(
        [0.417, -0.082, 0.187],
        [0.001, -0.002, 0.0],
        [0.419, -0.082, 0.187],
        [0.001, -0.002, 0.0],
        [0.002, 0.0, 0.0, 0.0, 0.0, 0.0],
        max_corrections=0,
        strict_axis=True,
    )

    assert len(calls) == 1
    assert calls[0][0] == pytest.approx([0.002, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert calls[0][1] is None
    assert result['orthogonal_drift_m'] == pytest.approx(0.0)
    assert node._debug['commanded_pose']['xyz'] == pytest.approx(
        [0.419, -0.082, 0.187])


def test_axis_correction_does_not_move_coordinates_already_in_tolerance():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'commanded_pose': {
            'xyz': [0.0, 0.0, 0.0],
            'rpy': [0.0, 0.0, 0.0],
        },
        'history': [],
    }
    calls = []

    def stage(delta, absolute_target=None):
        calls.append((list(delta), absolute_target))
        return {
            'success': True,
            'target_xyz_m': list(absolute_target[0]),
            'target_rpy_rad': list(absolute_target[1]),
        }

    samples = iter((
        ([0.0005, -0.0004, -0.0002], [0.0, 0.0, 0.0]),
        ([0.002, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ))
    node._stage_and_execute_jog = stage
    node._settled_tool_pose = lambda: next(samples)

    result = node._execute_debug_target(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.002, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.002, 0.0, 0.0, 0.0, 0.0, 0.0],
        max_corrections=1,
        strict_axis=False,
    )

    assert result['within_step_tolerance'] is True
    assert len(calls) == 2
    assert calls[1][1][0][1:] == pytest.approx([0.0, 0.0])


def test_direct_coordinate_move_rejects_more_than_20mm():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {
        'active': True, 'phase': 'ready', 'last_message': '',
        'selected_point': 'clean_dump', 'target_reached': True,
    }
    node._debug_require_paused = lambda: (True, 'ready')
    node._settled_tool_pose = lambda: ([0.42, -0.08, 0.19], [0.2, -0.3, 1.1])

    result = node.debug_move_to({
        'target_xyz_m': [0.45, -0.08, 0.19],
        'target_rpy_rad': [0.2, -0.3, 1.1],
    })

    assert result['success'] is False
    assert '20 mm' in result['message']
    assert node._debug['phase'] == 'ready'


def test_active_debug_session_rejects_production_commands_and_speed_changes():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {'active': True}

    command = node.call_command('auto_mode')
    speed = node.call_speed_scale({'scale': 0.5})

    assert command['success'] is False
    assert 'locked' in command['message']
    assert speed['success'] is False
    assert 'locked' in speed['message']


@pytest.mark.parametrize(
    ('command', 'state', 'context', 'allowed'),
    [
        ('actual_cycle_outlet_1', 'WAIT_DISCHARGE', {}, True),
        (
            'actual_cycle_outlet_1',
            'PAUSED',
            {'has_active_task': True, 'paused_from_state': 'PICK_FROM_OUTLET'},
            False,
        ),
        ('detection_done', 'WAIT_DISCHARGE', {}, False),
        (
            'detection_done',
            'PAUSED',
            {'has_active_task': True, 'paused_from_state': 'WAIT_DETECTION_DONE'},
            True,
        ),
    ],
)
def test_production_signals_are_gated_by_cell_state(
        command, state, context, allowed):
    node = WebHmiNode.__new__(WebHmiNode)
    node.state_store = SimpleNamespace(
        snapshot=lambda: {
            'spectrometer_cell': {
                'state': state,
                'context': context,
            },
        },
    )

    error = node._production_command_state_error(command)

    assert (error == '') is allowed


def external_node(tmp_path):
    node = WebHmiNode.__new__(WebHmiNode)
    node.external_command_token = ''
    node.external_command_journal_path = str(tmp_path / 'external_commands.json')
    node._external_command_lock = threading.Lock()
    node._external_command_records = {}
    node.state_store = SimpleNamespace(
        snapshot=lambda: {
            'spectrometer_cell': {
                'state': 'WAIT_DISCHARGE',
                'context': {
                    'cycle_id': 7,
                    'outlet': 'NONE',
                    'has_active_task': False,
                },
            },
        },
    )
    return node


def test_external_command_is_idempotent_and_persisted(tmp_path):
    node = external_node(tmp_path)
    calls = []
    node.call_command = lambda command: (
        calls.append(command) or {'success': True, 'message': 'queued'})

    first, first_status = node.call_external_command({
        'request_id': 'plc-42',
        'command': 'OUTLET_1_DISCHARGE_DONE',
    })
    duplicate, duplicate_status = node.call_external_command({
        'request_id': 'plc-42',
        'command': 'OUTLET_1_DISCHARGE_DONE',
    })

    assert first_status == duplicate_status == 200
    assert first['accepted'] is True
    assert duplicate['duplicate'] is True
    assert calls == ['actual_cycle_outlet_1']
    assert (tmp_path / 'external_commands.json').is_file()


def test_external_request_id_cannot_be_reused_for_another_command(tmp_path):
    node = external_node(tmp_path)
    node.call_command = lambda _command: {'success': True, 'message': 'queued'}
    node.call_external_command({
        'request_id': 'plc-43',
        'command': 'OUTLET_1_DISCHARGE_DONE',
    })

    result, status = node.call_external_command({
        'request_id': 'plc-43',
        'command': 'DETECTION_DONE',
    })

    assert status == 409
    assert result['success'] is False


def workcell_control_node(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    (scripts / 'start_workcell.sh').write_text('#!/usr/bin/env bash\n', encoding='utf-8')
    (scripts / 'stop_workcell.sh').write_text('#!/usr/bin/env bash\n', encoding='utf-8')
    (scripts / 'restart_workcell.sh').write_text('#!/usr/bin/env bash\n', encoding='utf-8')
    node = WebHmiNode.__new__(WebHmiNode)
    node.workcell_workspace = str(tmp_path)
    node._workcell_control_lock = threading.Lock()
    node._workcell_process = None
    node._workcell_control = {
        'busy': False,
        'operation': '',
        'message': 'ready',
        'last_exit_code': None,
        'last_log': '',
        'started_at': None,
        'completed_at': None,
    }
    return node


def test_workcell_control_rejects_concurrent_operation(tmp_path):
    node = workcell_control_node(tmp_path)
    node._workcell_control.update({'busy': True, 'operation': 'stop'})

    result = node.request_workcell_operation('start')

    assert result['success'] is False
    assert '请勿重复操作' in result['message']


def test_workcell_restart_runs_safe_stop_before_start(tmp_path, monkeypatch):
    node = workcell_control_node(tmp_path)
    commands = []

    class CompletedProcess:
        def poll(self):
            return 0

    def fake_popen(command, **_kwargs):
        commands.append(Path(command[1]).name)
        return CompletedProcess()

    monkeypatch.setattr(
        'panthera_web_hmi.web_hmi_node.subprocess.Popen', fake_popen)

    accepted = node.request_workcell_operation('restart')
    status = node.workcell_control_snapshot()

    assert accepted['success'] is True
    assert commands == ['restart_workcell.sh']
    assert status['busy'] is False
    assert status['last_exit_code'] == 0


def test_workcell_start_passes_selected_control_mode(tmp_path, monkeypatch):
    node = workcell_control_node(tmp_path)
    environments = []

    class RunningProcess:
        def poll(self):
            return None

    def fake_popen(_command, **kwargs):
        environments.append(kwargs['env'])
        return RunningProcess()

    monkeypatch.setattr(
        'panthera_web_hmi.web_hmi_node.subprocess.Popen', fake_popen)

    result = node.request_workcell_operation(
        'start', {'control_mode': 'mit_gravity_compensation'})

    assert result['success'] is True
    assert result['control_mode'] == 'mit_gravity_compensation'
    assert environments[0]['CONTROL_MODE'] == 'mit_gravity_compensation'


def test_workcell_start_rejects_unknown_control_mode(tmp_path):
    node = workcell_control_node(tmp_path)

    result = node.request_workcell_operation(
        'start', {'control_mode': 'unknown'})

    assert result['success'] is False
    assert '不支持' in result['message']


def test_rejected_external_command_remains_rejected_when_retried(tmp_path):
    node = external_node(tmp_path)
    node.call_command = lambda _command: {
        'success': False,
        'message': 'wrong state',
    }
    body = {
        'request_id': 'plc-45',
        'command': 'DETECTION_DONE',
    }

    first, first_status = node.call_external_command(body)
    duplicate, duplicate_status = node.call_external_command(body)

    assert first_status == duplicate_status == 409
    assert first['success'] is duplicate['success'] is False
    assert duplicate['duplicate'] is True


def test_external_processing_record_becomes_uncertain_after_restart(tmp_path):
    node = external_node(tmp_path)
    node._external_command_records['plc-44'] = {
        'request_id': 'plc-44',
        'command': 'OUTLET_2_DISCHARGE_DONE',
        'status': 'processing',
    }
    node._save_external_command_journal()

    restarted = external_node(tmp_path)
    restarted._external_command_records = restarted._load_external_command_journal()
    result, status = restarted.external_command_status('plc-44')

    assert status == 200
    assert result['accepted'] is False
    assert result['status'] == 'uncertain'
