from copy import deepcopy
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml

from panthera_web_hmi.web_hmi_node import WebHmiNode
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
    node._debug = {'active': True, 'phase': 'ready', 'last_message': ''}
    node._debug_require_paused = lambda: (True, 'ready')
    node._settled_tool_pose = lambda: ([0.42, -0.08, 0.19], [0.2, -0.3, 1.1])

    result = node.debug_move_to({
        'target_xyz_m': [0.42, -0.08, 0.19],
        'target_rpy_rad': [0.2, -0.3, 1.1],
    })

    assert result['success'] is True
    assert result['already_at_target'] is True
    assert node._debug['phase'] == 'ready'


def test_debug_target_corrects_submillimeter_residual():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_lock = threading.Lock()
    node._debug = {
        'commanded_pose': {'xyz': [0.42, -0.08, 0.19], 'rpy': [0.0, 0.0, 0.0]},
        'history': [],
    }
    measured = iter([
        ([0.42, -0.08, 0.19025], [0.0, 0.0, 0.0]),
        ([0.42, -0.08, 0.190], [0.0, 0.0, 0.0]),
        ([0.42, -0.08, 0.191], [0.0, 0.0, 0.0]),
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
    assert result['within_step_tolerance'] is True


def test_debug_jog_rejects_unrepeatable_one_millimeter_step():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {'active': True, 'phase': 'ready'}
    node._debug_require_paused = lambda: (True, 'ready')

    result = node.debug_jog({
        'translation_m': [0.0, 0.0, 0.001],
        'rotation_rad': [0.0, 0.0, 0.0],
    })

    assert result['success'] is False
    assert '2mm' in result['message']


def test_direct_coordinate_move_rejects_more_than_20mm():
    node = WebHmiNode.__new__(WebHmiNode)
    node._debug_operation_lock = threading.Lock()
    node._debug_lock = threading.Lock()
    node._debug = {'active': True, 'phase': 'ready', 'last_message': ''}
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
