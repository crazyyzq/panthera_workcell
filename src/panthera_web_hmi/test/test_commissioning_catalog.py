from copy import deepcopy
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml

from panthera_web_hmi.web_hmi_node import WebHmiNode
from panthera_web_hmi.web_hmi_node import _validate_motion_catalog_document
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


def test_equivalent_rpy_representations_have_no_orientation_error():
    assert rpy_orientation_error(
        (0.0, 0.0, 0.0),
        (-3.141592653589793, 3.141592653589793, 3.141592653589793),
    ) == pytest.approx(0.0, abs=1e-12)


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
