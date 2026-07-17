#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from panthera_interfaces.msg import DigitalInput, DigitalOutput, ExternalSignal
from panthera_interfaces.srv import (
    GetDigitalInput,
    RunWorkflow,
    SetDigitalInput,
    SetDigitalOutput,
)


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('1', 'true', 'on', 'yes', 'active', 'high'):
        return True
    if normalized in ('0', 'false', 'off', 'no', 'inactive', 'low'):
        return False
    raise ValueError(f"invalid boolean value: {value}")


def _gpio_from_name(name: str) -> int:
    text = name.strip().upper()
    if text.startswith('GPIO'):
        text = text[4:]
    if '_' not in text:
        return int(text)

    bank_text, pin_text = text.split('_', 1)
    if not bank_text.isdigit() or len(pin_text) != 2:
        raise ValueError(f"invalid Rockchip GPIO name: {name}")

    bank = int(bank_text)
    group = ord(pin_text[0]) - ord('A')
    bit = int(pin_text[1])
    if group < 0 or group > 3 or bit < 0 or bit > 7:
        raise ValueError(f"invalid Rockchip GPIO name: {name}")
    return bank * 32 + group * 8 + bit


@dataclass
class InputConfig:
    name: str
    gpio: int
    active_mode: str
    code: int
    workflow_name: str
    detail: str

    def active_from_raw(self, raw: bool) -> bool:
        return raw if self.active_mode == 'active_high' else not raw

    def raw_from_active(self, active: bool) -> bool:
        return active if self.active_mode == 'active_high' else not active


@dataclass
class OutputConfig:
    name: str
    gpio: int
    active_mode: str
    initial_value: bool

    def raw_from_value(self, value: bool) -> bool:
        return value if self.active_mode == 'active_high' else not value

    def value_from_raw(self, raw: bool) -> bool:
        return raw if self.active_mode == 'active_high' else not raw


class SysfsGpioBackend:
    def __init__(self, root: Path = Path('/sys/class/gpio')) -> None:
        self._root = root

    def setup_input(self, gpio: int) -> None:
        self._export(gpio)
        self._write(self._gpio_path(gpio) / 'direction', 'in')

    def setup_output(self, gpio: int, initial_raw: bool) -> None:
        self._export(gpio)
        self._write(self._gpio_path(gpio) / 'direction', 'high' if initial_raw else 'low')

    def read(self, gpio: int) -> bool:
        value = self._read(self._gpio_path(gpio) / 'value').strip()
        if value not in ('0', '1'):
            raise RuntimeError(f"unexpected GPIO value for gpio{gpio}: {value}")
        return value == '1'

    def write(self, gpio: int, raw: bool) -> None:
        self._write(self._gpio_path(gpio) / 'value', '1' if raw else '0')

    def _gpio_path(self, gpio: int) -> Path:
        return self._root / f'gpio{gpio}'

    def _export(self, gpio: int) -> None:
        if self._gpio_path(gpio).exists():
            return
        self._write(self._root / 'export', str(gpio))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self._gpio_path(gpio).exists():
                return
            time.sleep(0.01)
        raise RuntimeError(f"gpio{gpio} was not exported")

    @staticmethod
    def _read(path: Path) -> str:
        try:
            return path.read_text(encoding='ascii')
        except PermissionError as exc:
            raise PermissionError(
                f"permission denied reading {path}; run as root or install GPIO udev rules"
            ) from exc

    @staticmethod
    def _write(path: Path, value: str) -> None:
        try:
            path.write_text(value, encoding='ascii')
        except PermissionError as exc:
            raise PermissionError(
                f"permission denied writing {path}; run as root or install GPIO udev rules"
            ) from exc


class MockGpioBackend:
    def __init__(self) -> None:
        self._values: Dict[int, bool] = {}

    def setup_input(self, gpio: int) -> None:
        self._values.setdefault(gpio, False)

    def setup_output(self, gpio: int, initial_raw: bool) -> None:
        self._values[gpio] = initial_raw

    def read(self, gpio: int) -> bool:
        return self._values.get(gpio, False)

    def write(self, gpio: int, raw: bool) -> None:
        self._values[gpio] = raw


class GpioIoNode(Node):
    def __init__(self) -> None:
        super().__init__('gpio_io_node')

        self.enabled = self.declare_parameter('enabled', False).value
        self.backend_name = self.declare_parameter('backend', 'sysfs').value
        self.source = self.declare_parameter('source', 'gpio_dido').value
        self.poll_rate_hz = float(self.declare_parameter('poll_rate_hz', 20.0).value)
        self.debounce_ms = int(self.declare_parameter('debounce_ms', 30).value)
        self.publish_initial_state = self.declare_parameter('publish_initial_state', True).value
        self.publish_external_signal = self.declare_parameter('publish_external_signal', True).value
        self.external_signal_topic = self.declare_parameter(
            'external_signal_topic',
            '/workflow/external_signal',
        ).value
        self.trigger_on_change = self.declare_parameter('trigger_on_change', True).value
        self.trigger_active_only = self.declare_parameter('trigger_active_only', True).value
        self.min_trigger_interval_sec = float(
            self.declare_parameter('min_trigger_interval_sec', 1.0).value
        )
        self.auto_run_workflow = self.declare_parameter('auto_run_workflow', False).value
        self.workflow_service = self.declare_parameter('workflow_service', '/run_workflow').value
        self.dry_run_workflow = self.declare_parameter('dry_run_workflow', False).value
        self.allow_output_writes = self.declare_parameter('allow_output_writes', False).value
        self.input_topic = self.declare_parameter('input_topic', '/io/digital_inputs').value
        self.output_topic = self.declare_parameter('output_topic', '/io/digital_outputs').value

        if self.poll_rate_hz < 0.0:
            raise RuntimeError('poll_rate_hz must be greater than or equal to zero')
        if self.debounce_ms < 0:
            raise RuntimeError('debounce_ms must be greater than or equal to zero')
        if self.min_trigger_interval_sec < 0.0:
            raise RuntimeError('min_trigger_interval_sec must be greater than or equal to zero')

        input_specs = list(
            self.declare_parameter('inputs', Parameter.Type.STRING_ARRAY).value or []
        )
        output_specs = list(
            self.declare_parameter('outputs', Parameter.Type.STRING_ARRAY).value or []
        )
        self.inputs = self._parse_inputs(input_specs)
        self.outputs = self._parse_outputs(output_specs)
        self._validate_unique_names_and_gpios()

        if not self.enabled:
            self.get_logger().warn(
                'GPIO IO node is disabled; using mock backend and not publishing workflow signals'
            )
            self.backend = MockGpioBackend()
        elif self.backend_name == 'sysfs':
            self.backend = SysfsGpioBackend()
        elif self.backend_name == 'mock':
            self.backend = MockGpioBackend()
        else:
            raise RuntimeError(f"unsupported backend: {self.backend_name}")

        self.input_pub = self.create_publisher(DigitalInput, self.input_topic, 10)
        self.output_pub = self.create_publisher(DigitalOutput, self.output_topic, 10)
        self.signal_pub = self.create_publisher(ExternalSignal, self.external_signal_topic, 10)
        self.workflow_client = self.create_client(RunWorkflow, self.workflow_service)

        self.get_input_srv = self.create_service(
            GetDigitalInput,
            '~/get_input',
            self._handle_get_input,
        )
        self.set_input_srv = self.create_service(
            SetDigitalInput,
            '~/set_input',
            self._handle_set_input,
        )
        self.set_output_srv = self.create_service(
            SetDigitalOutput,
            '~/set_output',
            self._handle_set_output,
        )

        self._last_input_raw: Dict[str, Optional[bool]] = {cfg.name: None for cfg in self.inputs}
        self._last_input_active: Dict[str, Optional[bool]] = {cfg.name: None for cfg in self.inputs}
        self._last_raw_change_time: Dict[str, float] = {cfg.name: 0.0 for cfg in self.inputs}
        self._last_trigger_time: Dict[str, float] = {cfg.name: 0.0 for cfg in self.inputs}
        self._output_values: Dict[str, bool] = {}

        self._setup_lines()

        if self.poll_rate_hz > 0.0:
            self.timer = self.create_timer(1.0 / self.poll_rate_hz, self._poll_inputs)
        else:
            self.timer = None

        self.get_logger().info(
            f'GPIO IO node started: enabled={str(self.enabled).lower()} '
            f'backend={self.backend_name} inputs={len(self.inputs)} outputs={len(self.outputs)} '
            f'allow_output_writes={str(self.allow_output_writes).lower()}'
        )

    def _parse_inputs(self, specs: List[str]) -> List[InputConfig]:
        result: List[InputConfig] = []
        for index, spec in enumerate(specs, 1):
            parts = spec.split(':')
            if len(parts) < 2:
                raise ValueError(f"input spec must be name:gpio..., got: {spec}")
            if not parts[0]:
                raise ValueError(f"input name must not be empty: {spec}")
            active_mode = parts[2] if len(parts) >= 3 and parts[2] else 'active_high'
            if active_mode not in ('active_high', 'active_low'):
                raise ValueError(f"invalid active mode in input spec: {spec}")
            code = int(parts[3]) if len(parts) >= 4 and parts[3] else index
            workflow_name = parts[4] if len(parts) >= 5 else ''
            detail = ':'.join(parts[5:]) if len(parts) >= 6 else ''
            result.append(
                InputConfig(
                    name=parts[0],
                    gpio=_gpio_from_name(parts[1]),
                    active_mode=active_mode,
                    code=code,
                    workflow_name=workflow_name,
                    detail=detail,
                )
            )
        return result

    def _parse_outputs(self, specs: List[str]) -> List[OutputConfig]:
        result: List[OutputConfig] = []
        for spec in specs:
            parts = spec.split(':')
            if len(parts) < 2:
                raise ValueError(f"output spec must be name:gpio..., got: {spec}")
            if not parts[0]:
                raise ValueError(f"output name must not be empty: {spec}")
            active_mode = parts[2] if len(parts) >= 3 and parts[2] else 'active_high'
            if active_mode not in ('active_high', 'active_low'):
                raise ValueError(f"invalid active mode in output spec: {spec}")
            initial_value = _parse_bool(parts[3]) if len(parts) >= 4 and parts[3] else False
            result.append(
                OutputConfig(
                    name=parts[0],
                    gpio=_gpio_from_name(parts[1]),
                    active_mode=active_mode,
                    initial_value=initial_value,
                )
            )
        return result

    def _validate_unique_names_and_gpios(self) -> None:
        names = set()
        gpios = {}
        for cfg in self.inputs:
            key = f'input:{cfg.name}'
            if key in names:
                raise ValueError(f"duplicate input name: {cfg.name}")
            names.add(key)
            if cfg.gpio in gpios:
                raise ValueError(
                    f"GPIO {cfg.gpio} is used by both {gpios[cfg.gpio]} and input {cfg.name}"
                )
            gpios[cfg.gpio] = f'input {cfg.name}'

        for cfg in self.outputs:
            key = f'output:{cfg.name}'
            if key in names:
                raise ValueError(f"duplicate output name: {cfg.name}")
            names.add(key)
            if cfg.gpio in gpios:
                raise ValueError(
                    f"GPIO {cfg.gpio} is used by both {gpios[cfg.gpio]} and output {cfg.name}"
                )
            gpios[cfg.gpio] = f'output {cfg.name}'

    def _using_mock_backend(self) -> bool:
        return isinstance(self.backend, MockGpioBackend)

    def _output_writes_enabled(self) -> bool:
        return self._using_mock_backend() or self.allow_output_writes

    def _setup_lines(self) -> None:
        for cfg in self.inputs:
            self.backend.setup_input(cfg.gpio)
            if self._using_mock_backend():
                self.backend.write(cfg.gpio, cfg.raw_from_active(False))
        for cfg in self.outputs:
            self._output_values[cfg.name] = cfg.initial_value
            if self._output_writes_enabled():
                raw = cfg.raw_from_value(cfg.initial_value)
                self.backend.setup_output(cfg.gpio, raw)
                self._publish_output(cfg, cfg.initial_value, 'initialized')
            else:
                self._publish_output(cfg, cfg.initial_value, 'output_writes_disabled')

    def _poll_inputs(self) -> None:
        now = time.monotonic()
        for cfg in self.inputs:
            try:
                raw = self.backend.read(cfg.gpio)
                previous_raw = self._last_input_raw[cfg.name]
                if previous_raw is not None and previous_raw != raw:
                    self._last_raw_change_time[cfg.name] = now
                self._last_input_raw[cfg.name] = raw

                if (
                    previous_raw is not None
                    and self.debounce_ms > 0
                    and (now - self._last_raw_change_time[cfg.name]) * 1000.0 < self.debounce_ms
                ):
                    continue

                active = cfg.active_from_raw(raw)
                previous_active = self._last_input_active[cfg.name]
                first_sample = previous_active is None
                changed = (not first_sample) and previous_active != active
                self._last_input_active[cfg.name] = active

                if self.publish_initial_state or not first_sample:
                    self._publish_input(cfg, raw, active, changed, 'ok')
                if self.enabled:
                    self._maybe_publish_external_signal(cfg, active, changed, first_sample)
            except Exception as exc:  # noqa: BLE001 - ROS node should keep running after IO errors.
                self.get_logger().error(f'failed to read {cfg.name} gpio{cfg.gpio}: {exc}')
                self._publish_input(cfg, False, False, False, f'error: {exc}')

    def _maybe_publish_external_signal(
        self,
        cfg: InputConfig,
        active: bool,
        changed: bool,
        first_sample: bool,
    ) -> None:
        if not self.publish_external_signal:
            return
        if self.trigger_on_change and (not changed or first_sample):
            return
        if self.trigger_active_only and not active:
            return

        now = time.monotonic()
        if now - self._last_trigger_time[cfg.name] < self.min_trigger_interval_sec:
            return
        self._last_trigger_time[cfg.name] = now

        signal = ExternalSignal()
        signal.header.stamp = self.get_clock().now().to_msg()
        signal.header.frame_id = self.source
        signal.source = self.source
        signal.name = cfg.name
        signal.code = cfg.code
        signal.active = active
        signal.workflow_name = cfg.workflow_name
        signal.detail = cfg.detail
        self.signal_pub.publish(signal)
        self._maybe_trigger_workflow(signal)

    def _maybe_trigger_workflow(self, signal: ExternalSignal) -> None:
        if not self.auto_run_workflow or not signal.active or not signal.workflow_name:
            return
        if not self.workflow_client.service_is_ready():
            self.get_logger().warn(f'workflow service is not ready: {self.workflow_service}')
            return

        request = RunWorkflow.Request()
        request.workflow_name = signal.workflow_name
        request.dry_run = self.dry_run_workflow
        future = self.workflow_client.call_async(request)
        future.add_done_callback(self._workflow_done)

    def _workflow_done(self, future) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'workflow trigger accepted: {response.message}')
            else:
                self.get_logger().warn(f'workflow trigger rejected: {response.message}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'workflow trigger failed: {exc}')

    def _publish_input(
        self,
        cfg: InputConfig,
        raw: bool,
        active: bool,
        changed: bool,
        status: str,
    ) -> None:
        msg = DigitalInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source
        msg.source = self.source
        msg.name = cfg.name
        msg.gpio = cfg.gpio
        msg.value = raw
        msg.active = active
        msg.changed = changed
        msg.active_mode = cfg.active_mode
        msg.status = status
        self.input_pub.publish(msg)

    def _publish_output(self, cfg: OutputConfig, value: bool, status: str) -> None:
        msg = DigitalOutput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source
        msg.source = self.source
        msg.name = cfg.name
        msg.gpio = cfg.gpio
        msg.value = cfg.raw_from_value(value)
        msg.active = value
        msg.active_mode = cfg.active_mode
        msg.status = status
        self.output_pub.publish(msg)

    def _find_input(self, name: str) -> Optional[InputConfig]:
        for cfg in self.inputs:
            if cfg.name == name:
                return cfg
        return None

    def _find_output(self, name: str) -> Optional[OutputConfig]:
        for cfg in self.outputs:
            if cfg.name == name:
                return cfg
        return None

    def _handle_get_input(self, request, response):
        cfg = self._find_input(request.name)
        if cfg is None:
            response.success = False
            response.message = f"unknown input: {request.name}"
            return response

        try:
            raw = self.backend.read(cfg.gpio)
            response.success = True
            response.value = raw
            response.active = cfg.active_from_raw(raw)
            response.gpio = cfg.gpio
            response.message = 'ok'
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.gpio = cfg.gpio
            response.message = str(exc)
        return response

    def _handle_set_input(self, request, response):
        cfg = self._find_input(request.name)
        if cfg is None:
            response.success = False
            response.message = f"unknown input: {request.name}"
            return response
        if not self._using_mock_backend():
            response.success = False
            response.gpio = cfg.gpio
            response.message = 'set_input is only supported by the mock backend'
            return response

        raw = cfg.raw_from_active(request.active)
        previous_active = self._last_input_active[cfg.name]
        changed = previous_active is None or previous_active != request.active
        self.backend.write(cfg.gpio, raw)
        self._last_input_raw[cfg.name] = raw
        self._last_input_active[cfg.name] = request.active
        self._last_raw_change_time[cfg.name] = time.monotonic()

        self._publish_input(cfg, raw, request.active, changed, 'mock_set')
        if self.enabled:
            self._maybe_publish_external_signal(
                cfg,
                request.active,
                changed,
                first_sample=False,
            )

        response.success = True
        response.value = raw
        response.active = request.active
        response.gpio = cfg.gpio
        response.message = 'ok'
        return response

    def _handle_set_output(self, request, response):
        cfg = self._find_output(request.name)
        if cfg is None:
            response.success = False
            response.message = f"unknown output: {request.name}"
            return response
        if not self._output_writes_enabled():
            response.success = False
            response.gpio = cfg.gpio
            response.message = 'output writes are disabled; set allow_output_writes:=true after wiring is verified'
            return response

        try:
            raw = cfg.raw_from_value(request.active)
            self.backend.write(cfg.gpio, raw)
            self._output_values[cfg.name] = request.active
            self._publish_output(cfg, request.active, 'ok')
            response.success = True
            response.value = raw
            response.active = request.active
            response.gpio = cfg.gpio
            response.message = 'ok'
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.gpio = cfg.gpio
            response.message = str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpioIoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
