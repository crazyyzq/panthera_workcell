#!/usr/bin/env python3
"""Run repeatable workcell cycles with either bypassed or real gripper commands."""

import argparse
import json
import os
from pathlib import Path
import signal
import tempfile
import time
from urllib import error, request


BASE_URL = "http://127.0.0.1:8080"
WORKSPACE = Path(os.environ.get("WS", "/home/b1/panthera_workcell_ws"))
CONFIG_PATH = WORKSPACE / "src/panthera_spectrometer_cell/config/spectrometer_cell.yaml"
ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))


def api(path, body=None, timeout=15.0):
    data = None if body is None else json.dumps(body).encode()
    call = request.Request(
        BASE_URL + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if body is None else "POST",
    )
    try:
        with request.urlopen(call, timeout=timeout) as response:
            return json.load(response)
    except error.HTTPError as exc:
        try:
            detail = json.load(exc)
        except Exception:
            detail = {"message": str(exc)}
        raise RuntimeError(f"{path}: {detail.get('message', exc)}") from exc


def atomic_write(path, content):
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def set_gripper_bypass(original, enabled):
    source = "  command_enabled: true"
    target = "  command_enabled: false"
    expected = source if enabled else target
    replacement = target if enabled else source
    if original.count(expected) != 1:
        raise RuntimeError(f"expected exactly one '{expected.strip()}' in {CONFIG_PATH}")
    atomic_write(CONFIG_PATH, original.replace(expected, replacement, 1))
    result = api("/api/point_config/reload", {})
    if not result.get("success"):
        raise RuntimeError(f"config reload failed: {result.get('message', result)}")


def joint_map(status, field):
    state = status["joint_state"]
    return dict(zip(state["names"], state[field]))


def detection_signal_should_retry(result):
    return (not result.get("success") and
            "not waiting for detection completion" in result.get("message", ""))


def require_test_start(status):
    cell = status["spectrometer_cell"]
    context = cell["context"]
    positions = joint_map(status, "positions")
    if cell["state"] != "WAIT_DISCHARGE" or context["has_active_task"]:
        raise RuntimeError("workcell must be idle in WAIT_DISCHARGE")
    if context["cup_in_gripper"] or context["spectrometer_occupied"]:
        raise RuntimeError("workcell context is not empty")
    home_error = max(abs(positions[name]) for name in ARM_JOINTS)
    if home_error > 0.05:
        raise RuntimeError(f"arm is not at Home: max_error={home_error:.6f}rad")
    if positions["L_finger_joint"] < 0.045:
        raise RuntimeError(
            f"gripper is not open: position={positions['L_finger_joint']:.6f}m")
    return positions["L_finger_joint"]


def pause_before_restore():
    try:
        status = api("/api/status", timeout=5.0)
        if status["spectrometer_cell"]["state"] == "WAIT_DISCHARGE":
            return
        api("/api/command", {"command": "manual_mode"}, timeout=5.0)
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            status = api("/api/status", timeout=5.0)
            cell = status["spectrometer_cell"]
            if cell["state"] in ("PAUSED", "ERROR", "ESTOP"):
                return
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] pause unavailable: {exc}", flush=True)


def run_round(round_number, open_position, telemetry, real_gripper=False):
    start = time.monotonic()
    response = api("/api/command", {"command": "outlet_1_done"})
    if not response.get("success"):
        raise RuntimeError(f"outlet signal rejected: {response}")

    started = False
    detection_sent = False
    next_detection_attempt = 0.0
    last_state = ""
    max_velocity = 0.0
    gripper_min = open_position
    gripper_max = open_position
    held_gripper_min = None
    held_gripper_max = None
    visited_states = set()
    deadline = start + 240.0
    while time.monotonic() < deadline:
        status = api("/api/status")
        cell = status["spectrometer_cell"]
        context = cell["context"]
        state = cell["state"]
        positions = joint_map(status, "positions")
        velocities = joint_map(status, "velocities")
        efforts = joint_map(status, "efforts")
        gripper = positions["L_finger_joint"]
        gripper_min = min(gripper_min, gripper)
        gripper_max = max(gripper_max, gripper)
        max_velocity = max(max_velocity, *(abs(velocities[name]) for name in ARM_JOINTS))

        if not real_gripper and abs(gripper - open_position) > 0.003:
            raise RuntimeError(
                f"gripper moved during bypass: start={open_position:.6f} current={gripper:.6f}m")
        if real_gripper and context["cup_in_gripper"]:
            held_gripper_min = gripper if held_gripper_min is None else min(
                held_gripper_min, gripper)
            held_gripper_max = gripper if held_gripper_max is None else max(
                held_gripper_max, gripper)
            if not 0.025 <= gripper <= 0.045:
                raise RuntimeError(
                    f"possible cup loss while held: gripper={gripper:.6f}m")
        if state in ("ERROR", "ESTOP") or context.get("error"):
            raise RuntimeError(f"workcell fault state={state} error={context.get('error', '')}")

        telemetry.write(json.dumps({
            "time": time.time(),
            "round": round_number,
            "state": state,
            "positions": {name: positions[name] for name in ARM_JOINTS},
            "velocities": {name: velocities[name] for name in ARM_JOINTS},
            "efforts": {name: efforts[name] for name in ARM_JOINTS},
            "gripper_position_m": gripper,
        }, separators=(",", ":")) + "\n")

        if state != last_state:
            print(f"[round {round_number:03d}] {state}", flush=True)
            last_state = state
        if state != "WAIT_DISCHARGE" or context["has_active_task"]:
            started = True
        if started:
            visited_states.add(state)
        now = time.monotonic()
        if (state == "WAIT_DETECTION_DONE" and not detection_sent and
                now >= next_detection_attempt):
            result = api("/api/command", {"command": "detection_done"})
            if result.get("success"):
                detection_sent = True
                print(f"[round {round_number:03d}] detection_done sent", flush=True)
            elif detection_signal_should_retry(result):
                next_detection_attempt = now + 0.5
                print(f"[round {round_number:03d}] detection_done deferred", flush=True)
            else:
                raise RuntimeError(f"detection signal rejected: {result}")
        if started and state == "WAIT_DISCHARGE" and not context["has_active_task"]:
            required_states = {
                "PICK_FROM_OUTLET", "PLACE_TO_SPECTROMETER",
                "PICK_FROM_SPECTROMETER", "CLEAN_CUP", "RETURN_CUP",
            }
            missing_states = sorted(required_states - visited_states)
            if missing_states:
                raise RuntimeError(
                    "incomplete production cycle; missing states="
                    f"{','.join(missing_states)} reason="
                    f"{context.get('last_transition_reason', '')}")
            if real_gripper and held_gripper_min is None:
                raise RuntimeError("complete cycle reported without a held-cup sample")
            telemetry.flush()
            return {
                "round": round_number,
                "duration_sec": time.monotonic() - start,
                "max_abs_velocity_rad_sec": max_velocity,
                "gripper_min_m": gripper_min,
                "gripper_max_m": gripper_max,
                "held_gripper_min_m": held_gripper_min,
                "held_gripper_max_m": held_gripper_max,
                "detection_signal_sent": detection_sent,
                "home_error_rad": max(abs(positions[name]) for name in ARM_JOINTS),
            }
        time.sleep(0.2)
    raise RuntimeError(f"round {round_number} exceeded 240 seconds")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=100)
    parser.add_argument("--real-gripper", action="store_true")
    args = parser.parse_args()
    if args.rounds < 1:
        raise SystemExit("--rounds must be positive")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    mode = "real_gripper" if args.real_gripper else "open_gripper"
    log_dir = WORKSPACE / "validation_logs" / f"{stamp}_{args.rounds}_cycle_{mode}"
    log_dir.mkdir(parents=True)
    original = CONFIG_PATH.read_text(encoding="utf-8")
    (log_dir / "spectrometer_cell.yaml.before").write_text(original, encoding="utf-8")
    summaries = []
    bypass_active = False

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        open_position = require_test_start(api("/api/status"))
        if not args.real_gripper:
            set_gripper_bypass(original, True)
            bypass_active = True
        with (log_dir / "motor_telemetry.jsonl").open("w", encoding="utf-8") as telemetry:
            for round_number in range(1, args.rounds + 1):
                summary = run_round(
                    round_number, open_position, telemetry,
                    real_gripper=args.real_gripper)
                summaries.append(summary)
                print(
                    f"[round {round_number:03d}] PASS {summary['duration_sec']:.2f}s "
                    f"home={summary['home_error_rad']:.6f}rad "
                    f"vmax={summary['max_abs_velocity_rad_sec']:.3f}rad/s",
                    flush=True,
                )
        result = {"success": True, "completed": len(summaries), "rounds": summaries}
        (log_dir / "summary.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8")
        print(f"[done] {args.rounds}/{args.rounds} log={log_dir}", flush=True)
    except BaseException as exc:
        result = {
            "success": False,
            "completed": len(summaries),
            "error": str(exc),
            "rounds": summaries,
        }
        (log_dir / "summary.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8")
        print(f"[failed] completed={len(summaries)} error={exc} log={log_dir}", flush=True)
        raise
    finally:
        if bypass_active:
            pause_before_restore()
            atomic_write(CONFIG_PATH, original)
            try:
                reload_result = api("/api/point_config/reload", {}, timeout=10.0)
                print(f"[cleanup] gripper command restored: {reload_result}", flush=True)
            except Exception as exc:
                print(
                    f"[cleanup] config restored on disk; runtime reload failed: {exc}",
                    flush=True)


if __name__ == "__main__":
    main()
