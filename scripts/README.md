# Workcell Scripts

## Always-on HMI

The Web HMI starts automatically with Ubuntu and remains available when the
production workcell is stopped:

```text
http://192.168.137.186:8080
```

Use the **启动工作站**, **安全停止**, and **安全重启** buttons in the HMI.
Operations are serialized, show live progress, and reuse the lifecycle scripts
below. Safe stop returns the arm to Home and clears production processes without
stopping the HMI. The boot service source is
`config/system/panthera-hmi.service`.

The HMI debug page separates **进入调试（不移动）** from point motion. Once
debug mode is ready, brush cleaning time (default 6 s) and spectrometer scan
completion time (default 40 s) can be atomically saved and hot-reloaded without
moving the arm.

## Command-line fallback start

```bash
cd /home/b1/panthera_workcell_ws
scripts/start_workcell.sh
```

This starts the commissioned production chain only: MIT hardware, ros2_control,
Motion Server, the fixed-cache spectrometer state machine, and laser adapter.
The HMI is a separate always-on service. Production does not start MoveGroup,
the legacy workflow executor, pose tuner,
or camera. Startup is idempotent and requires active controllers, a settled Motion
Server, fresh encoder positions at the commissioned Home, and ready HMI services.
Laser data is optional: fresh valid data applies the calibrated spectrometer X
offset, while a sensor absent from startup uses the configured fixed reference.
The laser zero is an HMI calibration value, not a hard-coded distance.
Failed startup is retried only after a guarded cleanup; an unknown/non-Home pose
is left powered and holding rather than being disabled.

Startup also removes a stale ROS CLI daemon before controller health queries. If
that daemon ignores TERM or is already in `!rclpy.ok()`, the script kills only the
CLI daemon and stale CLI clients, then retries discovery; hardware/control nodes
remain powered and are never included in this reset.

## Command-line fallback stop

```bash
cd /home/b1/panthera_workcell_ws
scripts/stop_workcell.sh
```

Normal stop waits for an empty/finished cycle, pauses automatic operation, verifies
Motion Server is idle and settled, verifies the original Home from fresh encoders,
sends an explicit brush/motion stop, terminates the owned launch process group,
and removes any related legacy or production process left behind. It refuses to
disable the arm from an unknown pose or while a cup is still in process.
The always-on HMI is deliberately excluded from cleanup and remains accessible.

Emergency maintenance shutdown without motion:

```bash
scripts/stop_workcell.sh --force
```

`--force` (and the legacy alias `--no-home`) skips the cycle/Home guard. Use it
only after manually making the robot mechanically safe; it can disable the arm
at its current pose.

## Restart Camera Only

```bash
cd ~/panthera_workcell_ws
scripts/restart_camera.sh
```

The HMI "restart camera" button calls this script. It restarts Gemini305 at `1280x800@30` and records logs under `validation_logs/<timestamp>_restart_camera`.

## Source and Documentation Backup

```bash
cd ~/panthera_workcell_ws
scripts/backup_source_docs.sh
```

The backup script writes a clean archive to:

```text
~/panthera_workcell_ws_backups
```

It includes `src/`, `docs/`, `scripts/`, and `README.md`.

It excludes build and runtime artifacts such as `build/`, `install/`, `log/`, `Log/`, `validation_logs/`, `.runtime/`, `.last_*`, and `.git/`.
