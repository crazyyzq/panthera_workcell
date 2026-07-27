# Workcell Scripts

## One-click start

```bash
cd /home/b1/panthera_workcell_ws
scripts/start_workcell.sh
```

This starts the commissioned production chain only: MIT hardware, ros2_control,
Motion Server, the fixed-cache spectrometer state machine, and the Web HMI. It
does not start MoveGroup, the legacy workflow executor, laser, pose tuner, or
camera. Startup is idempotent and requires active controllers, a settled Motion
Server, fresh encoder positions at the commissioned Home, and ready HMI services.
Failed startup is retried only after a guarded cleanup; an unknown/non-Home pose
is left powered and holding rather than being disabled.

HMI default URL:

```text
http://192.168.137.186:8080
```

## One-click stop

```bash
cd /home/b1/panthera_workcell_ws
scripts/stop_workcell.sh
```

Normal stop waits for an empty/finished cycle, pauses automatic operation, verifies
Motion Server is idle and settled, verifies the original Home from fresh encoders,
sends an explicit brush/motion stop, terminates the owned launch process group,
and removes any related legacy or production process left behind. It refuses to
disable the arm from an unknown pose or while a cup is still in process.

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
