# Workcell Scripts

## One-click start

```bash
cd ~/panthera_workcell_ws
scripts/start_workcell.sh
```

This starts hardware/MoveIt/workflow/RS485 first, then starts the spectrometer cell, HMI, pose tuner, and Gemini305 camera. Logs are written under `validation_logs/<timestamp>_start_workcell`.

HMI default URL:

```text
http://<aibox-ip>:8080
```

## One-click stop

```bash
cd ~/panthera_workcell_ws
scripts/stop_workcell.sh
```

The stop script first calls `/run_workflow` with `arm_home`. If that fails, it tries the pose tuner `safe_joint_center` target. After the home/safe motion attempt it terminates the ROS processes.

Emergency maintenance shutdown without motion:

```bash
scripts/stop_workcell.sh --no-home
```

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
