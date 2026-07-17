# panthera_motion

`panthera_motion` is the deterministic motion layer for the Panthera workcell.
It compiles named joint/Cartesian routes once, validates every sampled state, and
executes the cached `JointTrajectory` directly through the arm controller action.
Production execution does not call a MoveIt planner for every step.

## Safety model

- The server is intended to be the only owner of
  `/arm_controller/follow_joint_trajectory`.
- A route is rejected unless fresh `/joint_states` match the compiled first point.
- The arm must be settled before a route starts.
- A catalog reload is all-or-nothing; a failed compile retains the previous cache.
- Runtime speed scaling can only slow a compiled trajectory.
- Long-running motion uses a cancellable ROS action.

## Interfaces

```text
/motion/execute          panthera_interfaces/action/ExecuteMotion
/motion/reload           std_srvs/srv/Trigger
/motion/list_routes      std_srvs/srv/Trigger
/motion/stop             std_srvs/srv/Trigger
/motion/set_speed_scale  panthera_interfaces/srv/SetSpeedScale
```

Dry-run a compiled route without requiring a controller or joint state:

```bash
ros2 action send_goal /motion/execute \
  panthera_interfaces/action/ExecuteMotion \
  "{route_name: home_to_safe_center, speed_scale: 0.2, dry_run: true}" \
  --feedback
```

Bring up the real controllers with the motion server as the only arm-trajectory
owner:

```bash
ros2 launch panthera_motion fixed_motion_bringup.launch.py \
  default_speed_scale:=0.20
```

Do not start the legacy workflow executor, pose tuner, or spectrometer cell in
real-motion mode alongside this launch file.

## Catalog

The canonical catalog is `config/motion_catalog.yaml`. A point contains explicit
joint values, a TCP pose, or both. Pose-only points are solved deterministically
from `ik_seed`. A route has one fixed start point and ordered `joint` or `linear`
segments. `linear` segments may require a strict X/Y/Z vertical constraint.

The catalog contains 6 operator-facing process points, shared safety hovers, and
18 deterministic routes. Normal tuning should change only points tagged
`tunable`; points tagged `advanced` are recovery or clearance points. Every
route compiles against the robot model, but real execution still requires the
staged low-speed commissioning procedure.
