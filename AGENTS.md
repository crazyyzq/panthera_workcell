# Panthera Workcell Engineering Notes

This file contains durable repository context and operating rules for future agents and maintainers. Do not put passwords, private keys, tokens, or other credentials here.

## Workspace and platform

- Canonical remote workspace: `/home/b1/panthera_workcell_ws`.
- The Windows network share `\\10.89.8.169\b1s_share\panthera_workcell_ws` maps to that same directory.
- Target platform observed on 2026-07-17: Ubuntu 22.04 on Rockchip kernel `5.10.0-1012-rockchip`.
- ROS distribution: ROS 2 Humble.
- MoveIt version observed: 2.5.9.
- ros2_control/ros2_controllers observed: 2.53.1/2.52.1.
- Build from the Linux host. Windows access is useful for inspection/editing, but ROS build and runtime validation must run on the target host.

## Repository map

- `src/hightorque_robot`: vendor/low-level Panthera SDK and bundled dependencies. Treat `third_part/` as vendored upstream code; avoid broad formatting or unrelated edits.
- `src/panthera_hardware`: ros2_control hardware interface for the Panthera arm and gripper.
- `src/panthera_ht_config`: MoveIt, ros2_control, controller and launch configuration.
- `src/panthera_ht_ros_description`: URDF/xacro and meshes.
- `src/panthera_interfaces`: shared ROS messages and services.
- `src/panthera_task_framework`: legacy YAML workflow executor; currently plans each motion with MoveIt.
- `src/panthera_spectrometer_cell`: production-oriented spectrometer state machine and its current duplicated MoveIt `RobotActions` implementation.
- `src/panthera_pose_tuner`: legacy point tuner with hard-coded point definitions.
- `src/panthera_web_hmi`: ROS/Web bridge and static HMI.
- `src/panthera_rs485`: serial/Modbus, laser and RS485 state adapters.
- `src/panthera_io`: optional GPIO/DIDO adapter; real GPIO is disabled by default because the installed interface board/pinmux is not confirmed.
- `src/panthera_motion`: deterministic catalog compiler, cache and single-owner `FollowJointTrajectory` executor. Use this for new production motion.
- `docs/FIXED_TRAJECTORY_REFACTOR_PLAN.md`: authoritative implementation plan for the fixed-trajectory refactor.

## Current motion facts

- The arm controller is `joint_trajectory_controller/JointTrajectoryController` at 100 Hz.
- Arm joints are `joint1` through `joint6`; gripper command joint is `L_finger_joint`; `R_finger_joint` is mimic/passive.
- The controller exposes standard `FollowJointTrajectory` and can execute deterministic precompiled trajectories without MoveIt planning at production runtime.
- Legacy `panthera_task_framework` and `panthera_spectrometer_cell::RobotActions` still replan with MoveIt and are migration-only. Their default HMI launch flags are disabled so they do not run beside another legacy owner.
- Target architecture: one motion server is the only arm trajectory owner. MoveIt is retained for commissioning, IK, collision checks and trajectory compilation, not per-step production planning.
- Fixed positioning is the current default requirement. Laser-based position correction must remain available behind an explicit `fixed | sensor_offset` mode.
- Near-object pick/place/lift/retreat segments must be explicit Cartesian lines with a vertical constraint and validated lateral error.

## Safety invariants

- Never execute real robot motion merely to test a code change. Use build, unit, simulation and dry-run validation first.
- Before real motion, confirm the workspace is clear, hardware E-stop is available, the controller is healthy, the robot is stationary, and the current joints are within the trajectory start tolerance.
- Never silently bridge an arbitrary current state to a cached trajectory. Reject start mismatch or use a separately commissioned recovery route.
- Only one node may command `/arm_controller/follow_joint_trajectory`.
- A software stop/HMI button is not a replacement for a wired hardware E-stop.
- Config reload is allowed only while idle/paused and with no active motion goal. New config becomes active only after complete validation/compilation succeeds.
- Deleting a point referenced by any route must be rejected.
- Preserve the state-machine business invariants: the cup returns to its source outlet; before-pick position is distinct from before-place position; ERROR/ESTOP retain recovery context; ESTOP has highest priority.
- Do not enable real GPIO/DIDO output until the installed interface board, pinmux, permissions and electrical wiring are verified.

## Known high-risk issues in the pre-refactor baseline

- MoveIt and the new motion server use a `0.05 rad` start tolerance. Do not loosen it without measured encoder/repeatability evidence.
- The spectrometer state machine calls robot actions synchronously, so its configured action timeout cannot interrupt a blocked action.
- Real E-stop, outlet signals and spectrometer-complete integration are not fully wired; some paths still use manual/simulation services.
- The legacy `clean_brush` roll has been normalized from the invalid-looking `30.0 rad`
  to `0.523599 rad` (30 degrees). This is an engineering assumption, not completed
  physical validation; confirm brush direction at low speed before insertion.
- Spectrometer sensor correction axis is inconsistent across historical docs/config (X versus Y). Confirm physical direction before enabling sensor mode.
- HMI exposes full point/route CRUD for `motion_catalog.yaml`. Saves use schema checks, Motion Server compilation, atomic replacement and automatic rollback. The lower legacy state-machine parameter editor remains only during migration.
- The optimized catalog has 15 points and 18 routes. Only 6 points carry the `tunable`
  tag and appear in the normal HMI view; `advanced` hover/recovery points are hidden by
  default. Do not reintroduce per-action approach/pre/near/retreat points unless measured
  collision evidence requires them.
- Normal station motion uses one hover directly above each process point and a strict
  vertical final segment. Cross-station transfers should be continuous cached routes,
  not sequences that stop at every intermediate sample.
- Historical `.bak_*`, `bakeup/`, zip and export artifacts exist. Do not confuse them with canonical configuration.

## Build and validation

Run on the Ubuntu target:

```bash
cd /home/b1/panthera_workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

For focused development, use `--packages-up-to <package>` and still run a full build before handoff.

As of the initial fixed-motion refactor, a full workspace build succeeds, while the
legacy full test suite still reports 34 pre-existing formatting/lint failures (mostly
`ament_uncrustify`). Do not misclassify those as Motion Server functional failures;
keep focused package tests green and reduce the legacy lint debt in dedicated commits.

The commissioning-only fixed motion launch is:

```bash
ros2 launch panthera_motion fixed_motion_bringup.launch.py default_speed_scale:=0.20
```

It starts hardware/controllers plus the Motion Server without MoveGroup. Do not start legacy real-motion nodes beside it.

Runtime outputs belong in `build/`, `install/`, `log/`, `validation_logs/` and `.runtime/`; do not commit them.

## Configuration conventions

- Position units: metres.
- Joint and RPY units: radians.
- Laser distances: millimetres at adapter boundaries; convert explicitly to metres inside calibrated transforms.
- Quaternion order when present: XYZW.
- Base frame: `base_link`.
- Tool/TCP frame: `gripper_center`.
- Point and route IDs use stable `snake_case` names.
- Production motion data must have one canonical source. Do not add another hard-coded list in C++, launch files or the HMI.
- Machine-managed config saves must be validated and atomic (temporary file, flush/fsync, rename). A failed save/compile must leave the previous active version intact.

## Coding rules for the refactor

- Keep motion compilation, controller execution, business orchestration, sensors and HMI in separate modules.
- Prefer small libraries with unit-testable pure validation/compilation logic; keep ROS nodes as adapters.
- Use ROS actions for cancellable long-running motion, not blocking services.
- Return structured error codes/messages identifying route, segment, sample and failed constraint.
- Enforce finite numeric values, dimensions, joint limits, trajectory continuity, fresh state, collision checks and start tolerance.
- Speed scaling must retime trajectories and recheck velocity/acceleration limits; do not only change a MoveIt planning parameter.
- Avoid detached threads and multiple command publishers. Own worker lifetime and cancellation explicitly.
- Do not make unrelated changes inside the vendor SDK or generated mesh assets.
- Add tests with every parser, validator, compiler, execution-state or safety change.

## Git workflow

- Preserve the initial pre-refactor baseline commit.
- Make phase-sized commits that remain buildable.
- Never commit credentials, runtime logs, PID files, caches or HMI timestamp backups.
- Before a real-hardware validation session, record the exact commit/config version and keep a known-good rollback point.
- Do not run destructive Git commands against user changes.

## Open confirmations required before hardware acceptance

- Low-speed physical confirmation of the assumed 30-degree brush roll and insertion axis.
- Final fixed spectrometer TCP and future sensor correction axis.
- Physical E-stop, outlet-ready and spectrometer-done wiring/interfaces.
- Approved production speed/acceleration and cycle-time target.
- Whether low-speed acceptance may use an empty cup.
