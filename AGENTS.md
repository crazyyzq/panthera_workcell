# Panthera Workcell Engineering Notes

This file contains durable repository context and operating rules for future agents and maintainers. Do not put passwords, private keys, tokens, or other credentials here.

## Workspace and platform

- Canonical remote workspace: `/home/b1/panthera_workcell_ws`.
- Current controller address: `192.168.137.186`. The Windows share is
  `\\192.168.137.186\b1s_share\panthera_workcell_ws` and maps to the canonical
  workspace. Treat older `10.89.*`, `172.20.*`, and `192.168.8.*` addresses as stale.
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
- `src/panthera_pose_tuner`: legacy point tuner with hard-coded point definitions; it is no longer launched or exposed by the production HMI.
- `src/panthera_web_hmi`: ROS/Web bridge and static HMI.
- `src/panthera_rs485`: serial/Modbus, laser and RS485 state adapters.
- `src/panthera_io`: optional GPIO/DIDO adapter; real GPIO is disabled by default because the installed interface board/pinmux is not confirmed.
- `src/panthera_motion`: deterministic catalog compiler, cache and single-owner `FollowJointTrajectory` executor. Use this for new production motion.
- `docs/FIXED_TRAJECTORY_REFACTOR_PLAN.md`: authoritative implementation plan for the fixed-trajectory refactor.

## Current motion facts

- Current physically validated smooth profile (2026-07-18): arm trajectory velocity
  limit `2.2 rad/s`, acceleration limit `3.0 rad/s^2`, and catalog jerk limit
  `300 rad/s^3`. Nearby process transitions use lower per-route scaling; the final
  10 mm pick/place segment remains at 40%. Do not restore the previous 4 rad/s^2
  acceleration profile, which produced visible overshoot.
- The commissioned spectrometer pickup column is `x=0.600 m`, `y=-0.132 m`;
  `spectrometer_pick`, `spectrometer_prepick`, and `spectrometer_pick_hover` must
  keep the same x/y so pickup and lift remain vertical. Pickup z is `0.312 m`,
  10 mm below the commissioned placement point.
- The arm controller is `joint_trajectory_controller/JointTrajectoryController` at 100 Hz.
- Arm joints are `joint1` through `joint6`; gripper command joint is `L_finger_joint`; `R_finger_joint` is mimic/passive.
- The controller exposes standard `FollowJointTrajectory` and can execute deterministic precompiled trajectories without MoveIt planning at production runtime.
- Vendor MIT MoveIt support was integrated from upstream `humble` commit `3815fce`
  on 2026-07-24. The hardware mode is `mit_gravity_compensation`: trajectory
  position/velocity targets plus Pinocchio gravity feed-forward are sent through
  the SDK position/velocity/torque/Kp/Kd command.
- Commissioned MIT arm gains are `Kp=[75,105,135,135,75,75]` and
  `Kd=[5.5,5.5,5.5,5.5,5.5,5.5]`. They are launch-configurable but malformed, non-finite, or
  negative vectors must fail closed. The gripper stays on retained position control.
  These per-joint gains were physically commissioned on 2026-07-28 by increasing
  one joint group at a time with Kd fixed. High-speed Home/process-point
  round trips showed no stopping oscillation, so Kd was not increased. The
  brush-roll joint3 steady error fell from about `0.0150 rad` at Kp 60 to
  `0.0087 rad` at Kp 120; clean-dump joint2 error fell from about
  `0.0073 rad` at Kp 60 to `0.0048 rad` at Kp 90; Home joint4 error fell
  from about `0.0148 rad` at Kp 60 to about `0.0085 rad` at Kp 120.
  A synchronized desired/actual refinement later that day increased only J2-J4
  to `105/135/135`: J2/J3 full-speed dynamic RMS error improved by about
  5.5%/4.4%, endpoint residuals improved, and stopping remained motionless.
  Raising J2-J4 Kd to 6.0 increased J3/J4 velocity-step p95 by about 8%/9%,
  so Kd remains 5.5. Reducing J2 gravity scale from 1.04 to 1.02 also worsened
  dynamic tracking and was rejected; keep the four-pose gravity calibration.
- The vendor CAN bridge has one packet mode per CAN port. The six arm motors use
  MIT, while motor 7 (the gripper) uses its proven pos/vel/max-torque mode. In every
  hardware write cycle send the gripper packet first and the six-axis MIT packet
  last. Sending arm MIT first and gripper pos/vel last clears the six MIT fields to
  `0x8000`, starves joints of refreshed holding commands, and caused the repeatable
  random joint death/J4 collapse. Forcing motor 7 into MIT did not move the gripper.
  Do not reverse this ordering or mix another packet mode after the arm command.
- `on_deactivate()` brakes the motors, so `on_activate()` must reset the motor boards
  before latching encoder state and accepting controller commands. A controller can
  otherwise report `active` while the physical joints remain braked.
- The 2026-07-24 physical acceptance found that the upstream all-axis gravity
  scale `[1,1,1,1,1,1]` causes unsafe joint drift on this arm. A four-pose
  closed-loop calibration on 2026-07-28 replaced the initial
  `[0,1,1.5,1,0,0]` calibration with `[0,1.04,1.10,1.52,0,0]`. At the two
  outlet grasp poses it reduced joint3 steady error from about
  `-0.026/-0.031 rad` to `-0.0053/-0.0036 rad`, and reduced joint4 error from
  about `+0.0075/+0.0078 rad` to `+0.0006/+0.0015 rad`. It also held Home and
  completed Home-to-point-to-Home routes without a controller or hardware fault.
  Keep the scale launch-configurable and do not restore the upstream default
  without a new staged physical acceptance.
- The end effector has been mechanically modified and is heavier than the
  factory assembly; its added mass and center of mass are not yet measured.
  Runtime gravity compensation loads the generated
  `panthera_ht_ros_description.urdf`, whose modeled link6, two fingers and
  gripper-center mass total about 0.361 kg. On 2026-07-28, five safe static
  poses proved that single-frame motor torque cannot reliably identify the
  unknown payload (wrist torque estimates jump with transmission/friction).
  The trustworthy signal is the settled controller desired-minus-actual joint
  error: with Kp=60 it reproduced the observed equilibrium motor torque.
  Four-pose joint2-4 fitting on 2026-07-28 could not be explained by one physical
  payload: bounded fits drove the COM outside the tool, while unconstrained fits
  produced negative mass. The repeatable brush-roll joint3 residual is therefore
  also affected by robot-model error and static transmission friction. Keep added
  payload disabled until a known reference mass or isolated end-effector
  measurement makes the parameters observable. Never accept a fitted negative
  mass or an implausible center of mass.
- MIT gravity compensation supports an explicit added payload via
  `PANTHERA_PAYLOAD_MASS_KG`, `PANTHERA_PAYLOAD_COM_XYZ_M` and
  `PANTHERA_PAYLOAD_FRAME` (launch/start equivalents:
  `PAYLOAD_MASS_KG`, `PAYLOAD_COM_XYZ_M`, `PAYLOAD_FRAME`). The COM is in the
  named frame, normally `gripper_center`. Defaults are zero added payload and
  therefore preserve the commissioned machine behavior. Measure the added
  hardware mass and COM before enabling it; do not guess payload values from
  the existing per-joint gravity multipliers.
- Hardware configuration and activation must each latch the median of five valid
  encoder frames collected over up to ten SDK reads, never one frame. The SDK
  reports `999` while a serial motor is still coming online; treat that as a
  retryable invalid sample, not a joint position and not an immediate process
  crash. A single-frame latch produced a repeatable joint1 startup drift.
- The SDK can continuously report quantized nonzero velocity while the encoder
  position is stationary (joint5 repeatedly reported about `0.062 rad/s`). Motion
  Server therefore estimates settled motion from an 11-frame least-squares encoder
  position slope while retaining the `0.05 rad/s` limit; it does not trust one SDK
  velocity sample or loosen the safety threshold.
- The ros2_control hardware velocity export uses a moving average after its existing
  position-jump rejection. Do not restore the old median: at 100 Hz the quantized
  encoder produces many zero deltas and occasional real steps, so the median falsely
  reported long zero-velocity intervals during continuous physical motion.
- `fixed_motion_bringup.launch.py` and `fixed_spectrometer_cell.launch.py` default
  to MIT mode. Explicit `control_mode:=position_velocity` remains the immediate
  rollback path. `hardware_moveit_rviz_mit.launch.py` is the MoveIt commissioning
  entry point; production fixed-cache motion still does not start MoveGroup.
- Legacy `panthera_task_framework` and `panthera_spectrometer_cell::RobotActions` still replan with MoveIt and are migration-only. Their default HMI launch flags are disabled so they do not run beside another legacy owner.
- Target architecture: one motion server is the only arm trajectory owner. MoveIt is retained for commissioning, IK, collision checks and trajectory compilation, not per-step production planning.
- Fixed positioning is the current default requirement. Laser-based position correction must remain available behind an explicit `fixed | sensor_offset` mode.
- The 2026-07-24 bench revision rotated the robot base clockwise 90 degrees. In the
  current `base_link`, +X points toward the outlets and +Y toward the spectrometer.
  Provisional measured process coordinates are outlet1
  `(0.46853,-0.095,0.190)m`, outlet2 `(0.46853,0.085,0.190)m`, and dump
  `(0.09126,-0.34374,0.250)m`. Dump z=0.250 m is an intentionally raised
  commissioning value, not final calibration. The transformed spectrometer pickup
  column is `(0.132,0.600)m`.
- The revised cleaning branch keeps the gripper pointing toward base `-Y`
  (yaw `-1.5708 rad`) and uses pour roll `-2.5 rad`. After pouring at
  `(0.09126,-0.34374,0.250)m`, move in a straight line along base `+X` by 150 mm
  to `(0.24126,-0.34374,0.250)m`, then insert 100 mm along the flipped cup axis
  to `(0.301107,-0.34374,0.169886)m`. Physical validation on 2026-07-24 completed
  a full production cycle with this 150 mm offset and 100 mm insertion, including
  gripper and brush operation, and returned Home successfully.
- Cleaning exit must exactly reverse those primitives: retract 100 mm along the
  cup axis, translate 150 mm along base `-X` back to the dump point, rotate upright
  in place, then lift vertically. Never combine translation, wrist return and lift
  in one joint-space segment; that produced a visible wrist/TCP dip.
- The 2026-07-24 full-cycle timing at speed scale 1.0 was 50.33 s. The optimized
  `brush_entry_to_outlet_1_return_continuous` route takes 9.57 s versus 17.61 s
  for the old monolithic route. Controller-state capture found joint6 acceleration
  spikes around 90-92 rad/s^2 on the two routes containing the large pour wrist
  transition (`spectrometer_pick_to_brush_entry_continuous` and
  `brush_entry_to_outlet_1_return_continuous`); other main segments were about
  4.7-6.5 rad/s^2. Treat joint6 continuous retiming/blending as the next motion
  optimization; do not tune MIT Kp/Kd to mask this trajectory-level discontinuity.
- Joints 1-5 use route path tolerance `0.15 rad` (or `0.20 rad` during
  pouring). Joint6 alone uses `0.45 rad` path tolerance because a measured
  `0.3536 rad` transient wrist lag caused a false abort at the former `0.35 rad`
  boundary; final position/velocity tolerances remain strict.
- Near-object pick/place/lift/retreat segments must be explicit Cartesian lines with a vertical constraint and validated lateral error.
- The cleaning brush motor driver is AQMD6030NS-A3 on `/dev/ttyS8`, Modbus RTU
  slave `0x02`, default `9600/8E1`. Its SW8 must be ON. The manual uses an
  offset decode: SW1-SW7 all OFF is `0x01`, while the installed SW1 ON setting
  is `0x02`; a full live scan confirmed only `0x02` replies. Production control
  uses duty-cycle mode (`0x0080=0`) and
  signed speed register `0x0040` (`-1000..1000` = `-100.0%..100.0%`).

## Safety invariants

- The commissioned original Home joint vector is
  `[-0.006, 0.0, 0.012, -0.072, -0.006, 0.034]`. Do not overwrite it from a
  post-power-loss/random pose. On a recoverable runtime error, return to this Home
  while enabled before disabling or restarting hardware.
- Production gripper close is a retained position target of `0.0 m`; it must remain
  commanded until an explicit release/open operation. Do not auto-release because
  contact prevents the encoder from reaching zero.
- Gripper path tolerance must not include a nonzero velocity tolerance. The SDK reports
  quantized finger velocity while opening/closing, and applying the final settled
  velocity threshold to the whole path caused immediate false aborts.
- After brush cleaning, first retract from `brush_center` to `brush_entry` along the
  calibrated cup axis, then lift vertically to `brush_clear_high` (`z=0.45 m`) before
  crossing to outlet 1. A direct low transfer from `brush_entry` toward the outlet can
  collide with the spectrometer.
- Never execute real robot motion merely to test a code change. Use build, unit, simulation and dry-run validation first.
- Before real motion, confirm the workspace is clear, hardware E-stop is available, the controller is healthy, the robot is stationary, and the current joints are within the trajectory start tolerance.
- Never silently bridge an arbitrary current state to a cached trajectory. Reject start mismatch or use a separately commissioned recovery route.
- A transient controller path-tolerance violation gets one bounded suffix resume:
  wait for measured settling, require the current state within `0.50 rad` of the
  unexecuted trajectory, restart from the nearest remaining point with measured
  zero velocity and at least a 0.25-second first interval. Never loop retries or
  resume an unknown/divergent pose.
- Fixed-motion startup requires fresh encoders within `0.05 rad` of commissioned
  Home. The dedicated recovery service may use the configured `0.50 rad` envelope
  and a 10-second smooth trajectory, followed by a fresh encoder confirmation.
  This was physically validated from a J4 fault pose about `0.31 rad` from Home.
  Outside that envelope, keep the arm enabled and holding for operator inspection.
  The `INIT` state must never invent an automatic Home path from an unknown pose.
- One-key start/stop must bound every ROS CLI call. It terminates stale daemon and
  non-launch `ros2 action/control/node/param/service/topic/...` clients, but never
  the active `ros2 launch` process during a healthy idempotent start. Startup READY
  also requires fresh encoders and nontrivial J2+J3+J4 holding effort; controller
  lifecycle state alone is insufficient.
- A 2026-07-27 maintenance test confirmed that a large direct interpolation from
  a gravity-loaded lowered pose to Home can initially lower joint4 further.
  After physical clearance is confirmed, manual maintenance recovery must first
  use the commissioned `safe_joint_center`, settle and verify encoders, then move
  to original Home. Do not turn this into automatic unknown-pose recovery.
- After any abnormal impact, loud mechanical noise, dropped link, dead joint, or
  suspected collision, stop motion and require a physical mechanical/wiring
  inspection plus explicit operator confirmation before re-energizing. Passing
  software state or encoder checks alone is not sufficient.
- Only one node may command `/arm_controller/follow_joint_trajectory`.
- A software stop/HMI button is not a replacement for a wired hardware E-stop.
- Config reload is allowed only while idle/paused and with no active motion goal. New config becomes active only after complete validation/compilation succeeds.
- Deleting a point referenced by any route must be rejected.
- Preserve the state-machine business invariants: the cup returns to its source outlet; before-pick position is distinct from before-place position; ERROR/ESTOP retain recovery context; ESTOP has highest priority.
- `WAIT_DETECTION_DONE` is an external-process wait, not an arm-action timeout.
  Exceeding `detection_timeout_sec` must leave the cup safely on the spectrometer,
  keep the state resumable, and publish a throttled warning. Never invent a
  detection-complete signal and never attempt a blind Home recovery from that pose.
- Do not enable real GPIO/DIDO output until the installed interface board, pinmux, permissions and electrical wiring are verified.
- Brush motor commands must use verified Modbus replies, enable nonzero communication-loss
  braking through register `0x008e`, and fail closed during application initialization.
  Do not restore the removed raw ASCII `1`/`0` USB-serial protocol.

## Known high-risk issues in the pre-refactor baseline

- MoveIt and the new motion server use a `0.05 rad` start tolerance. Do not loosen it without measured encoder/repeatability evidence.
- A route inside that start tolerance begins at the latest measured joint position,
  with zero initial velocity/acceleration, before following the cached trajectory.
  This removes the abrupt catch-up that occurred when each new controller goal
  restarted at the theoretical cached point. Arbitrary start mismatches are still
  rejected. MIT goal tolerance is `0.06 rad`, based on measured static endpoint
  errors up to about `0.057 rad`; path tolerances remain unchanged.
- The spectrometer state machine calls robot actions synchronously, so its configured action timeout cannot interrupt a blocked action.
- Real E-stop, outlet signals and spectrometer-complete integration are not fully wired; some paths still use manual/simulation services.
- The legacy `clean_brush` roll has been normalized from the invalid-looking `30.0 rad`
  to `0.523599 rad` (30 degrees). This is an engineering assumption, not completed
  physical validation; confirm brush direction at low speed before insertion.
- Spectrometer sensor correction axis is inconsistent across historical docs/config (X versus Y). Confirm physical direction before enabling sensor mode.
- HMI exposes full point/route CRUD for `motion_catalog.yaml`. Saves use schema checks, Motion Server compilation, atomic replacement and automatic rollback. The legacy numeric point editor and MoveIt pose-tuner controls have been removed; do not add a second point source.
- The optimized catalog has 24 points and 63 routes. Only 6 points carry the `tunable`
  tag and appear in the normal HMI view; `advanced` hover/recovery points are hidden by
  default. Do not reintroduce per-action approach/pre/near/retreat points unless measured
  collision evidence requires them.
- Normal station motion uses one hover directly above each process point and a strict
  vertical final segment. Cross-station transfers should be continuous cached routes,
  not sequences that stop at every intermediate sample.
- `panthera_spectrometer_cell` defaults to `motion.backend: fixed_cache`. In real mode
  it is an action client of Motion Server and must not construct MoveGroup or publish arm
  trajectories directly. `legacy_moveit` is a restart-only controlled fallback.
- Fixed cleaning is also catalog-driven: pour, one shake pattern, upright return, brush
  insertion/exit, and brush-area retreat are named routes. Do not reintroduce dynamic
  wrist planning into the production path.
- Historical `.bak_*`, `bakeup/`, zip and export artifacts exist. Do not confuse them with canonical configuration.
- On 2026-07-24 `/dev/ttyS8` existed, was unused, and `b1` had `dialout` access.
  AQMD communication and short 10%/50% duty-cycle operation were physically verified
  at slave `0x02`; steady PWM read back as 100/500 and 0 after each stop command.

## HMI point commissioning

- Point commissioning uses the same fixed-trajectory compiler and `/motion/execute`
  action as production. It must not create another controller publisher or start
  MoveGroup/`panthera_pose_tuner`.
- The operator mapping is base-frame `+X=right`, `+Y=front`, `+Z=up`. Roll, pitch,
  and yaw jogs are also about base-frame axes. Button jogs change exactly one degree
  of freedom. Direct coordinate entry may change XYZ/RPY together, but the backend
  rejects a target more than 20 mm or 10 degrees from the settled measured TCP.
- MIT static loaded deflection can make a small Cartesian command differ from the
  measured TCP near the extended outlet pose. Translation requests are limited to
  `2..20 mm` and rotations to `0.1..10 deg`. The 0.5/1 mm options were removed after
  physical testing showed a 0.385 mm maximum TCP repeatability spread, making a
  sub-2 mm command direction-dependent at this tool extension. With the commissioned
  gains, a 2 mm +Z/-Z test achieved 0.42/0.13 mm final error. Debug
  motion compilation starts from
  the current measured joints/TCP, not the previous
  nominal endpoint, and compiles one Cartesian line. Before and after a jog, accept
  the TCP only after six samples at 0.2-second intervals stay within 0.5 mm and
  0.25 degrees (maximum wait 8 seconds); use the six-sample mean XYZ rather than a
  noisy final frame. The trajectory always starts from measured
  joints. Apply the operator's physical target delta to the last command target,
  preserving the learned `commanded - measured` MIT load bias instead of resetting
  the command to the gravity-deflected measured TCP. Precision compensation then
  accumulates on that command target. After settling, it may make at most eight
  same-direction six-DOF corrections. Above 3 mm use 70% of position residual capped
  at 3 mm; at or below 3 mm use 35%. Orientation uses 60% above one degree and 30%
  below it, capped at 0.75 degrees. Stop at the 0.5 mm/0.5 degree acceptance
  threshold. Encoder quantization may make one sample temporarily worse, so do not
  abort on the first regression. Never reverse
  back to an earlier command: physical validation proved the loaded MIT endpoint is
  path-dependent and a nominal "best-command rollback" made accuracy worse. If the
  bounded corrections still do not converge, reset only the internal load-bias
  reference to the measured pose so a failed command cannot contaminate the next
  jog. Always return the full XYZ residual trace for diagnosis. A rejected input limit must
  leave the debug session ready. Direct-coordinate validation allows 0.5 mm numerical
  tolerance around the advertised 20 mm limit because the displayed TCP and the
  backend's settled sample are not simultaneous.
- Entering a tunable point pauses the state machine, verifies or recovers to original
  Home, then runs `home_to_safe_center` and the point's validated
  `debug_safe_to_<point>` route. Exiting must not replay inverse jogs: MIT endpoint
  error accumulates and made that sequence diverge. Instead, align the measured
  current state into `debug_<point>_to_safe` within its dedicated 0.50 rad limit,
  then run `safe_center_to_home`; automatic mode remains paused. If either safe-exit
  route fails, keep power enabled and use encoder-confirmed Home recovery as the
  final fallback.
- All commissioning operations are serialized. Gripper and brush commands are
  accepted only in an active, ready commissioning session. The brush remains owned
  by `panthera_spectrometer_cell` through the verified Modbus driver.
- While a commissioning session is active, the HMI backend must reject production
  auto/workflow/cycle commands, global speed changes, and external catalog/config
  reloads even if another browser bypasses disabled buttons. Only the commissioning
  save paths may perform their validated internal atomic reload. Emergency stop,
  clear-estop, reset, and staying paused remain available.
- Production outlet and detection signals are state-gated in the HMI backend as
  well as in the browser. A cycle starts only from an empty `IDLE`/`WAIT_DISCHARGE`
  boundary, and detection completion is accepted only from
  `WAIT_DETECTION_DONE` (or a pause whose resume target is that state).
- `spectrometer_cell.state_age_sec` is telemetry freshness, not time spent in the
  current state. The HMI must label it as a state-data update age, never as state
  duration.
- Commissioning translation jogs accept 2-20 mm per command; rotation jogs
  accept 0.1-10 degrees. Keep the browser limits and backend validation identical.
- The red HMI control is explicitly a software stop and must never be labelled or
  presented as a substitute for the wired hardware E-stop.
- Save persists the final nominal commanded pose, not the gravity-loaded measured
  TCP, so replaying the point does not double-count elastic deflection. It updates
  the selected canonical point and declared translation followers, patches only
  their `xyz`/`rpy` YAML values while preserving comments/layout, compiles every
  enabled route, atomically replaces the real source path behind the install
  symlink, and reloads only on complete success. A compile/reload failure must
  restore the previous catalog.
- The password-protected "zero" control sets only a temporary relative display
  reference for the current commissioning session. It does not reset motor encoders,
  alter the commissioned Home vector, or persist a hardware zero. Per the operator's
  2026-07-28 decision, an incorrect password is rejected but repeated failures must
  not lock the HMI or impose a retry delay.
- The 2026-07-28 physical zero-control validation rejected an incorrect password,
  accepted the configured password, preserved Home and the point catalog, and
  returned all six relative values to zero after re-zeroing. The first reference
  residual was within 0.36 mm and 0.11 degrees. A subsequent base `+Z 2.0 mm` jog
  measured `+2.41 mm` on the requested axis with 2.27 mm cross-axis elastic drift;
  the command completed safely, the UI reports that drift, and exit returned to
  encoder-confirmed Home.
- The 2026-07-28 compensated-coordinate commissioning validation at
  `outlet_1_grasp` proved the durable MIT load-bias rule. A combined
  `(+1,-1,+1) mm` plus `+0.5 degree yaw` target finished at 0.70 mm position and
  0.23 degree orientation error. Subsequent bias-preserving `+Z 1.0 mm` and
  `+Z 0.5 mm` jogs finished without correction at 0.67 mm/0.04 degree and
  0.41 mm/negligible orientation error. A real Edge coordinate-entry click of
  `+X 0.5 mm` finished at 0.81 mm/0.11 degree with no browser errors. No test
  point was saved; safe exit returned to encoder-confirmed Home.
- With the final `75/105/135/135/75/75` gains, empty-tool debug-path
  repeatability over three independent Home entries was 0.291 mm at
  `outlet_1_grasp`, 0.563 mm at `outlet_2_grasp`, 0.238 mm at
  `spectrometer_pick`, 0.099 mm at `clean_dump`, and 0.440 mm at
  `brush_center`. Mean measured-minus-commanded XYZ offsets were respectively
  `(+0.33,+0.27,-0.40)`, `(+0.10,+0.16,+1.03)`,
  `(+1.36,+0.20,-2.06)`, `(+0.60,-0.44,-1.65)`, and
  `(-1.58,-0.03,-2.20)` mm. These offsets are not proof that the physically
  tuned station points are wrong: they include model/load deflection, and the
  production approach path and cup payload differ from the empty debug path.
  Never auto-apply their inverse to the catalog. Instrument the production path
  with the real cup and require repeated physical acceptance before any
  route-specific endpoint compensation.
- The same session passed all 12 bidirectional 2 mm / 0.5 degree XYZ and
  Roll/Pitch/Yaw jogs at `clean_dump`; maximum position/orientation error was
  0.490 mm/0.360 degrees. If bounded correction exhausts while
  `within_step_tolerance` is false, the API and browser must not present the
  command as an ordinary green success merely because the trajectory executed.

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

As of 2026-07-28, focused tests report 17/17 Motion functional tests and 14/14 HMI
functional tests green. Hardware and state-machine compile/static checks pass; each
still has one legacy whole-package `ament_uncrustify` failure. Do not mass-format
vendor/legacy packages during a functional change; reduce that debt in a dedicated
commit.

The 2026-07-28 HMI/production acceptance used Edge through Playwright's browser
protocol, not screen-coordinate automation. XYZ plus roll/pitch/yaw jogs, safe
entry/exit and a real format-preserving hot reload were exercised. Representative
2 mm translation results were `1.43..2.51 mm`; 0.5-degree rotations measured
`0.4..0.7 deg`. Production then passed 10/10 independent stop/cold-start/full
OUTLET_1 cycles at speed scale 1.0. Each stop verified Home, brush stop and clean
process ownership; every start reached READY on attempt 1. Full-cycle times were
`59.70..61.96 s`, final gripper open positions were about `49.84..50.12 mm`, laser
data remained fresh, and the Edge page had no script/console errors.

The camera is optional and must never be part of startup READY. A connected
Gemini305 previously produced both 1280x800 RGB and depth frames while the arm and
laser stayed healthy. In the 10-cycle acceptance, a later camera restart hit vendor
`openUsbDevice failed`; its isolated retry process did not crash the HMI or interrupt
the full production cycle, and the next one-key stop removed it cleanly.

The 2026-07-28 physical regression after the CAN ordering fix completed three
combined Home-to-outlet-wait round trips with simultaneous gripper commands, then
three complete safe stop/cold-start cycles. All 12 arm routes and 12 gripper actions
completed on their first attempt; all starts reached READY on attempt 1, all stops
verified original Home, and no stale non-launch ROS CLI remained. A separate
three-cycle telemetry run had a 30.5 ms maximum joint-state sample gap and no arm
dropout.

The subsequent 50-round hot stability test ran 100 full-speed cached arm routes
(`home_to_outlet_wait_continuous` and its return) with 100 simultaneous gripper
actions. All completed on the first attempt in 181.69 seconds; no abort, retry,
recovery, path-tolerance error, stale joint state, or motor fault occurred. Across
18,168 joint-state samples, the largest sample gap was 41.24 ms, maximum measured
joint speed was 1.321 rad/s, minimum combined J2+J3+J4 effort was 3.479 Nm, and
maximum final Home error was 0.02947 rad. Raw velocity-difference acceleration
spikes remain encoder/SDK quantization artifacts; use position continuity, commanded
trajectory limits, and sustained motion evidence before classifying one as a
mechanical jerk.

The final per-joint MIT refinement used synchronized
`/arm_controller/controller_state` desired/actual data at full production speed.
The accepted `Kp=[75,105,135,135,75,75]`, `Kd=5.5` profile reduced J2/J3
dynamic RMS error by about 5.5%/4.4% from the preceding profile without adding
settling motion. Ten consecutive high-speed Home/wait routes then completed
first try in 19.45 seconds. HMI commissioning passed bidirectional 2, 5 and
20 mm Z jogs with maximum position/orientation error 0.476 mm/0.396 degrees,
safe exit returned Home, and an OUTLET_2 production cycle completed in
59.94 seconds without error or recovery. A final cold start with no environment
override logged the accepted gains and the existing four-pose gravity scales.

The commissioning-only fixed motion launch is:

```bash
ros2 launch panthera_motion fixed_motion_bringup.launch.py default_speed_scale:=0.20
```

It starts hardware/controllers plus the Motion Server without MoveGroup. Do not start legacy real-motion nodes beside it.
It defaults to `mit_gravity_compensation` with the commissioned `60/5.5` gains.
Use `control_mode:=position_velocity` only for controlled rollback.

The full fixed-backend production entry point is:

```bash
ros2 launch panthera_motion fixed_spectrometer_cell.launch.py default_speed_scale:=0.20
```

It adds the spectrometer state machine and optional HMI while keeping Motion Server as
the only arm-trajectory owner. The configured `motion.fixed_start_point` is a declaration,
not a homing command; the server still verifies fresh encoder positions before execution.

Runtime outputs belong in `build/`, `install/`, `log/`, `validation_logs/` and `.runtime/`; do not commit them.

The production lifecycle entry points are `scripts/start_workcell.sh` and
`scripts/stop_workcell.sh`. Start owns one `setsid` launch group for
`fixed_spectrometer_cell.launch.py` at speed 1.0 and reports success only after
controllers, Motion Server, fresh Home encoders, and HMI services pass. Stop
must verify an empty cycle, settled Motion Server, and original Home before
disabling, then terminate the owned group and verify all related processes are
gone. Laser startup and fresh data are required; camera startup is optional and
excluded from READY. Never restore the legacy MoveIt/workflow startup chain to
these production scripts. `--force` is an explicit maintenance escape hatch,
not a normal shutdown path.
- Production ROS control is local to the IPC: `ROS_LOCALHOST_ONLY=1` and
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` restrict DDS to UDP loopback while avoiding
  stale Fast DDS shared-memory locks. Remote operation uses HTTP/SSH. Never run
  production with UDP-only transport unless localhost isolation is also enabled;
  otherwise cable removal can split local participants and strand the controller.
- Start/stop must stop any stale ROS 2 CLI daemon before health queries.
  `ros2 control list_controllers` on this Humble install otherwise reuses a daemon
  left in `!rclpy.ok()` and falsely reports startup failure while controllers run.
- `b1` must belong to the `realtime` group and
  `/etc/security/limits.d/99-panthera-realtime.conf` must match
  `config/system/99-panthera-realtime.conf`. A fresh login must report
  `ulimit -r = 99`, `ulimit -l = unlimited`, and the ros2_control update thread must
  run as `SCHED_FIFO 50`. Missing realtime scheduling is a production preflight
  failure, not an ignorable warning.
- Keep the top-level production launch arguments distinct:
  `hardware_config_file` is the vendor `Follower_absolute.yaml`, while
  `cell_config_file` is `spectrometer_cell.yaml`. Reusing the generic name
  `config_file` across nested launches caused each consumer to receive the
  other's YAML and must not be reintroduced.
- Repeated disable/enable cycles can let joint4 settle downward by roughly
  10-15 mrad per cycle. Normal shutdown therefore calls
  `/spectrometer_cell/recover_home` and confirms fresh encoders before
  terminating ros2_control. Startup may auto-recover only a small drift within
  0.10 rad; a larger mismatch stays powered and requires operator inspection.
- Recovery trajectories must contain an explicit first point sampled from fresh
  encoders before the Home endpoint. A one-point recovery lets the controller
  interpolate from its stale previous desired state and can jerk a joint. Manual
  reset is valid from every state, and manual pause must allow `IDLE -> PAUSED`;
  these transitions are required by HMI commissioning and startup recovery.
- Fixed outlet pickup has two valid logical starts. From `home_near`, select
  `home_to_outlet_1_grasp_smooth` or `home_to_outlet_2_grasp_smooth` by outlet;
  from `outlet_wait`, select the matching `outlet_wait_to_outlet_*_grasp` route.
  Do not collapse this into an outlet-1-only Home special case. Both outlet-2
  variants completed consecutive full physical cycles at 100% speed in about
  60 seconds per cycle with the production MIT gains.
- Shutdown may encounter `ERROR` with a stale active task after safety recovery.
  It may clear that context only when the gripper and spectrometer are empty and
  fresh encoders independently verify commissioned Home. Otherwise it must keep
  the hardware powered and refuse shutdown; never use task flags alone to infer
  physical safety.
- `outlet_wait` is about 1.12 rad from Home and is outside the 0.5 rad unknown-pose
  recovery envelope. When that logical start is known, `/recover_home` must run
  the compiled `outlet_wait_to_home_continuous` route through `safe_joint_center`;
  do not widen the unknown-pose envelope. Physical validation completed this
  route in 1.80 seconds and encoder-confirmed Home within 0.00854 rad.
- A deferred pause request is one-shot: once `pauseRequested` is set, do not
  regenerate a higher-priority `PAUSE_AUTO` event every tick or action completion
  will be starved. At an action boundary, `pausedFromState` is the next unexecuted
  state (`MEASURE...`, `START_DETECTION`, `CLEAN_CUP`, `RETURN_CUP`, or
  `COMPLETE_CYCLE`), never the action that already succeeded.
- Cold controller discovery can take 10-15 seconds. Keep
  `gripper.action_server_wait_sec` at 20 seconds or longer so INIT cannot race the
  sequential controller spawners. Hardware reconnect attempts must retain a 5-second
  cooldown; reconnecting immediately after disable produced a transient SDK `999`
  encoder frame during real testing.

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
