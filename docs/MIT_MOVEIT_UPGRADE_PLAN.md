# MIT MoveIt Upgrade Plan

## Goal

Integrate the vendor's ROS 2 Humble MIT MoveIt mode into the commissioned
workcell without replacing the validated fixed trajectory catalog or removing
the existing hardware fault filtering. The arm joints use the vendor-recommended
gains:

- `Kp = [60, 60, 60, 60, 60, 60]`
- `Kd = [5, 5, 5, 5, 5, 5]`

The gripper remains on its existing retained position command path.

## Baseline and upstream

- Workcell baseline: `23500b7` (`validated-motion-20260718`).
- Vendor repository: `HighTorque-Robotics/Panthera-HT_ROS2`, branch `humble`.
- Vendor MIT commit reviewed: `3815fce1a2b805e39544f66368ed46a0089a5cb0`.
- Rollback: switch back to `main`/`validated-motion-20260718` and rebuild.

The vendor commit is not merged wholesale. The workcell contains later
stability, command validation, gripper retention, fixed trajectory, and HMI
changes which must not be overwritten.

## Design

1. Add the vendor `mit_gravity_compensation` hardware mode to
   `panthera_hardware`.
2. Use Pinocchio to load the installed robot URDF and calculate model gravity
   torque from current joint positions at each controller update.
3. Send the existing trajectory position and velocity targets through the
   vendor SDK `posVelTorqueKpKd()` command with clamped gravity feed-forward,
   `Kp`, and `Kd`.
4. Keep the existing communication sentinel rejection, state filtering,
   command finite/range checks, SDK return checking, and write suppression when
   the motors are unavailable.
5. Reject malformed MIT gain vectors instead of silently running unexpected
   defaults. Gains must contain exactly six finite, non-negative values.
6. Keep the normal `position_velocity` launch path available as a rollback
   mode.

## Launch integration

- Add the vendor-compatible commissioning entry point
  `hardware_moveit_rviz_mit.launch.py`.
- Expose `mit_kp` and `mit_kd` as comma-separated launch arguments.
- Make the fixed-motion production entry points default to
  `mit_gravity_compensation` with the commissioned `60/5` gains.
- Preserve explicit `control_mode:=position_velocity` support for immediate
  rollback.
- Do not add MoveGroup to the production fixed-cache path; MIT is a hardware
  control mode beneath both MoveIt and the deterministic trajectory server.

## Safety and failure behavior

- No real motion is used for build or software validation.
- MIT configuration fails closed if the URDF/dynamics model or gain vector
  cannot be loaded.
- Gravity torque is checked for finite values and clamped to each joint's
  configured torque limit before sending.
- Motor-unavailable sentinel handling continues to suppress all arm writes.
- Only the existing `arm_controller` remains the
  `/arm_controller/follow_joint_trajectory` owner.
- The original Home vector and validated motion catalog are unchanged.

## Validation

1. Verify Pinocchio and the installed URDF are available on the target.
2. Build `panthera_hardware`, `panthera_ht_config`, and packages depending on
   them.
3. Run focused package tests and report pre-existing lint failures separately.
4. Run Python launch syntax checks.
5. Verify `--show-args` exposes MIT mode and the exact `60/5` defaults.
6. Verify the non-MIT rollback launch still parses.
7. Inspect the final Git diff to ensure no motion catalog or workcell point
   changed.
8. Commit and push only after all non-motion checks pass.

## Hardware acceptance (separate controlled step)

After software validation, perform a low-speed, short-range test from Home with
the workspace clear and hardware E-stop available. Confirm startup hold,
direction, gravity compensation, following error, temperature/current, and
stop behavior before allowing a complete production cycle. Hardware acceptance
is not implied by a successful build.
