#include "panthera_spectrometer_cell/Config.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace panthera_spectrometer_cell
{
namespace
{

double readDouble(const YAML::Node & node, const std::string & key, double default_value)
{
  return node && node[key] ? node[key].as<double>() : default_value;
}

int readInt(const YAML::Node & node, const std::string & key, int default_value)
{
  return node && node[key] ? node[key].as<int>() : default_value;
}

bool readBool(const YAML::Node & node, const std::string & key, bool default_value)
{
  return node && node[key] ? node[key].as<bool>() : default_value;
}

std::string readString(
  const YAML::Node & node,
  const std::string & key,
  const std::string & default_value)
{
  return node && node[key] ? node[key].as<std::string>() : default_value;
}

Vec3 readVec3(const YAML::Node & node, const std::string & key, Vec3 default_value)
{
  if (!node || !node[key]) {
    return default_value;
  }

  const auto value = node[key];
  if (!value.IsSequence() || value.size() != 3) {
    throw std::runtime_error(key + " must be a 3-element array");
  }

  return Vec3{value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}

std::vector<double> readDoubleVector(
  const YAML::Node & node,
  const std::string & key,
  const std::vector<double> & default_value)
{
  if (!node || !node[key]) {
    return default_value;
  }

  const auto value = node[key];
  if (!value.IsSequence()) {
    throw std::runtime_error(key + " must be a numeric array");
  }

  std::vector<double> values;
  values.reserve(value.size());
  for (const auto & item : value) {
    values.push_back(item.as<double>());
  }
  return values;
}

void addDefaultPose(
  std::map<std::string, PoseConfig> & poses,
  const std::string & name,
  Vec3 xyz,
  Vec3 rpy)
{
  poses[name] = PoseConfig{name, xyz, rpy};
}

}  // namespace

WorkcellConfig WorkcellConfig::withDefaults()
{
  WorkcellConfig config;

  config.outlets[OutletId::OUTLET_1] = OutletConfig{
    OutletId::OUTLET_1,
    "OUTLET_1",
    "outlet_1_done",
    "outlet_1_pick_approach",
    "outlet_1_pick",
    "outlet_1_return_approach",
    "outlet_1_return"};

  config.outlets[OutletId::OUTLET_2] = OutletConfig{
    OutletId::OUTLET_2,
    "OUTLET_2",
    "outlet_2_done",
    "outlet_2_pick_approach",
    "outlet_2_pick",
    "outlet_2_return_approach",
    "outlet_2_return"};

  addDefaultPose(config.namedPoses, "outlet_1_pick_approach", {-0.02500, -0.42853, 0.180}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_1_pick", {-0.02500, -0.42853, 0.140}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_1_return_approach", {-0.02500, -0.42853, 0.180}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_1_return", {-0.02500, -0.42853, 0.140}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_2_pick_approach", {0.15500, -0.42853, 0.180}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_2_pick", {0.15500, -0.42853, 0.140}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_2_return_approach", {0.15500, -0.42853, 0.180}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "outlet_2_return", {0.15500, -0.42853, 0.140}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "spectrometer_base", {0.58414, -0.12103, 0.290}, {0.0, 0.0, 0.0});
  addDefaultPose(config.namedPoses, "outlet_wait_mid", {0.06500, -0.35000, 0.180}, {0.0, 0.0, -1.5708});
  addDefaultPose(config.namedPoses, "spectrometer_wait_ref", {0.44414, -0.12103, 0.360}, {0.0, 0.0, 0.0});
  addDefaultPose(config.namedPoses, "clean_approach", {0.250, 0.260, 0.240}, {0.0, 0.0, 1.5708});
  addDefaultPose(config.namedPoses, "clean_dump", {0.250, 0.400, 0.120}, {0.0, 0.0, 1.5708});
  addDefaultPose(config.namedPoses, "clean_leave", {0.250, 0.260, 0.240}, {0.0, 0.0, 1.5708});

  return config;
}

WorkcellConfig WorkcellConfig::loadFromFile(const std::string & path)
{
  WorkcellConfig config = WorkcellConfig::withDefaults();
  const YAML::Node root = YAML::LoadFile(path);

  const auto loop = root["loop"];
  config.loop.tickRateHz = readDouble(loop, "tick_rate_hz", config.loop.tickRateHz);
  config.loop.waitDischargeSoftTimeoutSec =
    readDouble(loop, "wait_discharge_soft_timeout_sec", config.loop.waitDischargeSoftTimeoutSec);
  config.loop.actionTimeoutSec =
    readDouble(loop, "action_timeout_sec", config.loop.actionTimeoutSec);
  config.loop.sensorTimeoutSec =
    readDouble(loop, "sensor_timeout_sec", config.loop.sensorTimeoutSec);
  config.loop.spectrometerStartTimeoutSec =
    readDouble(loop, "spectrometer_start_timeout_sec", config.loop.spectrometerStartTimeoutSec);
  config.loop.scanDurationSec =
    readDouble(loop, "scan_duration_sec", config.loop.scanDurationSec);
  config.loop.detectionTimeoutSec =
    readDouble(loop, "detection_timeout_sec", config.loop.detectionTimeoutSec);
  config.loop.resetTimeoutSec =
    readDouble(loop, "reset_timeout_sec", config.loop.resetTimeoutSec);

  const auto motion = root["motion"];
  config.motion.backend = readString(motion, "backend", config.motion.backend);
  config.motion.fixedStartPoint =
    readString(motion, "fixed_start_point", config.motion.fixedStartPoint);
  config.motion.motionServerWaitSec =
    readDouble(motion, "motion_server_wait_sec", config.motion.motionServerWaitSec);
  config.motion.transientRetryCount =
    readInt(motion, "transient_retry_count", config.motion.transientRetryCount);
  config.motion.transientRetryDelaySec = readDouble(
    motion, "transient_retry_delay_sec", config.motion.transientRetryDelaySec);
  config.motion.cancelWaitSec =
    readDouble(motion, "cancel_wait_sec", config.motion.cancelWaitSec);
  config.motion.velocityScale = readDouble(motion, "velocity_scale", config.motion.velocityScale);
  config.motion.accelerationScale =
    readDouble(motion, "acceleration_scale", config.motion.accelerationScale);
  config.motion.planningTimeSec =
    readDouble(motion, "planning_time_sec", config.motion.planningTimeSec);
  config.motion.planningAttempts =
    readInt(motion, "planning_attempts", config.motion.planningAttempts);
  config.motion.goalPositionTolerance =
    readDouble(motion, "goal_position_tolerance", config.motion.goalPositionTolerance);
  config.motion.goalOrientationTolerance =
    readDouble(motion, "goal_orientation_tolerance", config.motion.goalOrientationTolerance);
  config.motion.goalJointTolerance =
    readDouble(motion, "goal_joint_tolerance", config.motion.goalJointTolerance);
  config.motion.cartesianEefStep =
    readDouble(motion, "cartesian_eef_step", config.motion.cartesianEefStep);
  config.motion.cartesianJumpThreshold =
    readDouble(motion, "cartesian_jump_threshold", config.motion.cartesianJumpThreshold);
  config.motion.cartesianMinFraction =
    readDouble(motion, "cartesian_min_fraction", config.motion.cartesianMinFraction);
  config.motion.outletApproachY =
    readDouble(motion, "outlet_approach_y", config.motion.outletApproachY);
  config.motion.outletNearY =
    readDouble(motion, "outlet_near_y", config.motion.outletNearY);
  config.motion.outletHighZ =
    readDouble(motion, "outlet_high_z", config.motion.outletHighZ);
  config.motion.outletGripZ =
    readDouble(motion, "outlet_grip_z", config.motion.outletGripZ);
  config.motion.outletTransferZ =
    readDouble(motion, "outlet_transfer_z", config.motion.outletTransferZ);
  config.motion.spectrometerApproachXOffset =
    readDouble(motion, "spectrometer_approach_x_offset", config.motion.spectrometerApproachXOffset);
  config.motion.spectrometerHighZ =
    readDouble(motion, "spectrometer_high_z", config.motion.spectrometerHighZ);
  config.motion.cleanHighZ =
    readDouble(motion, "clean_high_z", config.motion.cleanHighZ);
  config.motion.cleanApproachY =
    readDouble(motion, "clean_approach_y", config.motion.cleanApproachY);
  config.motion.cleanPreZ =
    readDouble(motion, "clean_pre_z", config.motion.cleanPreZ);
  config.motion.cleanReadyZ =
    readDouble(motion, "clean_ready_z", config.motion.cleanReadyZ);
  config.motion.safeJointPose =
    readDoubleVector(motion, "safe_joint_pose", config.motion.safeJointPose);
  config.motion.homeJointPose =
    readDoubleVector(motion, "home_joint_pose", config.motion.homeJointPose);
  config.motion.errorRecoveryDurationSec = readDouble(
    motion, "error_recovery_duration_sec", config.motion.errorRecoveryDurationSec);
  config.motion.errorRecoveryPathToleranceRad = readDouble(
    motion, "error_recovery_path_tolerance_rad", config.motion.errorRecoveryPathToleranceRad);
  config.motion.errorRecoveryGoalToleranceRad = readDouble(
    motion, "error_recovery_goal_tolerance_rad", config.motion.errorRecoveryGoalToleranceRad);
  config.motion.errorRecoverySettleToleranceRad = readDouble(
    motion, "error_recovery_settle_tolerance_rad", config.motion.errorRecoverySettleToleranceRad);
  config.motion.errorRecoveryTimeoutMarginSec = readDouble(
    motion, "error_recovery_timeout_margin_sec", config.motion.errorRecoveryTimeoutMarginSec);

  const auto gripper = root["gripper"];
  config.gripper.openPosition = readDouble(gripper, "open_position", config.gripper.openPosition);
  config.gripper.closePosition = readDouble(gripper, "close_position", config.gripper.closePosition);
  config.gripper.openDurationSec =
    readDouble(gripper, "open_duration_sec", config.gripper.openDurationSec);
  config.gripper.closeDurationSec =
    readDouble(gripper, "close_duration_sec", config.gripper.closeDurationSec);
  config.gripper.actionServerWaitSec =
    readDouble(gripper, "action_server_wait_sec", config.gripper.actionServerWaitSec);
  config.gripper.commandTimeoutMarginSec = readDouble(
    gripper, "command_timeout_margin_sec", config.gripper.commandTimeoutMarginSec);
  config.gripper.positionToleranceM =
    readDouble(gripper, "position_tolerance_m", config.gripper.positionToleranceM);
  config.gripper.graspHoldGoalToleranceM = readDouble(
    gripper, "grasp_hold_goal_tolerance_m", config.gripper.graspHoldGoalToleranceM);
  config.gripper.settledVelocityToleranceMps = readDouble(
    gripper,
    "settled_velocity_tolerance_mps",
    config.gripper.settledVelocityToleranceMps);
  config.gripper.settleTimeoutSec =
    readDouble(gripper, "settle_timeout_sec", config.gripper.settleTimeoutSec);
  config.gripper.graspContactMinClosureM = readDouble(
    gripper, "grasp_contact_min_closure_m", config.gripper.graspContactMinClosureM);
  config.gripper.graspContactConfirmSec = readDouble(
    gripper, "grasp_contact_confirm_sec", config.gripper.graspContactConfirmSec);
  config.gripper.retryCount =
    readInt(gripper, "retry_count", config.gripper.retryCount);

  const auto positioning = root["positioning"];
  config.positioning.mode =
    readString(positioning, "mode", config.positioning.mode);
  config.positioning.fixedAxisPositionMm = readDouble(
    positioning,
    "fixed_axis_position_mm",
    config.positioning.fixedAxisPositionMm);

  const auto simulation = root["simulation"];
  config.simulation.enabled = readBool(simulation, "enabled", config.simulation.enabled);
  config.simulation.autoGenerateDischarge =
    readBool(simulation, "auto_generate_discharge", config.simulation.autoGenerateDischarge);
  config.simulation.autoDischargeIntervalSec =
    readDouble(simulation, "auto_discharge_interval_sec", config.simulation.autoDischargeIntervalSec);
  config.simulation.fakeDetectionTimeSec =
    readDouble(simulation, "fake_detection_time_sec", config.simulation.fakeDetectionTimeSec);
  config.simulation.actionDelayMs =
    readInt(simulation, "action_delay_ms", config.simulation.actionDelayMs);
  config.simulation.randomFailureRate =
    readDouble(simulation, "random_failure_rate", config.simulation.randomFailureRate);
  config.simulation.randomSeed =
    static_cast<unsigned int>(readInt(simulation, "random_seed", static_cast<int>(config.simulation.randomSeed)));

  const auto axis = root["spectrometer_axis"];
  config.spectrometerAxis.laserMinMm =
    readDouble(axis, "laser_min_mm", config.spectrometerAxis.laserMinMm);
  config.spectrometerAxis.laserMaxMm =
    readDouble(axis, "laser_max_mm", config.spectrometerAxis.laserMaxMm);
  config.spectrometerAxis.axisZeroLaserMm =
    readDouble(axis, "axis_zero_laser_mm", config.spectrometerAxis.axisZeroLaserMm);
  config.spectrometerAxis.axisScaleMPerMm =
    readDouble(axis, "axis_scale_m_per_mm", config.spectrometerAxis.axisScaleMPerMm);
  config.spectrometerAxis.basePoseName =
    readString(axis, "base_pose_name", config.spectrometerAxis.basePoseName);
  config.spectrometerAxis.axis = readString(axis, "axis", config.spectrometerAxis.axis);
  config.spectrometerAxis.placeOffsetXyz =
    readVec3(axis, "place_offset_xyz", config.spectrometerAxis.placeOffsetXyz);
  config.spectrometerAxis.pickOffsetXyz =
    readVec3(axis, "pick_offset_xyz", config.spectrometerAxis.pickOffsetXyz);

  const auto cleaning = root["cleaning"];
  config.cleaning.approachPose =
    readString(cleaning, "approach_pose", config.cleaning.approachPose);
  config.cleaning.dumpPose = readString(cleaning, "dump_pose", config.cleaning.dumpPose);
  config.cleaning.pourWristJointIndex =
    readInt(cleaning, "pour_wrist_joint_index", config.cleaning.pourWristJointIndex);
  config.cleaning.pourDirection =
    readInt(cleaning, "pour_direction", config.cleaning.pourDirection);
  config.cleaning.pourAngleRad =
    readDouble(cleaning, "pour_angle_rad", config.cleaning.pourAngleRad);
  config.cleaning.pourVelocityScale =
    readDouble(cleaning, "pour_velocity_scale", config.cleaning.pourVelocityScale);
  config.cleaning.pourAccelerationScale =
    readDouble(cleaning, "pour_acceleration_scale", config.cleaning.pourAccelerationScale);
  config.cleaning.pourHoldSec =
    readDouble(cleaning, "pour_hold_sec", config.cleaning.pourHoldSec);
  config.cleaning.shakeCount = readInt(cleaning, "shake_count", config.cleaning.shakeCount);
  config.cleaning.shakeAngleRad =
    readDouble(cleaning, "shake_angle_rad", config.cleaning.shakeAngleRad);
  config.cleaning.shakeHoldSec =
    readDouble(cleaning, "shake_hold_sec", config.cleaning.shakeHoldSec);
  config.cleaning.brushEnabled =
    readBool(cleaning, "brush_enabled", config.cleaning.brushEnabled);
  config.cleaning.brushPose = readString(cleaning, "brush_pose", config.cleaning.brushPose);
  config.cleaning.brushVelocityScale =
    readDouble(cleaning, "brush_velocity_scale", config.cleaning.brushVelocityScale);
  config.cleaning.brushAccelerationScale =
    readDouble(cleaning, "brush_acceleration_scale", config.cleaning.brushAccelerationScale);
  config.cleaning.brushApproachOffsetXyz =
    readVec3(cleaning, "brush_approach_offset_xyz", config.cleaning.brushApproachOffsetXyz);
  config.cleaning.brushUprightRetreatOffsetXyz =
    readVec3(
      cleaning,
      "brush_upright_retreat_offset_xyz",
      config.cleaning.brushUprightRetreatOffsetXyz);
  config.cleaning.brushStrokeCount =
    readInt(cleaning, "brush_stroke_count", config.cleaning.brushStrokeCount);
  config.cleaning.brushStrokeOffsetXyz =
    readVec3(cleaning, "brush_stroke_offset_xyz", config.cleaning.brushStrokeOffsetXyz);
  config.cleaning.brushHoldSec =
    readDouble(cleaning, "brush_hold_sec", config.cleaning.brushHoldSec);
  config.cleaning.brushMotorStopDelaySec =
    readDouble(cleaning, "brush_motor_stop_delay_sec", config.cleaning.brushMotorStopDelaySec);
  config.cleaning.motorRs485Enabled =
    readBool(cleaning, "motor_rs485_enabled", config.cleaning.motorRs485Enabled);
  config.cleaning.motorRs485Device =
    readString(cleaning, "motor_rs485_device", config.cleaning.motorRs485Device);
  config.cleaning.motorRs485Baudrate =
    readInt(cleaning, "motor_rs485_baudrate", config.cleaning.motorRs485Baudrate);
  config.cleaning.motorRs485SlaveId =
    readInt(cleaning, "motor_rs485_slave_id", config.cleaning.motorRs485SlaveId);
  config.cleaning.motorRs485DutyPermille =
    readInt(cleaning, "motor_rs485_duty_permille", config.cleaning.motorRs485DutyPermille);
  config.cleaning.motorRs485CommunicationTimeoutDs = readInt(
    cleaning,
    "motor_rs485_communication_timeout_ds",
    config.cleaning.motorRs485CommunicationTimeoutDs);
  config.cleaning.leavePose = readString(cleaning, "leave_pose", config.cleaning.leavePose);

  const auto outlets = root["outlets"];
  if (outlets && outlets.IsMap()) {
    for (const auto & item : outlets) {
      const std::string name = item.first.as<std::string>();
      const OutletId id = outletFromString(name);
      if (id == OutletId::NONE) {
        continue;
      }

      const auto node = item.second;
      auto outlet = config.outlets[id];
      outlet.id = id;
      outlet.name = name;
      outlet.dischargeSignal = readString(node, "discharge_signal", outlet.dischargeSignal);
      outlet.pickApproachPose = readString(node, "pick_approach_pose", outlet.pickApproachPose);
      outlet.pickPose = readString(node, "pick_pose", outlet.pickPose);
      outlet.returnApproachPose =
        readString(node, "return_approach_pose", outlet.returnApproachPose);
      outlet.returnPose = readString(node, "return_pose", outlet.returnPose);
      config.outlets[id] = outlet;
    }
  }

  const auto poses = root["named_poses"];
  if (poses && poses.IsMap()) {
    for (const auto & item : poses) {
      const std::string name = item.first.as<std::string>();
      const auto node = item.second;
      PoseConfig pose;
      pose.name = name;
      pose.xyz = readVec3(node, "xyz", pose.xyz);
      pose.rpy = readVec3(node, "rpy", pose.rpy);
      config.namedPoses[name] = pose;
    }
  }

  const auto collision_objects = root["collision_objects"];
  if (collision_objects && collision_objects.IsMap()) {
    config.collisionObjects.clear();
    for (const auto & item : collision_objects) {
      CollisionObjectConfig object;
      object.name = item.first.as<std::string>();
      const auto node = item.second;
      object.type = readString(node, "type", object.type);
      object.enabled = readBool(node, "enabled", object.enabled);
      object.frameId = readString(node, "frame_id", object.frameId);
      object.xyz = readVec3(node, "xyz", object.xyz);
      object.rpy = readVec3(node, "rpy", object.rpy);
      object.sizeXyz = readVec3(node, "size_xyz", object.sizeXyz);
      object.radius = readDouble(node, "radius", object.radius);
      object.height = readDouble(node, "height", object.height);
      config.collisionObjects.push_back(object);
    }
  }

  const auto validation = config.validate();
  if (!validation.success) {
    throw std::runtime_error(validation.message);
  }

  return config;
}

const OutletConfig * WorkcellConfig::findOutlet(OutletId outlet) const
{
  const auto it = outlets.find(outlet);
  if (it == outlets.end()) {
    return nullptr;
  }
  return &it->second;
}

const PoseConfig * WorkcellConfig::findPose(const std::string & name) const
{
  const auto it = namedPoses.find(name);
  if (it == namedPoses.end()) {
    return nullptr;
  }
  return &it->second;
}

ActionResult WorkcellConfig::validate() const
{
  if (loop.tickRateHz <= 0.0) {
    return ActionResult::fail("loop.tick_rate_hz must be > 0");
  }
  if (loop.actionTimeoutSec <= 0.0) {
    return ActionResult::fail("loop.action_timeout_sec must be > 0");
  }
  if (loop.sensorTimeoutSec <= 0.0) {
    return ActionResult::fail("loop.sensor_timeout_sec must be > 0");
  }
  if (loop.spectrometerStartTimeoutSec <= 0.0) {
    return ActionResult::fail("loop.spectrometer_start_timeout_sec must be > 0");
  }
  if (!std::isfinite(loop.scanDurationSec) || loop.scanDurationSec <= 0.0 ||
    loop.scanDurationSec > 300.0)
  {
    return ActionResult::fail("loop.scan_duration_sec must be finite and in (0, 300]");
  }
  if (positioning.mode != "fixed" && positioning.mode != "sensor_optional" &&
    positioning.mode != "sensor_offset")
  {
    return ActionResult::fail(
      "positioning.mode must be fixed, sensor_optional or sensor_offset");
  }
  if (!std::isfinite(positioning.fixedAxisPositionMm)) {
    return ActionResult::fail("positioning.fixed_axis_position_mm must be finite");
  }
  if (!std::isfinite(spectrometerAxis.laserMinMm) ||
    !std::isfinite(spectrometerAxis.laserMaxMm) ||
    !std::isfinite(spectrometerAxis.axisZeroLaserMm) ||
    !std::isfinite(spectrometerAxis.axisScaleMPerMm))
  {
    return ActionResult::fail("spectrometer_axis numeric values must be finite");
  }
  if (spectrometerAxis.laserMinMm >= spectrometerAxis.laserMaxMm) {
    return ActionResult::fail("spectrometer_axis laser_min_mm must be < laser_max_mm");
  }
  if (spectrometerAxis.axisScaleMPerMm == 0.0) {
    return ActionResult::fail("spectrometer_axis.axis_scale_m_per_mm must not be zero");
  }
  if (positioning.mode == "sensor_optional" &&
    std::abs(positioning.fixedAxisPositionMm - spectrometerAxis.axisZeroLaserMm) > 1e-9)
  {
    return ActionResult::fail(
      "sensor_optional requires fixed_axis_position_mm equal to axis_zero_laser_mm");
  }
  if (spectrometerAxis.axis != "x" && spectrometerAxis.axis != "y" && spectrometerAxis.axis != "z") {
    return ActionResult::fail("spectrometer_axis.axis must be x, y, or z");
  }
  if (motion.safeJointPose.empty()) {
    return ActionResult::fail("motion.safe_joint_pose must not be empty");
  }
  if (motion.homeJointPose.size() != 6u ||
    !std::all_of(
      motion.homeJointPose.begin(), motion.homeJointPose.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return ActionResult::fail("motion.home_joint_pose must contain six finite joint values");
  }
  if (motion.errorRecoveryDurationSec <= 0.0 ||
    motion.errorRecoveryPathToleranceRad <= 0.0 ||
    motion.errorRecoveryGoalToleranceRad <= 0.0 ||
    motion.errorRecoverySettleToleranceRad <= 0.0 ||
    motion.errorRecoveryTimeoutMarginSec <= 0.0)
  {
    return ActionResult::fail("motion error-recovery settings must be > 0");
  }
  if (motion.backend != "fixed_cache" && motion.backend != "legacy_moveit") {
    return ActionResult::fail("motion.backend must be fixed_cache or legacy_moveit");
  }
  if (motion.fixedStartPoint != "safe_joint_center" && motion.fixedStartPoint != "home_near") {
    return ActionResult::fail("motion.fixed_start_point must be safe_joint_center or home_near");
  }
  if (motion.motionServerWaitSec <= 0.0) {
    return ActionResult::fail("motion.motion_server_wait_sec must be > 0");
  }
  if (motion.transientRetryCount < 0 || motion.transientRetryCount > 5 ||
    motion.transientRetryDelaySec < 0.0 || motion.cancelWaitSec <= 0.0)
  {
    return ActionResult::fail("motion retry/cancel settings are invalid");
  }
  if (motion.outletTransferZ <= 0.0) {
    return ActionResult::fail("motion.outlet_transfer_z must be > 0");
  }
  if (motion.outletHighZ <= 0.0 || motion.outletGripZ <= 0.0 || motion.spectrometerHighZ <= 0.0 ||
    motion.cleanHighZ <= 0.0 || motion.cleanPreZ <= 0.0 || motion.cleanReadyZ <= 0.0)
  {
    return ActionResult::fail("motion path z values must be > 0");
  }
  if (gripper.closePosition > gripper.openPosition ||
    gripper.openDurationSec <= 0.0 || gripper.closeDurationSec <= 0.0 ||
    gripper.actionServerWaitSec <= 0.0 || gripper.commandTimeoutMarginSec <= 0.0 ||
    gripper.positionToleranceM <= 0.0 ||
    gripper.graspHoldGoalToleranceM <= gripper.positionToleranceM ||
    gripper.graspHoldGoalToleranceM >= (gripper.openPosition - gripper.closePosition) ||
    gripper.settledVelocityToleranceMps <= 0.0 ||
    gripper.settleTimeoutSec <= 0.0 || gripper.graspContactMinClosureM <= 0.0 ||
    gripper.graspContactMinClosureM >= (gripper.openPosition - gripper.closePosition) ||
    gripper.graspContactConfirmSec <= 0.0 || gripper.retryCount < 0 ||
    gripper.retryCount > 3)
  {
    return ActionResult::fail("gripper limits, timing, tolerance, or retry count are invalid");
  }
  if (!findPose(spectrometerAxis.basePoseName)) {
    return ActionResult::fail(
      "missing spectrometer base pose: " + spectrometerAxis.basePoseName);
  }
  if (!findPose(cleaning.approachPose) || !findPose(cleaning.dumpPose) || !findPose(cleaning.leavePose)) {
    return ActionResult::fail("cleaning poses are incomplete");
  }
  if (cleaning.brushEnabled && !findPose(cleaning.brushPose)) {
    return ActionResult::fail("cleaning brush pose is incomplete: " + cleaning.brushPose);
  }
  if (cleaning.pourWristJointIndex < 0) {
    return ActionResult::fail("cleaning.pour_wrist_joint_index must be >= 0");
  }
  if (cleaning.pourDirection < -1 || cleaning.pourDirection > 1) {
    return ActionResult::fail("cleaning.pour_direction must be -1, 0, or 1");
  }
  if (cleaning.pourAngleRad <= 0.0) {
    return ActionResult::fail("cleaning.pour_angle_rad must be > 0");
  }
  if (cleaning.pourVelocityScale <= 0.0 || cleaning.pourAccelerationScale <= 0.0) {
    return ActionResult::fail("cleaning pour velocity/acceleration scale must be > 0");
  }
  if (cleaning.brushVelocityScale <= 0.0 || cleaning.brushAccelerationScale <= 0.0) {
    return ActionResult::fail("cleaning brush velocity/acceleration scale must be > 0");
  }
  if (cleaning.brushStrokeCount < 0) {
    return ActionResult::fail("cleaning.brush_stroke_count must be >= 0");
  }
  if (motion.backend == "fixed_cache" && cleaning.brushEnabled &&
    (std::abs(cleaning.brushStrokeOffsetXyz[0]) > 1e-9 ||
    std::abs(cleaning.brushStrokeOffsetXyz[1]) > 1e-9 ||
    std::abs(cleaning.brushStrokeOffsetXyz[2]) > 1e-9))
  {
    return ActionResult::fail(
      "fixed_cache brush supports center hold only; add compiled stroke routes before setting brush_stroke_offset_xyz");
  }
  if (!std::isfinite(cleaning.brushHoldSec) || cleaning.brushHoldSec < 0.0 ||
    cleaning.brushHoldSec > 60.0)
  {
    return ActionResult::fail("cleaning.brush_hold_sec must be finite and in [0, 60]");
  }
  if (cleaning.brushMotorStopDelaySec < 0.0) {
    return ActionResult::fail("cleaning.brush_motor_stop_delay_sec must be >= 0");
  }
  if (cleaning.motorRs485Enabled && cleaning.motorRs485Device.empty()) {
    return ActionResult::fail(
      "cleaning.motor_rs485_device must not be empty when motor RS485 is enabled");
  }
  if (cleaning.motorRs485Baudrate <= 0) {
    return ActionResult::fail("cleaning.motor_rs485_baudrate must be > 0");
  }
  if (cleaning.motorRs485SlaveId < 1 || cleaning.motorRs485SlaveId > 127) {
    return ActionResult::fail("cleaning.motor_rs485_slave_id must be in [1, 127]");
  }
  if (cleaning.motorRs485DutyPermille < -1000 ||
    cleaning.motorRs485DutyPermille > 1000 ||
    cleaning.motorRs485DutyPermille == 0)
  {
    return ActionResult::fail(
      "cleaning.motor_rs485_duty_permille must be in [-1000, 1000] and non-zero");
  }
  if (cleaning.motorRs485CommunicationTimeoutDs < 1 ||
    cleaning.motorRs485CommunicationTimeoutDs > 255)
  {
    return ActionResult::fail(
      "cleaning.motor_rs485_communication_timeout_ds must be in [1, 255]");
  }

  for (const auto & item : outlets) {
    const auto & outlet = item.second;
    if (!findPose(outlet.pickApproachPose) || !findPose(outlet.pickPose) ||
      !findPose(outlet.returnApproachPose) || !findPose(outlet.returnPose))
    {
      std::ostringstream out;
      out << "outlet " << outlet.name << " has missing pick/return poses";
      return ActionResult::fail(out.str());
    }
  }

  return ActionResult::ok("config valid");
}

}  // namespace panthera_spectrometer_cell
