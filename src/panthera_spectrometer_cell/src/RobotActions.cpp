#include "panthera_spectrometer_cell/RobotActions.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace panthera_spectrometer_cell
{
namespace
{

constexpr char kBaseFrame[] = "base_link";
constexpr char kArmGroup[] = "arm";
constexpr char kHandFrame[] = "gripper_center";
constexpr char kGripperTopic[] = "/gripper_controller/joint_trajectory";
constexpr char kGripperJoint[] = "L_finger_joint";
constexpr char kCupObjectId[] = "carried_cup";

double clampPosition(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    seconds = 0.0;
  }

  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>(
    std::round((seconds - static_cast<double>(duration.sec)) * 1e9));

  if (duration.nanosec >= 1000000000u) {
    duration.sec += 1;
    duration.nanosec -= 1000000000u;
  }

  return duration;
}

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

geometry_msgs::msg::Pose poseFromConfig(const PoseConfig & pose)
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.xyz[0];
  out.position.y = pose.xyz[1];
  out.position.z = pose.xyz[2];
  out.orientation = quaternionFromRpy(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
  return out;
}

PoseConfig poseWithNameAndZ(
  const PoseConfig & source,
  const std::string & name,
  double z)
{
  PoseConfig pose = source;
  pose.name = name;
  pose.xyz[2] = z;
  return pose;
}

PoseConfig poseWithNameXyz(
  const PoseConfig & source,
  const std::string & name,
  double x,
  double y,
  double z)
{
  PoseConfig pose = source;
  pose.name = name;
  pose.xyz = {x, y, z};
  return pose;
}

PoseConfig poseWithOffset(
  const PoseConfig & source,
  const std::string & name,
  const Vec3 & offset,
  double direction)
{
  PoseConfig pose = source;
  pose.name = name;
  pose.xyz[0] += direction * offset[0];
  pose.xyz[1] += direction * offset[1];
  pose.xyz[2] += direction * offset[2];
  return pose;
}

Vec3 rotateLocalOffsetToBase(const Vec3 & local_offset, const Vec3 & rpy)
{
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);

  return Vec3{
    (cy * cp) * local_offset[0] +
      (cy * sp * sr - sy * cr) * local_offset[1] +
      (cy * sp * cr + sy * sr) * local_offset[2],
    (sy * cp) * local_offset[0] +
      (sy * sp * sr + cy * cr) * local_offset[1] +
      (sy * sp * cr - cy * sr) * local_offset[2],
    (-sp) * local_offset[0] +
      (cp * sr) * local_offset[1] +
      (cp * cr) * local_offset[2]};
}

PoseConfig poseWithLocalOffset(
  const PoseConfig & source,
  const std::string & name,
  const Vec3 & local_offset,
  double direction)
{
  return poseWithOffset(
    source,
    name,
    rotateLocalOffsetToBase(local_offset, source.rpy),
    direction);
}

std::string poseToString(const PoseConfig & pose)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << pose.name << " xyz=["
      << pose.xyz[0] << ", " << pose.xyz[1] << ", " << pose.xyz[2]
      << "] rpy=["
      << pose.rpy[0] << ", " << pose.rpy[1] << ", " << pose.rpy[2]
      << "]";
  return out.str();
}

std::vector<std::string> armJointNames(
  const moveit::planning_interface::MoveGroupInterface & arm)
{
  auto names = arm.getJointNames();
  if (!names.empty()) {
    return names;
  }

  const auto model = arm.getRobotModel();
  if (!model) {
    return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  }

  const auto * group = model->getJointModelGroup(kArmGroup);
  if (!group) {
    return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  }

  names = group->getVariableNames();
  if (!names.empty()) {
    return names;
  }

  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

bool jointVelocityBelowThreshold(
  const sensor_msgs::msg::JointState & joint_state,
  const std::vector<std::string> & joint_names,
  double threshold,
  double & max_abs_velocity)
{
  max_abs_velocity = 0.0;
  if (joint_names.empty() || joint_state.velocity.empty()) {
    return false;
  }

  bool saw_any_arm_joint = false;
  for (const auto & joint_name : joint_names) {
    const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    if (it == joint_state.name.end()) {
      continue;
    }

    const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), it));
    if (index >= joint_state.velocity.size()) {
      continue;
    }

    saw_any_arm_joint = true;
    max_abs_velocity = std::max(max_abs_velocity, std::abs(joint_state.velocity[index]));
  }

  return saw_any_arm_joint && max_abs_velocity <= threshold;
}

bool lookupJointValue(
  const sensor_msgs::msg::JointState & joint_state,
  const std::string & joint_name,
  const std::vector<double> & values,
  double & out)
{
  const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
  if (it == joint_state.name.end()) {
    return false;
  }

  const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), it));
  if (index >= values.size()) {
    return false;
  }

  out = values[index];
  return std::isfinite(out);
}

bool isPlausibleArmJointState(
  const sensor_msgs::msg::JointState & joint_state,
  const sensor_msgs::msg::JointState * previous)
{
  const std::vector<std::string> arm_joint_names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  constexpr double kMaxAbsPositionRad = 3.30;
  constexpr double kMaxAbsVelocityRadSec = 4.00;
  constexpr double kMaxSingleSampleJumpRad = 0.60;

  for (const auto & joint_name : arm_joint_names) {
    double position = 0.0;
    if (!lookupJointValue(joint_state, joint_name, joint_state.position, position)) {
      return false;
    }
    if (std::abs(position) > kMaxAbsPositionRad) {
      return false;
    }

    if (!joint_state.velocity.empty()) {
      double velocity = 0.0;
      if (lookupJointValue(joint_state, joint_name, joint_state.velocity, velocity) &&
        std::abs(velocity) > kMaxAbsVelocityRadSec)
      {
        return false;
      }
    }

    if (previous && !previous->name.empty()) {
      double previous_position = 0.0;
      if (lookupJointValue(*previous, joint_name, previous->position, previous_position) &&
        std::abs(position - previous_position) > kMaxSingleSampleJumpRad)
      {
        return false;
      }
    }
  }

  return true;
}

}  // namespace

RobotActions::RobotActions(rclcpp::Node::SharedPtr node, WorkcellConfig config)
: node_(std::move(node)),
  logger_(node_->get_logger().get_child("robot_actions")),
  config_(std::move(config)),
  rng_(config_.simulation.randomSeed + 11)
{
}

ActionResult RobotActions::initialize()
{
  RCLCPP_INFO(
    logger_,
    "initialize robot actions, simulation=%s velocity_scale=%.3f acceleration_scale=%.3f",
    config_.simulation.enabled ? "true" : "false",
    config_.motion.velocityScale,
    config_.motion.accelerationScale);

  if (!config_.simulation.enabled) {
    const auto init_result = initializeRealInterfaces();
    if (!init_result.success) {
      return init_result;
    }
  }

  const auto collision_result = applyCollisionObjects();
  if (!collision_result.success) {
    return collision_result;
  }

  return ActionResult::ok("robot actions initialized");
}

ActionResult RobotActions::stop()
{
  RCLCPP_WARN(logger_, "robot stop requested");
  const auto motor_stop = setCleaningMotor(false, "stop cleaning motor on robot stop");
  if (!motor_stop.success) {
    RCLCPP_ERROR(logger_, "%s", motor_stop.message.c_str());
  }
  if (!config_.simulation.enabled && arm_) {
    try {
      arm_->stop();
      arm_->clearPoseTargets();
    } catch (const std::exception & exc) {
      return ActionResult::fail(std::string("robot stop failed: ") + exc.what());
    }
  }
  return ActionResult::ok("robot stopped");
}

ActionResult RobotActions::reset()
{
  RCLCPP_INFO(logger_, "robot reset requested");
  if (!config_.simulation.enabled && arm_) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    arm_->stop();
    arm_->clearPoseTargets();
    return ActionResult::ok("robot reset ok: stopped active motion");
  }
  return simulateDelay("robot reset");
}

void RobotActions::updateConfig(const WorkcellConfig & config)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  const bool cleaning_motor_serial_changed =
    config_.cleaning.motorSerialEnabled != config.cleaning.motorSerialEnabled ||
    config_.cleaning.motorSerialDevice != config.cleaning.motorSerialDevice ||
    config_.cleaning.motorSerialBaudrate != config.cleaning.motorSerialBaudrate;
  config_ = config;
  if (cleaning_motor_serial_changed) {
    std::lock_guard<std::mutex> motor_lock(cleaning_motor_mutex_);
    if (cleaning_motor_serial_) {
      cleaning_motor_serial_->close();
    }
  }
  if (!config_.simulation.enabled && arm_) {
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));
    arm_->setPlanningTime(config_.motion.planningTimeSec);
    arm_->setNumPlanningAttempts(config_.motion.planningAttempts);
    arm_->setGoalPositionTolerance(config_.motion.goalPositionTolerance);
    arm_->setGoalOrientationTolerance(config_.motion.goalOrientationTolerance);
    arm_->setGoalJointTolerance(config_.motion.goalJointTolerance);
  }
  RCLCPP_WARN(logger_, "robot action config reloaded");
}

ActionResult RobotActions::setSpeedScale(double scale)
{
  if (!std::isfinite(scale)) {
    return ActionResult::fail("setSpeedScale failed: scale is not finite");
  }

  const double applied = std::clamp(scale, 0.20, 1.20);
  speed_scale_.store(applied);
  if (!config_.simulation.enabled && arm_) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));
  }

  std::ostringstream out;
  out << "speed scale set to " << std::fixed << std::setprecision(2) << applied;
  RCLCPP_WARN(logger_, "%s", out.str().c_str());
  return ActionResult::ok(out.str());
}

double RobotActions::speedScale() const
{
  return speed_scale_.load();
}

ActionResult RobotActions::pickFromOutlet(OutletId outlet)
{
  const auto outlet_config = config_.findOutlet(outlet);
  if (!outlet_config) {
    return ActionResult::fail("pickFromOutlet failed: unknown outlet " + toString(outlet));
  }

  RCLCPP_INFO(logger_, "pick cup from %s", outlet_config->name.c_str());

  const auto target_pose = config_.findPose(outlet_config->pickPose);
  if (!target_pose) {
    return ActionResult::fail(
      "pickFromOutlet failed: missing pick pose " + outlet_config->pickPose);
  }
  const auto configured_approach_pose = config_.findPose(outlet_config->pickApproachPose);
  if (!configured_approach_pose) {
    return ActionResult::fail(
      "pickFromOutlet failed: missing pick approach pose " + outlet_config->pickApproachPose);
  }

  const double overhead_z =
    std::max({
      config_.motion.outletTransferZ,
      config_.motion.outletHighZ,
      config_.motion.outletGripZ + 0.05,
      target_pose->xyz[2] + 0.05,
    });
  const PoseConfig configured_approach_high =
    poseWithNameAndZ(
      *configured_approach_pose,
      outlet_config->name + "_pick_configured_approach_high",
      overhead_z);
  const PoseConfig above_grasp =
    poseWithNameXyz(
      *target_pose,
      outlet_config->name + "_pick_above_grasp",
      target_pose->xyz[0],
      target_pose->xyz[1],
      overhead_z);
  const PoseConfig grasp =
    poseWithNameXyz(
      *target_pose,
      outlet_config->name + "_pick_grasp",
      target_pose->xyz[0],
      target_pose->xyz[1],
      config_.motion.outletGripZ);
  const PoseConfig lift =
    poseWithNameXyz(
      *target_pose,
      outlet_config->name + "_pick_lift",
      target_pose->xyz[0],
      target_pose->xyz[1],
      overhead_z);

  ActionResult result = openGripper();
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(configured_approach_high, "moveJ outlet pick configured approach high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(above_grasp, "moveJ outlet pick above grasp");
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(grasp, "cartesian descend outlet pick grasp");
  if (!result.success) {
    return result;
  }

  result = closeGripper();
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(lift, "cartesian lift cup from outlet");
  if (!result.success) {
    return result;
  }

  return attachCup();
}

ActionResult RobotActions::returnCupToOutlet(OutletId outlet)
{
  const auto outlet_config = config_.findOutlet(outlet);
  if (!outlet_config) {
    return ActionResult::fail("returnCupToOutlet failed: unknown outlet " + toString(outlet));
  }

  RCLCPP_INFO(logger_, "return cup to original outlet %s", outlet_config->name.c_str());

  const auto target_pose = config_.findPose(outlet_config->returnPose);
  if (!target_pose) {
    return ActionResult::fail(
      "returnCupToOutlet failed: missing return pose " + outlet_config->returnPose);
  }
  const auto configured_approach_pose = config_.findPose(outlet_config->returnApproachPose);
  if (!configured_approach_pose) {
    return ActionResult::fail(
      "returnCupToOutlet failed: missing return approach pose " + outlet_config->returnApproachPose);
  }

  const double overhead_z =
    std::max({
      config_.motion.outletTransferZ,
      config_.motion.outletHighZ,
      config_.motion.outletGripZ + 0.05,
      target_pose->xyz[2] + 0.05,
    });
  const PoseConfig configured_approach_high =
    poseWithNameAndZ(
      *configured_approach_pose,
      outlet_config->name + "_return_configured_approach_high",
      overhead_z);
  const PoseConfig above_return =
    poseWithNameXyz(
      *target_pose,
      outlet_config->name + "_return_above_target",
      target_pose->xyz[0],
      target_pose->xyz[1],
      overhead_z);
  const PoseConfig return_pose =
    poseWithNameXyz(
      *target_pose,
      outlet_config->name + "_return_target",
      target_pose->xyz[0],
      target_pose->xyz[1],
      config_.motion.outletGripZ);

  ActionResult result =
    moveToPoseJoint(configured_approach_high, "moveJ outlet return configured approach high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(above_return, "moveJ outlet return above target");
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(return_pose, "cartesian descend outlet return target");
  if (!result.success) {
    return result;
  }

  result = openGripper();
  if (!result.success) {
    return result;
  }

  result = detachCup();
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(above_return, "cartesian lift from outlet return target");
  if (!result.success) {
    return result;
  }

  return moveToPoseJoint(configured_approach_high, "moveJ outlet return configured leave high");
}

ActionResult RobotActions::placeToSpectrometer(double axis_position_mm)
{
  PoseConfig target;
  auto result = computeSpectrometerTarget(
    axis_position_mm,
    config_.spectrometerAxis.placeOffsetXyz,
    "spectrometer_place_target",
    target);
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(
    logger_,
    "place cup to spectrometer using laser %.3f mm -> %s",
    axis_position_mm,
    poseToString(target).c_str());

  const PoseConfig approach_high =
    poseWithNameXyz(
      target,
      "spectrometer_place_approach_high",
      target.xyz[0] + config_.motion.spectrometerApproachXOffset,
      target.xyz[1],
      config_.motion.spectrometerHighZ);
  const PoseConfig above_place =
    poseWithNameAndZ(
      target,
      "spectrometer_place_above_target",
      std::max(config_.motion.spectrometerHighZ, target.xyz[2] + 0.05));
  const PoseConfig place =
    poseWithNameAndZ(target, "spectrometer_place_target", target.xyz[2]);
  const PoseConfig wait_pose =
    poseWithNameXyz(
      target,
      "spectrometer_wait",
      target.xyz[0] + config_.motion.spectrometerApproachXOffset,
      target.xyz[1],
      config_.motion.spectrometerHighZ);

  result = moveToPoseJoint(approach_high, "moveJ spectrometer place approach high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(above_place, "moveJ spectrometer place above target");
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(place, "cartesian descend spectrometer place target");
  if (!result.success) {
    return result;
  }

  result = openGripper();
  if (!result.success) {
    return result;
  }

  result = detachCup();
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(above_place, "cartesian lift from spectrometer place target");
  if (!result.success) {
    return result;
  }

  return moveToPoseJoint(wait_pose, "moveJ spectrometer wait after place");
}

ActionResult RobotActions::pickFromSpectrometer(double axis_position_mm)
{
  PoseConfig target;
  auto result = computeSpectrometerTarget(
    axis_position_mm,
    config_.spectrometerAxis.pickOffsetXyz,
    "spectrometer_pick_target",
    target);
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(
    logger_,
    "pick cup from spectrometer using laser %.3f mm -> %s",
    axis_position_mm,
    poseToString(target).c_str());

  const PoseConfig approach_high =
    poseWithNameXyz(
      target,
      "spectrometer_pick_approach_high",
      target.xyz[0] + config_.motion.spectrometerApproachXOffset,
      target.xyz[1],
      config_.motion.spectrometerHighZ);
  const PoseConfig above_pick =
    poseWithNameAndZ(
      target,
      "spectrometer_pick_above_target",
      std::max(config_.motion.spectrometerHighZ, target.xyz[2] + 0.05));
  const PoseConfig pick =
    poseWithNameAndZ(target, "spectrometer_pick_target", target.xyz[2]);

  result = moveToPoseJoint(approach_high, "moveJ spectrometer pick approach high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(above_pick, "moveJ spectrometer pick above target");
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(pick, "cartesian descend spectrometer pick target");
  if (!result.success) {
    return result;
  }

  result = closeGripper();
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(above_pick, "cartesian lift cup from spectrometer");
  if (!result.success) {
    return result;
  }

  return attachCup();
}

ActionResult RobotActions::cleanCup()
{
  RCLCPP_INFO(logger_, "clean cup");

  const auto approach_pose = config_.findPose(config_.cleaning.approachPose);
  const auto ready_pose = config_.findPose(config_.cleaning.dumpPose);
  const auto leave_pose = config_.findPose(config_.cleaning.leavePose);
  if (!approach_pose || !ready_pose || !leave_pose) {
    return ActionResult::fail("clean cup failed: missing clean approach/dump/leave pose");
  }

  const PoseConfig approach_high =
    poseWithNameXyz(
      *ready_pose,
      "clean_approach_high",
      ready_pose->xyz[0],
      config_.motion.cleanApproachY,
      config_.motion.cleanHighZ);
  const PoseConfig pre_clean =
    poseWithNameXyz(
      *ready_pose,
      "clean_pre",
      ready_pose->xyz[0],
      config_.motion.cleanApproachY,
      config_.motion.cleanPreZ);
  const PoseConfig ready =
    poseWithNameAndZ(*ready_pose, "clean_ready_high", config_.motion.cleanReadyZ);
  const PoseConfig dump =
    poseWithNameAndZ(*ready_pose, "clean_dump", ready_pose->xyz[2]);
  const PoseConfig retreat =
    poseWithNameXyz(
      *ready_pose,
      "clean_retreat_pre",
      ready_pose->xyz[0],
      config_.motion.cleanApproachY,
      config_.motion.cleanPreZ);
  const PoseConfig retreat_high =
    poseWithNameXyz(
      *ready_pose,
      "clean_retreat_high",
      ready_pose->xyz[0],
      config_.motion.cleanApproachY,
      config_.motion.cleanHighZ);

  ActionResult result = moveToPoseJoint(*approach_pose, "moveJ clean configured approach");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(approach_high, "moveJ clean approach high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(pre_clean, "moveJ clean pre");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(ready, "moveJ clean ready high");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(dump, "moveJ clean dump");
  if (!result.success) {
    return result;
  }

  result = closeGripper();
  if (!result.success) {
    return result;
  }

  result = executeWristPourSequence();
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(ready, "moveJ clean back to ready");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(retreat, "moveJ clean retreat pre");
  if (!result.success) {
    return result;
  }

  result = moveToPoseJoint(retreat_high, "moveJ clean retreat high");
  if (!result.success) {
    return result;
  }

  return moveToPoseJoint(*leave_pose, "moveJ clean configured leave");
}

ActionResult RobotActions::brushCleanCup()
{
  if (!config_.cleaning.brushEnabled) {
    return ActionResult::ok("brush clean disabled");
  }

  const auto brush_pose = config_.findPose(config_.cleaning.brushPose);
  if (!brush_pose) {
    return ActionResult::fail(
      "brush clean failed: missing brush pose '" + config_.cleaning.brushPose + "'");
  }

  RCLCPP_INFO(
    logger_,
    "brush clean pose=%s brush_speed=[%.3f, %.3f] hold=%.3fs motor_stop_delay=%.3fs tool_approach_offset=[%.3f, %.3f, %.3f] tool_upright_retreat_offset=[%.3f, %.3f, %.3f] strokes=%d tool_stroke_offset=[%.3f, %.3f, %.3f]",
    config_.cleaning.brushPose.c_str(),
    config_.cleaning.brushVelocityScale,
    config_.cleaning.brushAccelerationScale,
    config_.cleaning.brushHoldSec,
    config_.cleaning.brushMotorStopDelaySec,
    config_.cleaning.brushApproachOffsetXyz[0],
    config_.cleaning.brushApproachOffsetXyz[1],
    config_.cleaning.brushApproachOffsetXyz[2],
    config_.cleaning.brushUprightRetreatOffsetXyz[0],
    config_.cleaning.brushUprightRetreatOffsetXyz[1],
    config_.cleaning.brushUprightRetreatOffsetXyz[2],
    config_.cleaning.brushStrokeCount,
    config_.cleaning.brushStrokeOffsetXyz[0],
    config_.cleaning.brushStrokeOffsetXyz[1],
    config_.cleaning.brushStrokeOffsetXyz[2]);

  const PoseConfig brush_approach =
    poseWithLocalOffset(
      *brush_pose,
      "brush_clean_approach",
      config_.cleaning.brushApproachOffsetXyz,
      1.0);
  const PoseConfig brush_upright_retreat =
    poseWithLocalOffset(
      *brush_pose,
      "brush_clean_upright_retreat",
      config_.cleaning.brushUprightRetreatOffsetXyz,
      1.0);
  const bool needs_upright_retreat =
    std::abs(
      config_.cleaning.brushUprightRetreatOffsetXyz[0] -
      config_.cleaning.brushApproachOffsetXyz[0]) > 1e-6 ||
    std::abs(
      config_.cleaning.brushUprightRetreatOffsetXyz[1] -
      config_.cleaning.brushApproachOffsetXyz[1]) > 1e-6 ||
    std::abs(
      config_.cleaning.brushUprightRetreatOffsetXyz[2] -
      config_.cleaning.brushApproachOffsetXyz[2]) > 1e-6;

  const double saved_velocity_scale = config_.motion.velocityScale;
  const double saved_acceleration_scale = config_.motion.accelerationScale;
  config_.motion.velocityScale = config_.cleaning.brushVelocityScale;
  config_.motion.accelerationScale = config_.cleaning.brushAccelerationScale;

  bool cleaning_motor_running = false;
  auto finish_with_cleanup =
    [this, &cleaning_motor_running, saved_velocity_scale, saved_acceleration_scale](
      ActionResult result)
    {
      ActionResult final_result = result;
      if (cleaning_motor_running) {
        const auto stop_result =
          setCleaningMotor(false, "send cleaning motor stop ascii '0' after brush clean");
        cleaning_motor_running = false;
        if (!stop_result.success) {
          if (!final_result.success) {
            final_result =
              ActionResult::fail(final_result.message + "; additionally " + stop_result.message);
          } else {
            final_result = stop_result;
          }
        }
      }
      config_.motion.velocityScale = saved_velocity_scale;
      config_.motion.accelerationScale = saved_acceleration_scale;
      return final_result;
    };

  // Always enter the brush lane from the far outside point.  A direct joint-space
  // move to the near approach point can swing the cup through the brush centre
  // before the commanded Cartesian insertion starts.
  ActionResult result = moveToPoseJoint(
    brush_upright_retreat,
    "moveJ brush clean pre-entry far outside");
  if (!result.success) {
    return finish_with_cleanup(result);
  }

  if (needs_upright_retreat) {
    result = moveToPoseCartesian(
      brush_approach,
      "cartesian approach from far outside to brush entry");
    if (!result.success) {
      return finish_with_cleanup(result);
    }
  }

  result = setCleaningMotor(true, "send cleaning motor start ascii '1' before brush entry");
  if (!result.success) {
    return finish_with_cleanup(result);
  }
  cleaning_motor_running = true;

  result = moveToPoseCartesian(*brush_pose, "cartesian translate into rotating brush");
  if (!result.success) {
    return finish_with_cleanup(result);
  }

  const PoseConfig stroke_negative =
    poseWithLocalOffset(
      *brush_pose,
      "brush_clean_negative",
      config_.cleaning.brushStrokeOffsetXyz,
      -1.0);
  const PoseConfig stroke_positive =
    poseWithLocalOffset(
      *brush_pose,
      "brush_clean_positive",
      config_.cleaning.brushStrokeOffsetXyz,
      1.0);
  const auto hold_duration =
    std::chrono::duration<double>(effectiveDuration(config_.cleaning.brushHoldSec));
  const bool has_stroke_motion =
    std::abs(config_.cleaning.brushStrokeOffsetXyz[0]) > 1e-6 ||
    std::abs(config_.cleaning.brushStrokeOffsetXyz[1]) > 1e-6 ||
    std::abs(config_.cleaning.brushStrokeOffsetXyz[2]) > 1e-6;

  if (!has_stroke_motion) {
    if (hold_duration.count() > 0.0) {
      std::this_thread::sleep_for(hold_duration);
    }
  } else {
    for (int stroke = 0; stroke < config_.cleaning.brushStrokeCount; ++stroke) {
      result = moveToPoseCartesian(stroke_negative, "cartesian brush clean negative stroke");
      if (!result.success) {
        return finish_with_cleanup(result);
      }
      if (hold_duration.count() > 0.0) {
        std::this_thread::sleep_for(hold_duration);
      }

      result = moveToPoseCartesian(stroke_positive, "cartesian brush clean positive stroke");
      if (!result.success) {
        return finish_with_cleanup(result);
      }
      if (hold_duration.count() > 0.0) {
        std::this_thread::sleep_for(hold_duration);
      }
    }

    result = moveToPoseCartesian(*brush_pose, "cartesian brush clean return center");
    if (!result.success) {
      return finish_with_cleanup(result);
    }
  }

  result = moveToPoseCartesian(
    brush_approach,
    "cartesian exit brush centre to near outside");
  if (!result.success) {
    return finish_with_cleanup(result);
  }

  if (needs_upright_retreat) {
    result = moveToPoseCartesian(
      brush_upright_retreat,
      "cartesian retract near outside to far outside before wrist upright");
    if (!result.success) {
      return finish_with_cleanup(result);
    }
  }

  if (config_.cleaning.brushMotorStopDelaySec > 0.0) {
    RCLCPP_INFO(
      logger_,
      "brush is fully out; keep cleaning motor running %.3fs before wrist upright",
      config_.cleaning.brushMotorStopDelaySec);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(
        static_cast<int>(effectiveDuration(config_.cleaning.brushMotorStopDelaySec) * 1000.0)));
  }

  RCLCPP_INFO(logger_, "brush exit complete; wrist upright is allowed now");
  return finish_with_cleanup(result);
}

ActionResult RobotActions::moveToOutletWait()
{
  const auto wait_pose = config_.findPose("outlet_wait_mid");
  if (!wait_pose) {
    return ActionResult::ok("outlet_wait_mid not configured; skip wait pose move");
  }
  return moveToPoseJoint(*wait_pose, "moveJ outlet wait mid");
}

ActionResult RobotActions::moveToNamedPose(const std::string & pose_name, const std::string & label)
{
  const auto pose = config_.findPose(pose_name);
  if (!pose) {
    return ActionResult::fail(label + " failed: missing pose '" + pose_name + "'");
  }
  return moveToPose(*pose, label);
}

ActionResult RobotActions::moveToPose(const PoseConfig & pose, const std::string & label)
{
  return moveToPoseJoint(pose, label);
}

ActionResult RobotActions::moveToPoseJoint(const PoseConfig & pose, const std::string & label)
{
  auto result = maybeSimulatedFailure(label);
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(logger_, "%s -> %s", label.c_str(), poseToString(pose).c_str());
  if (config_.simulation.enabled) {
    return simulateDelay(label);
  }

  return executePoseJoint(pose, label);
}

ActionResult RobotActions::moveToPoseCartesian(const PoseConfig & pose, const std::string & label)
{
  auto result = maybeSimulatedFailure(label);
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(logger_, "%s cartesian -> %s", label.c_str(), poseToString(pose).c_str());
  if (config_.simulation.enabled) {
    return simulateDelay(label);
  }

  return executePoseCartesian(pose, label);
}

ActionResult RobotActions::moveToPoseCartesianWithJointFallback(
  const PoseConfig & pose,
  const std::string & label)
{
  // Debug-compatible mode:
  // The pose_tuner targets were validated one-by-one using normal MoveIt pose planning:
  // setPoseTarget -> plan -> execute.
  //
  // During full-flow commissioning, do not use computeCartesianPath first.
  // This makes integrated workflow behavior consistent with successful point tuning.
  return moveToPoseJoint(pose, label + " debug-compatible moveJ");
}

ActionResult RobotActions::moveToSafePose(const std::string & label)
{
  auto result = maybeSimulatedFailure(label);
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(logger_, "%s -> configured safe_joint_pose", label.c_str());
  if (config_.simulation.enabled) {
    return simulateDelay(label);
  }

  return executeJointTarget(config_.motion.safeJointPose, label);
}

ActionResult RobotActions::initializeRealInterfaces()
{
  try {
    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, kArmGroup);
    arm_->setEndEffectorLink(kHandFrame);
    arm_->setPoseReferenceFrame(kBaseFrame);
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));
    arm_->setPlanningTime(config_.motion.planningTimeSec);
    arm_->setNumPlanningAttempts(config_.motion.planningAttempts);
    arm_->setGoalPositionTolerance(config_.motion.goalPositionTolerance);
    arm_->setGoalOrientationTolerance(config_.motion.goalOrientationTolerance);
    arm_->setGoalJointTolerance(config_.motion.goalJointTolerance);

    gripper_pub_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(kGripperTopic, 10);

    joint_state_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions joint_state_options;
    joint_state_options.callback_group = joint_state_callback_group_;
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!isPlausibleArmJointState(*msg, has_joint_state_ ? &latest_joint_state_ : nullptr)) {
          return;
        }
        latest_joint_state_ = *msg;
        has_joint_state_ = true;
      },
      joint_state_options);
  } catch (const std::exception & exc) {
    return ActionResult::fail(std::string("initialize real robot interfaces failed: ") + exc.what());
  }

  RCLCPP_INFO(
    logger_,
    "real robot interfaces initialized arm_group=%s hand_frame=%s gripper_topic=%s",
    kArmGroup,
    kHandFrame,
    kGripperTopic);
  return ActionResult::ok("real robot interfaces initialized");
}

ActionResult RobotActions::executePoseJoint(const PoseConfig & pose, const std::string & label)
{
  if (!arm_) {
    return ActionResult::fail(label + " failed: MoveGroupInterface is not initialized");
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = kBaseFrame;
  target.header.stamp = node_->now();
  target.pose = poseFromConfig(pose);

  std::string last_error;
  constexpr int kMaxAttempts = 2;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    waitForArmStateSettled(label);
    setStartStateFromLatestArmJointState(label);
    arm_->setPoseReferenceFrame(kBaseFrame);
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));
    arm_->setPoseTarget(target, kHandFrame);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_->plan(plan);
    arm_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      last_error = label + " failed: MoveIt plan failed";
      if (attempt < kMaxAttempts) {
        RCLCPP_WARN(
          logger_,
          "%s; retrying after state refresh (%d/%d)",
          last_error.c_str(),
          attempt,
          kMaxAttempts);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        continue;
      }
      return ActionResult::fail(last_error);
    }

    const auto exec_result = arm_->execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      waitForArmStateSettled(label);
      return ActionResult::ok(label + " ok");
    }

    last_error = label + " failed: MoveIt execute failed";
    if (attempt < kMaxAttempts) {
      RCLCPP_WARN(
        logger_,
        "%s; retrying after state refresh (%d/%d)",
        last_error.c_str(),
        attempt,
        kMaxAttempts);
      arm_->stop();
      arm_->clearPoseTargets();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  return ActionResult::fail(last_error);
}

ActionResult RobotActions::executePoseCartesian(const PoseConfig & pose, const std::string & label)
{
  if (!arm_) {
    return ActionResult::fail(label + " failed: MoveGroupInterface is not initialized");
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  geometry_msgs::msg::Pose target = poseFromConfig(pose);

  std::string last_error;
  constexpr int kMaxAttempts = 2;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    waitForArmStateSettled(label);
    setStartStateFromLatestArmJointState(label);
    arm_->setPoseReferenceFrame(kBaseFrame);
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);

    const double fraction = arm_->computeCartesianPath(
      waypoints,
      config_.motion.cartesianEefStep,
      config_.motion.cartesianJumpThreshold,
      trajectory,
      true);

    if (fraction < config_.motion.cartesianMinFraction) {
      std::ostringstream out;
      out << label << " failed: Cartesian fraction " << std::fixed << std::setprecision(3)
          << fraction << " < " << config_.motion.cartesianMinFraction;
      last_error = out.str();
      if (attempt < kMaxAttempts) {
        RCLCPP_WARN(
          logger_,
          "%s; retrying after state refresh (%d/%d)",
          last_error.c_str(),
          attempt,
          kMaxAttempts);
        arm_->stop();
        arm_->clearPoseTargets();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        continue;
      }
      return ActionResult::fail(last_error);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto exec_result = arm_->execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      waitForArmStateSettled(label);
      return ActionResult::ok(label + " ok");
    }

    last_error = label + " failed: MoveIt cartesian execute failed";
    if (attempt < kMaxAttempts) {
      RCLCPP_WARN(
        logger_,
        "%s; retrying after state refresh (%d/%d)",
        last_error.c_str(),
        attempt,
        kMaxAttempts);
      arm_->stop();
      arm_->clearPoseTargets();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  return ActionResult::fail(last_error);
}

ActionResult RobotActions::executeJointTarget(
  const std::vector<double> & positions,
  const std::string & label)
{
  if (!arm_) {
    return ActionResult::fail(label + " failed: MoveGroupInterface is not initialized");
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  const auto joint_names = armJointNames(*arm_);
  if (positions.size() != joint_names.size()) {
    std::ostringstream out;
    out << label << " failed: joint target has " << positions.size()
        << " values, arm group requires " << joint_names.size();
    return ActionResult::fail(out.str());
  }

  std::string last_error;
  constexpr int kMaxAttempts = 2;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    waitForArmStateSettled(label);
    setStartStateFromLatestArmJointState(label);
    if (!arm_->setJointValueTarget(joint_names, positions)) {
      return ActionResult::fail(label + " failed: setJointValueTarget failed");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      last_error = label + " failed: MoveIt joint plan failed";
      if (attempt < kMaxAttempts) {
        RCLCPP_WARN(
          logger_,
          "%s; retrying after state refresh (%d/%d)",
          last_error.c_str(),
          attempt,
          kMaxAttempts);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        continue;
      }
      return ActionResult::fail(last_error);
    }

    const auto exec_result = arm_->execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      waitForArmStateSettled(label);
      return ActionResult::ok(label + " ok");
    }

    last_error = label + " failed: MoveIt joint execute failed";
    if (attempt < kMaxAttempts) {
      RCLCPP_WARN(
        logger_,
        "%s; retrying after state refresh (%d/%d)",
        last_error.c_str(),
        attempt,
        kMaxAttempts);
      arm_->stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  return ActionResult::fail(last_error);
}

void RobotActions::waitForArmStateSettled(const std::string & label) const
{
  if (config_.simulation.enabled || !arm_) {
    return;
  }

  const auto joint_names = armJointNames(*arm_);
  constexpr double kSettledVelocityRadSec = 0.035;
  constexpr int kRequiredStableSamples = 4;
  const auto sample_period = std::chrono::milliseconds(50);
  const auto timeout = std::chrono::milliseconds(1200);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  int stable_samples = 0;
  bool saw_joint_state = false;
  double last_max_abs_velocity = std::numeric_limits<double>::infinity();
  while (std::chrono::steady_clock::now() < deadline) {
    sensor_msgs::msg::JointState joint_state;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (has_joint_state_) {
        joint_state = latest_joint_state_;
      }
    }

    if (!joint_state.name.empty()) {
      saw_joint_state = true;
      double max_abs_velocity = 0.0;
      if (jointVelocityBelowThreshold(
            joint_state, joint_names, kSettledVelocityRadSec, max_abs_velocity)) {
        last_max_abs_velocity = max_abs_velocity;
        ++stable_samples;
        if (stable_samples >= kRequiredStableSamples) {
          return;
        }
      } else {
        last_max_abs_velocity = max_abs_velocity;
        stable_samples = 0;
      }
    }

    std::this_thread::sleep_for(sample_period);
  }

  if (!saw_joint_state) {
    RCLCPP_WARN(
      logger_,
      "%s: no joint state observed while waiting for arm settle; continuing",
      label.c_str());
    return;
  }

  RCLCPP_WARN(
    logger_,
    "%s: arm did not fully settle before timeout; last max arm velocity %.4f rad/s, continuing",
    label.c_str(),
    last_max_abs_velocity);
}

bool RobotActions::setStartStateFromLatestArmJointState(const std::string & label)
{
  if (!arm_) {
    return false;
  }

  std::vector<double> positions;
  if (!getLatestArmJointValues(positions)) {
    RCLCPP_WARN(
      logger_,
      "%s: latest /joint_states unavailable; falling back to MoveIt current state",
      label.c_str());
    arm_->setStartStateToCurrentState();
    return false;
  }

  const auto joint_names = armJointNames(*arm_);
  if (positions.size() != joint_names.size()) {
    RCLCPP_WARN(
      logger_,
      "%s: latest /joint_states arm joint count %zu != MoveIt arm joint count %zu; falling back",
      label.c_str(),
      positions.size(),
      joint_names.size());
    arm_->setStartStateToCurrentState();
    return false;
  }

  const auto model = arm_->getRobotModel();
  if (!model) {
    RCLCPP_WARN(
      logger_,
      "%s: robot model unavailable; falling back to MoveIt current state",
      label.c_str());
    arm_->setStartStateToCurrentState();
    return false;
  }

  moveit::core::RobotState start_state(model);
  start_state.setToDefaultValues();
  start_state.setVariablePositions(joint_names, positions);
  start_state.update();
  arm_->setStartState(start_state);
  RCLCPP_DEBUG(logger_, "%s: MoveIt start state synchronized from /joint_states", label.c_str());
  return true;
}

ActionResult RobotActions::executeWristPourSequence()
{
  if (config_.simulation.enabled) {
    auto result = simulateDelay("wrist pour sequence");
    if (!result.success || !config_.cleaning.brushEnabled) {
      return result;
    }
    result = brushCleanCup();
    if (!result.success) {
      return result;
    }
    return simulateDelay("return cup upright after brush clean");
  }
  if (!arm_) {
    return ActionResult::fail("wrist pour sequence failed: MoveGroupInterface is not initialized");
  }

  std::vector<double> upright_joints;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    upright_joints = arm_->getCurrentJointValues();
    if (upright_joints.size() < 6) {
      const auto state = arm_->getCurrentState(2.0);
      if (state) {
        state->copyJointGroupPositions(kArmGroup, upright_joints);
      }
    }
    if (upright_joints.size() < 6) {
      getLatestArmJointValues(upright_joints);
    }
  }
  if (upright_joints.size() < 6) {
    std::ostringstream out;
    out << "wrist pour sequence failed: arm group has " << upright_joints.size()
        << " joints, expected at least 6";
    return ActionResult::fail(out.str());
  }

  const std::size_t wrist_index =
    std::min<std::size_t>(
      static_cast<std::size_t>(config_.cleaning.pourWristJointIndex),
      upright_joints.size() - 1);
  const auto joint_names = armJointNames(*arm_);
  const auto model = arm_->getRobotModel();
  double lower_limit = -std::numeric_limits<double>::infinity();
  double upper_limit = std::numeric_limits<double>::infinity();
  bool bounded = false;
  if (model && wrist_index < joint_names.size()) {
    const auto & bounds = model->getVariableBounds(joint_names[wrist_index]);
    bounded = bounds.position_bounded_;
    if (bounded) {
      lower_limit = bounds.min_position_;
      upper_limit = bounds.max_position_;
    }
  }

  const double plus_requested = upright_joints[wrist_index] + config_.cleaning.pourAngleRad;
  const double minus_requested = upright_joints[wrist_index] - config_.cleaning.pourAngleRad;
  const double plus_target = bounded ? clampPosition(plus_requested, lower_limit, upper_limit) : plus_requested;
  const double minus_target = bounded ? clampPosition(minus_requested, lower_limit, upper_limit) : minus_requested;
  const double plus_travel = std::abs(plus_target - upright_joints[wrist_index]);
  const double minus_travel = std::abs(minus_target - upright_joints[wrist_index]);

  double direction = 0.0;
  if (config_.cleaning.pourDirection > 0) {
    direction = 1.0;
  } else if (config_.cleaning.pourDirection < 0) {
    direction = -1.0;
  } else {
    direction = plus_travel >= minus_travel ? 1.0 : -1.0;
  }

  const double selected_travel = direction > 0.0 ? plus_travel : minus_travel;
  if (selected_travel < 0.05) {
    std::ostringstream out;
    out << "wrist pour sequence failed: joint index " << wrist_index
        << " has no usable travel in configured direction " << direction
        << " near current position " << upright_joints[wrist_index];
    return ActionResult::fail(out.str());
  }

  const double pour_target = direction > 0.0 ? plus_target : minus_target;
  RCLCPP_INFO(
    logger_,
    "wrist pour direction=%+.0f mode=%s joint=%zu current=%.3f target=%.3f",
    direction,
    config_.cleaning.pourDirection == 0 ? "auto" : "configured",
    wrist_index,
    upright_joints[wrist_index],
    pour_target);
  const bool clamped_to_limit =
    bounded && std::abs((direction > 0.0 ? plus_requested : minus_requested) - pour_target) > 1e-6;
  if (clamped_to_limit) {
    RCLCPP_WARN(
      logger_,
      "wrist pour requested %.3f rad from joint %zu but hits limit; using limit target %.3f rad",
      config_.cleaning.pourAngleRad,
      wrist_index,
      pour_target);
  }

  const auto restore_scales = [this]() {
      if (arm_) {
        arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.motion.velocityScale));
        arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.motion.accelerationScale));
      }
    };

  arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.cleaning.pourVelocityScale));
  arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.cleaning.pourAccelerationScale));

  std::vector<double> pour_joints = upright_joints;
  pour_joints[wrist_index] = pour_target;

  ActionResult result = executeJointTarget(pour_joints, "pour cup by wrist 180deg");
  if (!result.success) {
    restore_scales();
    return result;
  }
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<int>(effectiveDuration(config_.cleaning.pourHoldSec) * 1000.0)));

  for (int i = 0; i < config_.cleaning.shakeCount; ++i) {
    std::vector<double> left = pour_joints;
    std::vector<double> right = pour_joints;
    if (clamped_to_limit) {
      left[wrist_index] = pour_joints[wrist_index] - direction * config_.cleaning.shakeAngleRad;
      right[wrist_index] = pour_joints[wrist_index];
    } else {
      left[wrist_index] = pour_joints[wrist_index] - direction * config_.cleaning.shakeAngleRad;
      right[wrist_index] = pour_joints[wrist_index] + direction * config_.cleaning.shakeAngleRad;
    }
    if (bounded) {
      left[wrist_index] = clampPosition(left[wrist_index], lower_limit, upper_limit);
      right[wrist_index] = clampPosition(right[wrist_index], lower_limit, upper_limit);
    }

    std::ostringstream label;
    label << "shake cup " << (i + 1) << "/" << config_.cleaning.shakeCount << " left";
    result = executeJointTarget(left, label.str());
    if (!result.success) {
      restore_scales();
      return result;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(effectiveDuration(config_.cleaning.shakeHoldSec) * 1000.0)));

    label.str("");
    label.clear();
    label << "shake cup " << (i + 1) << "/" << config_.cleaning.shakeCount << " right";
    result = executeJointTarget(right, label.str());
    if (!result.success) {
      restore_scales();
      return result;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(effectiveDuration(config_.cleaning.shakeHoldSec) * 1000.0)));
  }

  if (config_.cleaning.brushEnabled) {
    restore_scales();
    RCLCPP_INFO(
      logger_,
      "brush enabled: skip pour-center stop after shake; align outside brush directly");

    result = brushCleanCup();
    if (!result.success) {
      return result;
    }

    std::vector<double> brush_joints;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      brush_joints = arm_->getCurrentJointValues();
      if (brush_joints.size() < upright_joints.size()) {
        const auto state = arm_->getCurrentState(2.0);
        if (state) {
          state->copyJointGroupPositions(kArmGroup, brush_joints);
        }
      }
      if (brush_joints.size() < upright_joints.size()) {
        getLatestArmJointValues(brush_joints);
      }
    }
    if (brush_joints.size() <= wrist_index) {
      std::ostringstream out;
      out << "return cup upright after brush failed: arm group has " << brush_joints.size()
          << " joints, expected wrist index " << wrist_index;
      return ActionResult::fail(out.str());
    }

    brush_joints[wrist_index] = upright_joints[wrist_index];
    arm_->setMaxVelocityScalingFactor(effectiveVelocityScale(config_.cleaning.pourVelocityScale));
    arm_->setMaxAccelerationScalingFactor(effectiveVelocityScale(config_.cleaning.pourAccelerationScale));
    result = executeJointTarget(brush_joints, "return cup upright after brush clean");
    restore_scales();
    return result;
  }

  result = executeJointTarget(pour_joints, "return to pour center after shake");
  if (!result.success) {
    restore_scales();
    return result;
  }

  result = executeJointTarget(upright_joints, "return cup upright after pour");
  restore_scales();
  return result;
}

bool RobotActions::isJointWithinLimit(std::size_t joint_index, double position) const
{
  if (!arm_) {
    return false;
  }

  const auto joint_names = armJointNames(*arm_);
  if (joint_index >= joint_names.size()) {
    return false;
  }

  const auto model = arm_->getRobotModel();
  if (!model) {
    return true;
  }

  const auto & bounds = model->getVariableBounds(joint_names[joint_index]);
  if (!bounds.position_bounded_) {
    return true;
  }
  return position >= bounds.min_position_ && position <= bounds.max_position_;
}

bool RobotActions::getLatestArmJointValues(std::vector<double> & positions) const
{
  if (!arm_) {
    return false;
  }

  const auto joint_names = armJointNames(*arm_);
  if (joint_names.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!has_joint_state_) {
    return false;
  }

  std::vector<double> ordered;
  ordered.reserve(joint_names.size());
  for (const auto & joint_name : joint_names) {
    const auto it = std::find(
      latest_joint_state_.name.begin(),
      latest_joint_state_.name.end(),
      joint_name);
    if (it == latest_joint_state_.name.end()) {
      return false;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(latest_joint_state_.name.begin(), it));
    if (index >= latest_joint_state_.position.size()) {
      return false;
    }
    ordered.push_back(latest_joint_state_.position[index]);
  }

  positions = ordered;
  return true;
}

ActionResult RobotActions::sendGripperTo(
  double position,
  double duration_sec,
  const std::string & label)
{
  if (config_.simulation.enabled) {
    return simulateDelay(label);
  }
  if (!gripper_pub_) {
    return ActionResult::fail(label + " failed: gripper publisher is not initialized");
  }

  const double command_position =
    clampPosition(position, config_.gripper.closePosition, config_.gripper.openPosition);

  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = node_->now();
  traj.joint_names.push_back(kGripperJoint);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(command_position);
  point.velocities.push_back(0.0);
  const double scaled_duration = effectiveDuration(duration_sec);
  point.time_from_start = secondsToDuration(scaled_duration);
  traj.points.push_back(point);

  gripper_pub_->publish(traj);
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<int>((scaled_duration + 0.2) * 1000.0)));
  return ActionResult::ok(label + " ok");
}

ActionResult RobotActions::setCleaningMotor(bool enabled, const std::string & label)
{
  if (config_.simulation.enabled) {
    return ActionResult::ok(label + " skipped in simulation");
  }
  if (!config_.cleaning.motorSerialEnabled) {
    return ActionResult::ok(label + " skipped: cleaning motor serial disabled");
  }
  if (config_.cleaning.motorSerialDevice.empty()) {
    return ActionResult::fail(label + " failed: cleaning.motor_serial_device is empty");
  }

  const uint8_t command_byte = static_cast<uint8_t>(
    enabled ? config_.cleaning.motorStartByte : config_.cleaning.motorStopByte);

  std::lock_guard<std::mutex> lock(cleaning_motor_mutex_);
  try {
    if (!cleaning_motor_serial_) {
      cleaning_motor_serial_ = std::make_unique<panthera_rs485::SerialPort>();
    }
    if (!cleaning_motor_serial_->isOpen()) {
      cleaning_motor_serial_->open(
        config_.cleaning.motorSerialDevice,
        config_.cleaning.motorSerialBaudrate,
        std::chrono::milliseconds(100));
      RCLCPP_WARN(
        logger_,
        "cleaning motor serial opened device=%s baud=%d mode=8N1 raw/no-newline",
        config_.cleaning.motorSerialDevice.c_str(),
        config_.cleaning.motorSerialBaudrate);
    }

    cleaning_motor_serial_->writeAll({command_byte});
    const char command_char =
      command_byte >= 32 && command_byte <= 126 ? static_cast<char>(command_byte) : '?';
    RCLCPP_WARN(
      logger_,
      "%s sent byte=0x%02X ascii='%c' no_newline device=%s",
      label.c_str(),
      static_cast<unsigned int>(command_byte),
      command_char,
      config_.cleaning.motorSerialDevice.c_str());
  } catch (const std::exception & exc) {
    if (cleaning_motor_serial_) {
      cleaning_motor_serial_->close();
    }
    return ActionResult::fail(label + " failed: " + exc.what());
  }

  return ActionResult::ok(label + " ok");
}

double RobotActions::effectiveVelocityScale(double base_scale) const
{
  return std::clamp(base_scale * speed_scale_.load(), 0.01, 1.0);
}

double RobotActions::effectiveDuration(double duration_sec) const
{
  const double scale = std::clamp(speed_scale_.load(), 0.20, 1.20);
  return std::max(0.05, duration_sec / scale);
}

ActionResult RobotActions::openGripper()
{
  auto result = maybeSimulatedFailure("open gripper");
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(
    logger_,
    "open gripper position=%.4f duration=%.2fs",
    config_.gripper.openPosition,
    config_.gripper.openDurationSec);
  return sendGripperTo(
    config_.gripper.openPosition,
    config_.gripper.openDurationSec,
    "open gripper");
}

ActionResult RobotActions::closeGripper()
{
  auto result = maybeSimulatedFailure("close gripper");
  if (!result.success) {
    return result;
  }

  RCLCPP_INFO(
    logger_,
    "close gripper position=%.4f duration=%.2fs",
    config_.gripper.closePosition,
    config_.gripper.closeDurationSec);
  return sendGripperTo(
    config_.gripper.closePosition,
    config_.gripper.closeDurationSec,
    "close gripper");
}

ActionResult RobotActions::simulateDelay(const std::string & label)
{
  if (config_.simulation.actionDelayMs > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.simulation.actionDelayMs));
  }
  return ActionResult::ok(label + " ok");
}

ActionResult RobotActions::maybeSimulatedFailure(const std::string & action_name)
{
  if (!config_.simulation.enabled || config_.simulation.randomFailureRate <= 0.0) {
    return ActionResult::ok();
  }

  if (unit_dist_(rng_) < config_.simulation.randomFailureRate) {
    return ActionResult::fail(action_name + " failed: simulated random failure");
  }

  return ActionResult::ok();
}

ActionResult RobotActions::computeSpectrometerTarget(
  double axis_position_mm,
  const Vec3 & offset_xyz,
  const std::string & target_name,
  PoseConfig & target) const
{
  const auto & axis = config_.spectrometerAxis;
  if (axis_position_mm < axis.laserMinMm || axis_position_mm > axis.laserMaxMm) {
    std::ostringstream out;
    out << "spectrometer target failed: laser position " << axis_position_mm
        << " mm outside range [" << axis.laserMinMm << ", " << axis.laserMaxMm << "] mm";
    return ActionResult::fail(out.str());
  }

  const auto base = config_.findPose(axis.basePoseName);
  if (!base) {
    return ActionResult::fail("spectrometer target failed: missing base pose " + axis.basePoseName);
  }

  target = *base;
  target.name = target_name;
  target.xyz[0] += offset_xyz[0];
  target.xyz[1] += offset_xyz[1];
  target.xyz[2] += offset_xyz[2];

  const double delta_m = (axis_position_mm - axis.axisZeroLaserMm) * axis.axisScaleMPerMm;
  if (axis.axis == "x") {
    target.xyz[0] += delta_m;
  } else if (axis.axis == "y") {
    target.xyz[1] += delta_m;
  } else if (axis.axis == "z") {
    target.xyz[2] += delta_m;
  } else {
    return ActionResult::fail("spectrometer target failed: invalid axis " + axis.axis);
  }

  return ActionResult::ok("spectrometer target computed");
}

ActionResult RobotActions::applyCollisionObjects()
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  for (const auto & object : config_.collisionObjects) {
    if (!object.enabled) {
      continue;
    }
    RCLCPP_INFO(
      logger_,
      "load collision object %s type=%s frame=%s",
      object.name.c_str(),
      object.type.c_str(),
      object.frameId.c_str());

    if (config_.simulation.enabled) {
      continue;
    }

    moveit_msgs::msg::CollisionObject collision;
    collision.header.frame_id = object.frameId.empty() ? kBaseFrame : object.frameId;
    collision.id = object.name;
    collision.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    if (object.type == "cylinder") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = object.height;
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = object.radius;
    } else if (object.type == "sphere") {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = object.radius;
    } else {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = object.sizeXyz[0];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = object.sizeXyz[1];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = object.sizeXyz[2];
    }

    geometry_msgs::msg::Pose primitive_pose;
    primitive_pose.position.x = object.xyz[0];
    primitive_pose.position.y = object.xyz[1];
    primitive_pose.position.z = object.xyz[2];
    primitive_pose.orientation = quaternionFromRpy(object.rpy[0], object.rpy[1], object.rpy[2]);

    collision.primitives.push_back(primitive);
    collision.primitive_poses.push_back(primitive_pose);
    objects.push_back(collision);
  }

  if (!config_.simulation.enabled && !objects.empty()) {
    planning_scene_.applyCollisionObjects(objects);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  return ActionResult::ok("collision objects applied");
}

ActionResult RobotActions::attachCup()
{
  if (config_.simulation.enabled) {
    return simulateDelay("attach cup");
  }
  RCLCPP_INFO(
    logger_,
    "cup collision attach disabled: keeping real gripper action only for full-flow tuning");
  planning_scene_.removeCollisionObjects({kCupObjectId});
  return ActionResult::ok("cup collision attach disabled");
}

ActionResult RobotActions::detachCup()
{
  if (config_.simulation.enabled) {
    return simulateDelay("detach cup");
  }
  RCLCPP_INFO(
    logger_,
    "cup collision detach disabled: removing any stale carried cup object only");
  planning_scene_.removeCollisionObjects({kCupObjectId});
  return ActionResult::ok("cup collision detach disabled");
}

}  // namespace panthera_spectrometer_cell
