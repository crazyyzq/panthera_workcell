#include "panthera_spectrometer_cell/RobotActions.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
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
constexpr char kGripperAction[] = "/gripper_controller/follow_joint_trajectory";
constexpr char kGripperJoint[] = "L_finger_joint";
constexpr char kCupObjectId[] = "carried_cup";
constexpr char kMotionAction[] = "/motion/execute";
constexpr char kArmAction[] = "/arm_controller/follow_joint_trajectory";

using ExecuteMotion = panthera_interfaces::action::ExecuteMotion;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;

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
    "initialize robot actions, simulation=%s backend=%s velocity_scale=%.3f acceleration_scale=%.3f",
    config_.simulation.enabled ? "true" : "false",
    config_.motion.backend.c_str(),
    config_.motion.velocityScale,
    config_.motion.accelerationScale);

  const auto motor_stop = setCleaningMotor(false, "initialize brush motor stopped");
  if (!motor_stop.success) {
    return motor_stop;
  }

  if (usesFixedMotion() || !config_.simulation.enabled) {
    const auto init_result = usesFixedMotion() ?
      initializeFixedMotionInterface() : initializeRealInterfaces();
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
  if (motion_client_) {
    motion_client_->async_cancel_all_goals();
    std::lock_guard<std::mutex> lock(fixed_state_mutex_);
    fixed_point_.clear();
    active_outlet_ = OutletId::NONE;
  }
  if (gripper_client_) {
    gripper_client_->async_cancel_all_goals();
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

ActionResult RobotActions::recoverHomeAfterError()
{
  constexpr double kSettledVelocityRadSec = 0.05;
  const auto & home = config_.motion.homeJointPose;
  const std::vector<std::string> joint_names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  if (config_.simulation.enabled) {
    std::lock_guard<std::mutex> lock(fixed_state_mutex_);
    fixed_point_ = "home_near";
    active_outlet_ = OutletId::NONE;
    return ActionResult::ok("simulated error recovery at home_near");
  }
  if (!arm_recovery_client_) {
    return ActionResult::fail("error recovery unavailable: arm controller client is not initialized");
  }

  const auto motor_stop = setCleaningMotor(false, "stop cleaning motor before error recovery");
  if (!motor_stop.success) {
    RCLCPP_ERROR(logger_, "%s", motor_stop.message.c_str());
  }

  if (motion_client_) {
    const auto cancel_future = motion_client_->async_cancel_all_goals();
    cancel_future.wait_for(std::chrono::duration<double>(config_.motion.cancelWaitSec));
  }

  std::vector<double> current;
  if (!getLatestArmJointValues(current) || current.size() != home.size()) {
    return ActionResult::fail("error recovery refused: fresh six-joint encoder state unavailable");
  }

  auto confirm_home = [this, &home, &joint_names, kSettledVelocityRadSec](
      double & max_error, double & max_velocity) {
      max_error = std::numeric_limits<double>::infinity();
      max_velocity = std::numeric_limits<double>::infinity();
      sensor_msgs::msg::JointState state;
      rclcpp::Time received;
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!has_joint_state_) {
          return false;
        }
        state = latest_joint_state_;
        received = latest_joint_state_received_;
      }
      if ((node_->now() - received).seconds() > 0.5) {
        return false;
      }
      max_error = 0.0;
      for (std::size_t index = 0; index < joint_names.size(); ++index) {
        double position = 0.0;
        if (!lookupJointValue(state, joint_names[index], state.position, position)) {
          return false;
        }
        max_error = std::max(max_error, std::abs(position - home[index]));
      }
      return jointVelocityBelowThreshold(
        state, joint_names, kSettledVelocityRadSec, max_velocity) &&
             max_error <= config_.motion.errorRecoverySettleToleranceRad;
    };

  double max_error = 0.0;
  double max_velocity = 0.0;
  if (confirm_home(max_error, max_velocity)) {
    std::lock_guard<std::mutex> lock(fixed_state_mutex_);
    fixed_point_ = "home_near";
    active_outlet_ = OutletId::NONE;
    return ActionResult::ok("error recovery skipped: encoder already confirms home_near");
  }

  if (!arm_recovery_client_->wait_for_action_server(
      std::chrono::duration<double>(config_.motion.motionServerWaitSec)))
  {
    return ActionResult::fail("error recovery unavailable: arm controller action server missing");
  }

  FollowTrajectory::Goal goal;
  goal.trajectory.header.stamp = node_->now() + rclcpp::Duration::from_seconds(0.10);
  goal.trajectory.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint target;
  target.positions = home;
  target.velocities.assign(home.size(), 0.0);
  target.accelerations.assign(home.size(), 0.0);
  target.time_from_start = secondsToDuration(config_.motion.errorRecoveryDurationSec);
  goal.trajectory.points.push_back(target);

  for (const auto & joint_name : joint_names) {
    control_msgs::msg::JointTolerance path_tolerance;
    path_tolerance.name = joint_name;
    path_tolerance.position = config_.motion.errorRecoveryPathToleranceRad;
    goal.path_tolerance.push_back(path_tolerance);

    control_msgs::msg::JointTolerance goal_tolerance;
    goal_tolerance.name = joint_name;
    goal_tolerance.position = config_.motion.errorRecoveryGoalToleranceRad;
    goal_tolerance.velocity = kSettledVelocityRadSec;
    goal.goal_tolerance.push_back(goal_tolerance);
  }
  goal.goal_time_tolerance = secondsToDuration(config_.motion.errorRecoveryTimeoutMarginSec);

  RCLCPP_ERROR(
    logger_,
    "SAFETY_RECOVERY_START keep_enabled=true target=home_near duration=%.2fs",
    config_.motion.errorRecoveryDurationSec);
  const auto goal_future = arm_recovery_client_->async_send_goal(goal);
  if (goal_future.wait_for(
      std::chrono::duration<double>(config_.motion.motionServerWaitSec)) !=
    std::future_status::ready)
  {
    return ActionResult::fail("error recovery failed: controller goal response timeout; keep enabled");
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return ActionResult::fail("error recovery failed: controller rejected Home goal; keep enabled");
  }

  const auto result_future = arm_recovery_client_->async_get_result(goal_handle);
  const double result_timeout = config_.motion.errorRecoveryDurationSec +
    config_.motion.errorRecoveryTimeoutMarginSec;
  if (result_future.wait_for(std::chrono::duration<double>(result_timeout)) !=
    std::future_status::ready)
  {
    return ActionResult::fail("error recovery failed: controller result timeout; keep enabled");
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
    wrapped.result->error_code != FollowTrajectory::Result::SUCCESSFUL)
  {
    const std::string detail = wrapped.result ? wrapped.result->error_string : "missing result";
    return ActionResult::fail("error recovery controller failure: " + detail + "; keep enabled");
  }

  const auto confirm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  do {
    if (confirm_home(max_error, max_velocity)) {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      fixed_point_ = "home_near";
      active_outlet_ = OutletId::NONE;
      RCLCPP_ERROR(
        logger_, "SAFETY_RECOVERY_CONFIRMED target=home_near max_error=%.6f max_velocity=%.6f",
        max_error, max_velocity);
      return ActionResult::ok("error recovery encoder-confirmed at home_near");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (std::chrono::steady_clock::now() < confirm_deadline);

  std::ostringstream error;
  error << "error recovery final encoder confirmation failed: max_error=" << max_error
        << " max_velocity=" << max_velocity << "; keep enabled";
  return ActionResult::fail(error.str());
}

ActionResult RobotActions::reset()
{
  RCLCPP_INFO(logger_, "robot reset requested");
  if (gripper_client_) {
    gripper_client_->async_cancel_all_goals();
  }
  if (usesFixedMotion() && motion_client_) {
    motion_client_->async_cancel_all_goals();
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      fixed_point_.clear();
      active_outlet_ = OutletId::NONE;
    }
    return ActionResult::ok(
      "fixed motion reset: active goal cancelled; restart after placing arm at configured start point");
  }
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
  const bool cleaning_motor_rs485_changed =
    config_.cleaning.motorRs485Enabled != config.cleaning.motorRs485Enabled ||
    config_.cleaning.motorRs485Device != config.cleaning.motorRs485Device ||
    config_.cleaning.motorRs485Baudrate != config.cleaning.motorRs485Baudrate ||
    config_.cleaning.motorRs485SlaveId != config.cleaning.motorRs485SlaveId;
  const std::string active_backend = config_.motion.backend;
  const std::string active_start_point = config_.motion.fixedStartPoint;
  config_ = config;
  if (config_.motion.backend != active_backend ||
    config_.motion.fixedStartPoint != active_start_point)
  {
    RCLCPP_WARN(
      logger_,
      "motion backend/start point changes require restart; keeping backend=%s start=%s",
      active_backend.c_str(),
      active_start_point.c_str());
    config_.motion.backend = active_backend;
    config_.motion.fixedStartPoint = active_start_point;
  }
  if (cleaning_motor_rs485_changed) {
    std::lock_guard<std::mutex> motor_lock(cleaning_motor_mutex_);
    cleaning_motor_modbus_.reset();
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

  const double applied = std::clamp(scale, 0.10, 1.00);
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

  if (usesFixedMotion()) {
    auto result = openGripper();
    if (!result.success) {
      return result;
    }
    const bool outlet_one = outlet == OutletId::OUTLET_1;
    std::string current_point;
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      current_point = fixed_point_;
    }
    const std::string route_name =
      current_point == "home_near" && outlet_one ?
      "home_to_outlet_1_grasp_smooth" :
      (outlet_one ? "outlet_wait_to_outlet_1_grasp" : "outlet_wait_to_outlet_2_grasp");
    result = executeFixedRoute(
      route_name,
      outlet_one ? "outlet_1_grasp" : "outlet_2_grasp",
      "fixed route pick from outlet");
    if (!result.success) {
      return result;
    }
    result = closeGripper();
    if (!result.success) {
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      active_outlet_ = outlet;
    }
    return attachCup();
  }

  const auto target_pose = config_.findPose(outlet_config->pickPose);
  if (!target_pose) {
    return ActionResult::fail(
      "pickFromOutlet failed: missing pick pose " + outlet_config->pickPose);
  }
  const double overhead_z =
    std::max({
      config_.motion.outletTransferZ,
      config_.motion.outletHighZ,
      config_.motion.outletGripZ + 0.05,
      target_pose->xyz[2] + 0.05,
    });
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

  if (usesFixedMotion()) {
    OutletId active_outlet = OutletId::NONE;
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      active_outlet = active_outlet_;
    }
    if (active_outlet != outlet) {
      return ActionResult::fail(
        "fixed return rejected: active outlet does not match " + toString(outlet));
    }
    const bool outlet_one = outlet == OutletId::OUTLET_1;
    std::string current_point;
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      current_point = fixed_point_;
    }
    ActionResult result = ActionResult::ok("already at outlet return point");
    if (current_point != (outlet_one ? "outlet_1_grasp" : "outlet_2_grasp")) {
      if (current_point == "brush_entry" && outlet_one) {
        result = executeFixedRoute(
          "brush_entry_to_outlet_1_return_continuous",
          "outlet_1_grasp",
          "fixed continuous brush exit and return to outlet");
      } else {
        if (current_point == "brush_entry") {
          result = executeFixedRoute(
            "brush_entry_to_clean_hover", "clean_hover", "fixed leave brush area");
          if (!result.success) {
            return result;
          }
        }
        result = executeFixedRoute(
          outlet_one ? "clean_hover_to_outlet_1_return" : "clean_hover_to_outlet_2_return",
          outlet_one ? "outlet_1_grasp" : "outlet_2_grasp",
          "fixed route return cup to outlet");
      }
      if (!result.success) {
        return result;
      }
    }
    result = openGripper();
    if (!result.success) {
      return result;
    }
    result = detachCup();
    if (!result.success) {
      return result;
    }
    result = executeFixedRoute(
      outlet_one ? "outlet_1_return_to_home_fast" : "outlet_2_return_to_wait",
      outlet_one ? "home_near" : "outlet_wait",
      "fixed route leave returned cup");
    if (result.success) {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      active_outlet_ = OutletId::NONE;
    }
    return result;
  }

  const auto target_pose = config_.findPose(outlet_config->returnPose);
  if (!target_pose) {
    return ActionResult::fail(
      "returnCupToOutlet failed: missing return pose " + outlet_config->returnPose);
  }
  const double overhead_z =
    std::max({
      config_.motion.outletTransferZ,
      config_.motion.outletHighZ,
      config_.motion.outletGripZ + 0.05,
      target_pose->xyz[2] + 0.05,
    });
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

  ActionResult result = moveToPoseJoint(above_return, "moveJ outlet return above target");
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

  return moveToPoseCartesian(above_return, "cartesian lift from outlet return target");
}

ActionResult RobotActions::placeToSpectrometer(double axis_position_mm)
{
  if (usesFixedMotion()) {
    OutletId active_outlet = OutletId::NONE;
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      active_outlet = active_outlet_;
    }
    if (active_outlet == OutletId::NONE) {
      return ActionResult::fail("fixed spectrometer place rejected: no active source outlet");
    }
    const bool outlet_one = active_outlet == OutletId::OUTLET_1;
    auto result = executeFixedRoute(
      outlet_one ? "outlet_1_grasp_to_spectrometer_place_continuous" :
      "outlet_2_grasp_to_spectrometer_place",
      "spectrometer_place",
      "fixed route place to spectrometer");
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
    return executeFixedRoute(
      "spectrometer_place_to_hover",
      "spectrometer_hover",
      "fixed route leave spectrometer after place");
  }

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

  const PoseConfig above_place =
    poseWithNameAndZ(
      target,
      "spectrometer_place_above_target",
      std::max(config_.motion.spectrometerHighZ, target.xyz[2] + 0.05));
  const PoseConfig place =
    poseWithNameAndZ(target, "spectrometer_place_target", target.xyz[2]);
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

  return moveToPoseCartesian(above_place, "cartesian lift from spectrometer place target");
}

ActionResult RobotActions::pickFromSpectrometer(double axis_position_mm)
{
  if (usesFixedMotion()) {
    auto result = executeFixedRoute(
      "spectrometer_hover_to_pick",
      "spectrometer_pick",
      "fixed route pick from spectrometer");
    if (!result.success) {
      return result;
    }
    result = closeGripper();
    if (!result.success) {
      return result;
    }
    return attachCup();
  }

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

  const PoseConfig above_pick =
    poseWithNameAndZ(
      target,
      "spectrometer_pick_above_target",
      std::max(config_.motion.spectrometerHighZ, target.xyz[2] + 0.05));
  const PoseConfig pick =
    poseWithNameAndZ(target, "spectrometer_pick_target", target.xyz[2]);

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

  if (usesFixedMotion()) {
    return executeFixedCleaning();
  }

  const auto dump_pose = config_.findPose(config_.cleaning.dumpPose);
  if (!dump_pose) {
    return ActionResult::fail("clean cup failed: missing clean dump pose");
  }

  const PoseConfig hover =
    poseWithNameAndZ(*dump_pose, "clean_hover", config_.motion.cleanHighZ);
  const PoseConfig dump =
    poseWithNameAndZ(*dump_pose, "clean_dump", dump_pose->xyz[2]);

  ActionResult result = moveToPoseJoint(hover, "moveJ clean hover");
  if (!result.success) {
    return result;
  }

  result = moveToPoseCartesian(dump, "cartesian descend clean dump");
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

  return moveToPoseJoint(hover, "moveJ clean hover after cleaning");
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
          setCleaningMotor(false, "stop cleaning motor after brush clean");
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

  result = setCleaningMotor(true, "start cleaning motor before brush entry");
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
  if (usesFixedMotion()) {
    std::string current_point;
    {
      std::lock_guard<std::mutex> lock(fixed_state_mutex_);
      current_point = fixed_point_;
    }
    if (current_point == "outlet_wait") {
      return ActionResult::ok("already at fixed outlet wait point");
    }
    if (current_point == "home_near") {
      return ActionResult::ok("remain at Home until an outlet task is selected");
    }
    if (current_point != "safe_joint_center") {
      return ActionResult::fail(
        "fixed outlet wait requires known home_near or safe_joint_center start; current=" +
        (current_point.empty() ? std::string("unknown") : current_point));
    }
    return executeFixedRoute(
      "safe_center_to_outlet_wait", "outlet_wait", "fixed startup to outlet wait");
  }

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

bool RobotActions::usesFixedMotion() const
{
  return config_.motion.backend == "fixed_cache";
}

ActionResult RobotActions::initializeFixedMotionInterface()
{
  try {
    motion_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    motion_client_ = rclcpp_action::create_client<ExecuteMotion>(
      node_, kMotionAction, motion_callback_group_);
  } catch (const std::exception & exc) {
    return ActionResult::fail(
      std::string("initialize fixed motion interface failed: ") + exc.what());
  }

  if (!motion_client_->wait_for_action_server(
      std::chrono::duration<double>(config_.motion.motionServerWaitSec)))
  {
    return ActionResult::fail(
      "fixed motion server unavailable at " + std::string(kMotionAction));
  }
  const auto gripper_result = initializeGripperAndJointStateInterfaces();
  if (!gripper_result.success) {
    return gripper_result;
  }

  {
    std::lock_guard<std::mutex> lock(fixed_state_mutex_);
    fixed_point_ = config_.motion.fixedStartPoint;
    active_outlet_ = OutletId::NONE;
  }
  RCLCPP_INFO(
    logger_,
    "fixed motion interface ready action=%s expected_start=%s",
    kMotionAction,
    config_.motion.fixedStartPoint.c_str());
  return ActionResult::ok("fixed motion interface initialized");
}

ActionResult RobotActions::initializeGripperAndJointStateInterfaces()
{
  if (config_.simulation.enabled) {
    return ActionResult::ok("gripper/joint-state interfaces skipped in simulation");
  }

  try {
    arm_recovery_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    arm_recovery_client_ = rclcpp_action::create_client<FollowTrajectory>(
      node_, kArmAction, arm_recovery_callback_group_);
    gripper_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripper_client_ = rclcpp_action::create_client<FollowTrajectory>(
      node_, kGripperAction, gripper_callback_group_);

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
        latest_joint_state_received_ = node_->now();
        has_joint_state_ = true;
      },
      joint_state_options);
  } catch (const std::exception & exc) {
    return ActionResult::fail(
      std::string("initialize gripper/joint-state interfaces failed: ") + exc.what());
  }

  if (!gripper_client_->wait_for_action_server(
      std::chrono::duration<double>(config_.gripper.actionServerWaitSec)))
  {
    return ActionResult::fail(
      "gripper action server unavailable at " + std::string(kGripperAction));
  }
  if (!arm_recovery_client_->wait_for_action_server(
      std::chrono::duration<double>(config_.motion.motionServerWaitSec)))
  {
    return ActionResult::fail(
      "arm recovery action server unavailable at " + std::string(kArmAction));
  }
  return ActionResult::ok("gripper action and joint-state feedback ready");
}

ActionResult RobotActions::executeFixedRoute(
  const std::string & route_name,
  const std::string & expected_end_point,
  const std::string & label)
{
  if (!motion_client_) {
    return ActionResult::fail(label + " failed: fixed motion client is not initialized");
  }
  ExecuteMotion::Goal goal;
  goal.route_name = route_name;
  goal.speed_scale = std::clamp(speed_scale_.load(), 0.10, 1.0);
  goal.dry_run = config_.simulation.enabled;

  std::string logical_start;
  {
    std::lock_guard<std::mutex> lock(fixed_state_mutex_);
    logical_start = fixed_point_;
  }
  RCLCPP_INFO(
    logger_,
    "%s route=%s logical_start=%s speed_scale=%.2f dry_run=%s",
    label.c_str(),
    route_name.c_str(),
    logical_start.empty() ? "unknown" : logical_start.c_str(),
    goal.speed_scale,
    goal.dry_run ? "true" : "false");

  std::string last_error;
  for (int attempt = 0; attempt <= config_.motion.transientRetryCount; ++attempt) {
    if (!motion_client_->wait_for_action_server(
        std::chrono::duration<double>(config_.motion.motionServerWaitSec)))
    {
      last_error = "motion server unavailable";
    } else {
      auto goal_future = motion_client_->async_send_goal(goal);
      if (goal_future.wait_for(
          std::chrono::duration<double>(config_.motion.motionServerWaitSec)) !=
        std::future_status::ready)
      {
        // The future may still be accepted later; do not create a duplicate route goal.
        return ActionResult::fail(label + " failed: action goal response timeout");
      }

      const auto goal_handle = goal_future.get();
      if (!goal_handle) {
        last_error = "route goal rejected before execution: " + route_name;
      } else {
        auto result_future = motion_client_->async_get_result(goal_handle);
        if (result_future.wait_for(
            std::chrono::duration<double>(config_.loop.actionTimeoutSec)) !=
          std::future_status::ready)
        {
          const auto cancel_future = motion_client_->async_cancel_goal(goal_handle);
          cancel_future.wait_for(std::chrono::duration<double>(config_.motion.cancelWaitSec));
          std::lock_guard<std::mutex> lock(fixed_state_mutex_);
          fixed_point_.clear();
          return ActionResult::fail(
            label + " failed: route execution timeout and cancel requested: " + route_name);
        }

        const auto wrapped = result_future.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success)
        {
          std::lock_guard<std::mutex> lock(fixed_state_mutex_);
          fixed_point_ = expected_end_point;
          return ActionResult::ok(
            label + " ok route=" + route_name + " end=" + expected_end_point);
        }

        const std::int32_t error_code = wrapped.result ?
          wrapped.result->error_code : ExecuteMotion::Result::ERROR_CONTROLLER_FAILED;
        last_error = wrapped.result ? wrapped.result->message : "missing action result";
        const bool safe_to_retry =
          error_code == ExecuteMotion::Result::ERROR_BUSY ||
          error_code == ExecuteMotion::Result::ERROR_STATE_UNAVAILABLE ||
          error_code == ExecuteMotion::Result::ERROR_CONTROLLER_UNAVAILABLE ||
          error_code == ExecuteMotion::Result::ERROR_CONTROLLER_REJECTED;
        if (!safe_to_retry) {
          std::lock_guard<std::mutex> lock(fixed_state_mutex_);
          fixed_point_.clear();
          return ActionResult::fail(label + " failed: " + last_error);
        }
      }
    }

    if (attempt < config_.motion.transientRetryCount) {
      RCLCPP_WARN(
        logger_, "%s transient failure attempt %d/%d: %s; retrying before motion",
        label.c_str(), attempt + 1, config_.motion.transientRetryCount + 1,
        last_error.c_str());
      std::this_thread::sleep_for(
        std::chrono::duration<double>(config_.motion.transientRetryDelaySec));
    }
  }
  return ActionResult::fail(label + " failed after transient retries: " + last_error);
}

ActionResult RobotActions::executeFixedCleaning()
{
  if (!config_.cleaning.brushEnabled) {
    auto result = executeFixedRoute(
      "spectrometer_pick_to_clean_dump",
      "clean_dump",
      "fixed route spectrometer to clean dump");
    if (!result.success) {
      return result;
    }
    result = executeFixedRoute(
      "clean_dump_to_pour", "clean_dump_pour", "fixed wrist pour");
    if (!result.success) {
      return result;
    }
    return executeFixedRoute(
      "clean_dump_pour_to_clean_hover", "clean_hover",
      "fixed route leave pour pose");
  }

  auto result = executeFixedRoute(
    "spectrometer_pick_to_brush_entry_continuous",
    "brush_entry",
    "fixed continuous spectrometer lift, pour, shake and brush approach");
  if (!result.success) {
    return result;
  }

  result = setCleaningMotor(true, "start cleaning motor before fixed brush insertion");
  if (!result.success) {
    return result;
  }
  bool motor_running = true;
  const auto stop_motor = [this, &motor_running]() {
      if (!motor_running) {
        return ActionResult::ok("cleaning motor already stopped");
      }
      auto stop_result = setCleaningMotor(false, "stop cleaning motor after fixed brush exit");
      motor_running = false;
      return stop_result;
    };

  result = executeFixedRoute(
    "brush_entry_to_center", "brush_center", "fixed Cartesian brush insertion");
  if (!result.success) {
    stop_motor();
    return result;
  }
  if (config_.cleaning.brushHoldSec > 0.0) {
    std::this_thread::sleep_for(
      std::chrono::duration<double>(effectiveDuration(config_.cleaning.brushHoldSec)));
  }

  result = executeFixedRoute(
    "brush_center_to_entry", "brush_entry", "fixed vertical 50mm brush exit");
  if (!result.success) {
    stop_motor();
    return result;
  }
  if (config_.cleaning.brushMotorStopDelaySec > 0.0) {
    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        effectiveDuration(config_.cleaning.brushMotorStopDelaySec)));
  }
  const auto motor_stop = stop_motor();
  if (!motor_stop.success) {
    return motor_stop;
  }

  return ActionResult::ok("fixed cleaning complete at brush_entry");
}

ActionResult RobotActions::initializeRealInterfaces()
{
  try {
    planning_scene_ =
      std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
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

  } catch (const std::exception & exc) {
    return ActionResult::fail(std::string("initialize real robot interfaces failed: ") + exc.what());
  }

  const auto gripper_result = initializeGripperAndJointStateInterfaces();
  if (!gripper_result.success) {
    return gripper_result;
  }

  RCLCPP_INFO(
    logger_,
    "real robot interfaces initialized arm_group=%s hand_frame=%s gripper_action=%s",
    kArmGroup,
    kHandFrame,
    kGripperAction);
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
  const auto joint_names = arm_ ? armJointNames(*arm_) :
    std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
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
  if (!gripper_client_) {
    return ActionResult::fail(label + " failed: gripper action client is not initialized");
  }

  const double command_position =
    clampPosition(position, config_.gripper.closePosition, config_.gripper.openPosition);
  const double scaled_duration = effectiveDuration(duration_sec);
  const bool allow_grasp_contact =
    command_position <= config_.gripper.closePosition + config_.gripper.positionToleranceM;
  std::string last_error;

  for (int attempt = 0; attempt <= config_.gripper.retryCount; ++attempt) {
    if (!gripper_client_->wait_for_action_server(
        std::chrono::duration<double>(config_.gripper.actionServerWaitSec)))
    {
      last_error = "gripper action server unavailable";
      continue;
    }

    FollowTrajectory::Goal goal;
    goal.trajectory.header.stamp =
      node_->now() + rclcpp::Duration::from_seconds(0.05);
    goal.trajectory.joint_names.push_back(kGripperJoint);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(command_position);
    point.velocities.push_back(0.0);
    point.time_from_start = secondsToDuration(scaled_duration);
    goal.trajectory.points.push_back(point);

    if (allow_grasp_contact) {
      auto hold_point = point;
      hold_point.time_from_start = secondsToDuration(scaled_duration + 3600.0);
      goal.trajectory.points.push_back(hold_point);
    }

    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = kGripperJoint;
    tolerance.position = allow_grasp_contact ?
      config_.gripper.graspHoldGoalToleranceM : config_.gripper.positionToleranceM;
    tolerance.velocity = config_.gripper.settledVelocityToleranceMps;
    goal.goal_tolerance.push_back(tolerance);
    if (allow_grasp_contact) {
      auto path_tolerance = tolerance;
      path_tolerance.position = 0.10;
      goal.path_tolerance.push_back(path_tolerance);
    }
    goal.goal_time_tolerance = secondsToDuration(config_.gripper.commandTimeoutMarginSec);

    const auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(
        std::chrono::duration<double>(config_.gripper.actionServerWaitSec)) !=
      std::future_status::ready)
    {
      return ActionResult::fail(label + " failed: gripper goal response timeout");
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      last_error = "gripper controller rejected goal";
      continue;
    }

    const auto result_future = gripper_client_->async_get_result(goal_handle);
    bool grasp_contact = false;
    std::string target_error;
    if (waitForGripperTarget(
        command_position,
        std::chrono::duration<double>(scaled_duration + config_.gripper.settleTimeoutSec),
        allow_grasp_contact,
        grasp_contact,
        target_error))
    {
      if (grasp_contact) {
        // The cup intentionally prevents the commanded zero position. Keep the accepted
        // trajectory alive: its final target remains zero, so ros2_control continues applying
        // closing effort after contact until an explicit open command replaces it.
        RCLCPP_INFO(
          logger_, "%s stable grasp contact detected; continuous close target retained",
          label.c_str());
        return ActionResult::ok(label + " ok (stable grasp contact)");
      }
      return ActionResult::ok(label + " ok (encoder target reached)");
    }

    if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto wrapped = result_future.get();
      last_error = wrapped.result ? wrapped.result->error_string : "missing gripper result";
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
        wrapped.result->error_code == FollowTrajectory::Result::SUCCESSFUL)
      {
        last_error = "controller succeeded but encoder verification failed";
      }
    } else {
      const auto cancel_future = gripper_client_->async_cancel_goal(goal_handle);
      cancel_future.wait_for(
        std::chrono::duration<double>(config_.gripper.commandTimeoutMarginSec));
      last_error = "gripper result timeout";
    }
    last_error += "; " + target_error;
    if (attempt < config_.gripper.retryCount) {
      RCLCPP_WARN(
        logger_, "%s attempt %d/%d failed: %s; retrying idempotent gripper target",
        label.c_str(), attempt + 1, config_.gripper.retryCount + 1, last_error.c_str());
    }
  }

  return ActionResult::fail(label + " failed: " + last_error);
}

bool RobotActions::waitForGripperTarget(
  double target_position,
  std::chrono::duration<double> timeout,
  bool allow_grasp_contact,
  bool & grasp_contact,
  std::string & error) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::optional<std::chrono::steady_clock::time_point> contact_since;
  grasp_contact = false;
  do {
    sensor_msgs::msg::JointState state;
    rclcpp::Time received;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (has_joint_state_) {
        state = latest_joint_state_;
        received = latest_joint_state_received_;
      }
    }
    if (state.name.empty()) {
      error = "no joint state received for gripper verification";
    } else if ((node_->now() - received).seconds() > 0.5) {
      error = "joint state is stale during gripper verification";
    } else {
      double current_position = 0.0;
      double current_velocity = std::numeric_limits<double>::infinity();
      const bool has_position = lookupJointValue(
        state, kGripperJoint, state.position, current_position);
      const bool has_velocity = lookupJointValue(
        state, kGripperJoint, state.velocity, current_velocity);
      if (!has_position || !has_velocity) {
        error = "joint state is missing gripper position or velocity";
      } else {
        const double position_error = std::abs(current_position - target_position);
        if (position_error <= config_.gripper.positionToleranceM &&
          std::abs(current_velocity) <= config_.gripper.settledVelocityToleranceMps)
        {
          return true;
        }
        const bool contact_candidate =
          allow_grasp_contact &&
          current_position <= config_.gripper.openPosition -
          config_.gripper.graspContactMinClosureM &&
          current_position > target_position + config_.gripper.positionToleranceM &&
          std::abs(current_velocity) <= config_.gripper.settledVelocityToleranceMps;
        if (contact_candidate) {
          const auto now = std::chrono::steady_clock::now();
          if (!contact_since) {
            contact_since = now;
          } else if (std::chrono::duration<double>(now - *contact_since).count() >=
            config_.gripper.graspContactConfirmSec)
          {
            grasp_contact = true;
            return true;
          }
        } else {
          contact_since.reset();
        }
        std::ostringstream out;
        out << "gripper target not reached: position_error=" << position_error
            << "m velocity=" << current_velocity << "m/s";
        error = out.str();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
  return false;
}

ActionResult RobotActions::setCleaningMotor(bool enabled, const std::string & label)
{
  constexpr uint16_t kSpeedRegister = 0x0040;
  constexpr uint16_t kControlModeRegister = 0x0080;
  constexpr uint16_t kCommunicationTimeoutRegister = 0x008e;

  if (config_.simulation.enabled) {
    return ActionResult::ok(label + " skipped in simulation");
  }
  if (!config_.cleaning.motorRs485Enabled) {
    return ActionResult::ok(label + " skipped: cleaning motor RS485 disabled");
  }
  if (config_.cleaning.motorRs485Device.empty()) {
    return ActionResult::fail(label + " failed: cleaning.motor_rs485_device is empty");
  }

  std::lock_guard<std::mutex> lock(cleaning_motor_mutex_);
  std::string last_error;
  for (int attempt = 1; attempt <= 2; ++attempt) {
    try {
      if (!cleaning_motor_modbus_) {
        cleaning_motor_modbus_ = std::make_unique<panthera_rs485::ModbusRtuMaster>(
          config_.cleaning.motorRs485Device,
          config_.cleaning.motorRs485Baudrate,
          std::chrono::milliseconds(250),
          panthera_rs485::SerialParity::EVEN,
          1);
        RCLCPP_INFO(
          logger_,
          "cleaning motor Modbus opened device=%s baud=%d mode=8E1 slave=0x%02X",
          config_.cleaning.motorRs485Device.c_str(),
          config_.cleaning.motorRs485Baudrate,
          config_.cleaning.motorRs485SlaveId);
      }

      if (enabled) {
        cleaning_motor_modbus_->writeSingleRegister(
          config_.cleaning.motorRs485SlaveId, kControlModeRegister, 0);
        cleaning_motor_modbus_->writeSingleRegister(
          config_.cleaning.motorRs485SlaveId,
          kCommunicationTimeoutRegister,
          static_cast<uint16_t>(config_.cleaning.motorRs485CommunicationTimeoutDs));
      }
      const int command = enabled ? config_.cleaning.motorRs485DutyPermille : 0;
      cleaning_motor_modbus_->writeSingleRegister(
        config_.cleaning.motorRs485SlaveId,
        kSpeedRegister,
        static_cast<uint16_t>(static_cast<int16_t>(command)));
      RCLCPP_INFO(
        logger_,
        "%s Modbus speed=%d (%.1f%%) device=%s slave=0x%02X",
        label.c_str(),
        command,
        static_cast<double>(command) * 0.1,
        config_.cleaning.motorRs485Device.c_str(),
        config_.cleaning.motorRs485SlaveId);
      return ActionResult::ok(label + " ok");
    } catch (const std::exception & exc) {
      last_error = exc.what();
      cleaning_motor_modbus_.reset();
      if (attempt == 1) {
        RCLCPP_WARN(logger_, "%s failed once, reconnecting: %s", label.c_str(), exc.what());
      }
    }
  }
  return ActionResult::fail(label + " failed after reconnect: " + last_error);
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

  if (!config_.simulation.enabled && planning_scene_ && !objects.empty()) {
    planning_scene_->applyCollisionObjects(objects);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  return ActionResult::ok("collision objects applied");
}

ActionResult RobotActions::attachCup()
{
  if (config_.simulation.enabled || !planning_scene_) {
    return ActionResult::ok("simulated cup attach");
  }
  if (config_.simulation.enabled) {
    return simulateDelay("attach cup");
  }
  RCLCPP_INFO(
    logger_,
    "cup collision attach disabled: keeping real gripper action only for full-flow tuning");
  planning_scene_->removeCollisionObjects({kCupObjectId});
  return ActionResult::ok("cup collision attach disabled");
}

ActionResult RobotActions::detachCup()
{
  if (config_.simulation.enabled || !planning_scene_) {
    return ActionResult::ok("simulated cup detach");
  }
  if (config_.simulation.enabled) {
    return simulateDelay("detach cup");
  }
  RCLCPP_INFO(
    logger_,
    "cup collision detach disabled: removing any stale carried cup object only");
  planning_scene_->removeCollisionObjects({kCupObjectId});
  return ActionResult::ok("cup collision detach disabled");
}

}  // namespace panthera_spectrometer_cell
