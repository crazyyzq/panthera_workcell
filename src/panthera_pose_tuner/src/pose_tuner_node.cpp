#include "panthera_pose_tuner/TuningTargets.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <panthera_interfaces/srv/run_workflow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using RunTarget = panthera_interfaces::srv::RunWorkflow;
using Trigger = std_srvs::srv::Trigger;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

double clampScale(double value)
{
  if (!std::isfinite(value)) {
    return 0.08;
  }
  return std::clamp(value, 0.01, 1.0);
}

geometry_msgs::msg::Quaternion quaternionFromRpyDeg(
  const panthera_pose_tuner::RpyDeg & rpy_deg)
{
  const double roll = degToRad(rpy_deg.roll);
  const double pitch = degToRad(rpy_deg.pitch);
  const double yaw = degToRad(rpy_deg.yaw);

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

std::string describeTarget(const panthera_pose_tuner::TuningTarget & target)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(2);
  out << target.name << " | ";
  if (target.kind == panthera_pose_tuner::TargetKind::Pose) {
    out << "pose xyz_mm=[" << target.xyz_mm.x << ", " << target.xyz_mm.y << ", "
        << target.xyz_mm.z << "] rpy_deg=[" << target.rpy_deg.roll << ", "
        << target.rpy_deg.pitch << ", " << target.rpy_deg.yaw << "]";
  } else {
    out << "joint rad=[";
    for (std::size_t i = 0; i < target.joints_rad.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << target.joints_rad[i];
    }
    out << "]";
  }
  out << " | " << target.description;
  return out.str();
}

template<typename T>
T getOrDeclareParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    rclcpp::Parameter parameter;
    if (node->get_parameter(name, parameter)) {
      return parameter.get_value<T>();
    }
  }
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace

class PantheraPoseTuner
{
public:
  explicit PantheraPoseTuner(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    logger_(node->get_logger())
  {
    arm_group_ = getOrDeclareParameter<std::string>(node_, "arm_group", "arm");
    default_base_frame_ = getOrDeclareParameter<std::string>(node_, "base_frame", "base_link");
    default_hand_frame_ = getOrDeclareParameter<std::string>(node_, "hand_frame", "gripper_center");
    execute_motion_ = getOrDeclareParameter<bool>(node_, "execute_motion", true);

    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
    arm_->setPoseReferenceFrame(default_base_frame_);
    arm_->setEndEffectorLink(default_hand_frame_);

    loadTargets();

    run_target_srv_ = node_->create_service<RunTarget>(
      "pose_tuner/run_target",
      std::bind(
        &PantheraPoseTuner::runTargetCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    list_targets_srv_ = node_->create_service<Trigger>(
      "pose_tuner/list_targets",
      std::bind(
        &PantheraPoseTuner::listTargetsCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_srv_ = node_->create_service<Trigger>(
      "pose_tuner/stop",
      std::bind(
        &PantheraPoseTuner::stopCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(logger_, "panthera_pose_tuner ready");
    RCLCPP_INFO(logger_, "services: /pose_tuner/run_target, /pose_tuner/list_targets, /pose_tuner/stop");
    RCLCPP_INFO(logger_, "execute_motion parameter: %s", execute_motion_ ? "true" : "false");
    logAvailableTargets();
  }

private:
  void loadTargets()
  {
    const auto targets = panthera_pose_tuner::buildTuningTargets();
    if (targets.empty()) {
      throw std::runtime_error("buildTuningTargets() returned no targets");
    }

    for (const auto & target : targets) {
      if (target.name.empty()) {
        throw std::runtime_error("tuning target name must not be empty");
      }
      const auto inserted = targets_.emplace(target.name, target);
      if (!inserted.second) {
        throw std::runtime_error("duplicate tuning target name: " + target.name);
      }
    }
  }

  void logAvailableTargets() const
  {
    for (const auto & item : targets_) {
      RCLCPP_INFO(logger_, "target: %s", describeTarget(item.second).c_str());
    }
  }

  void listTargetsCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::ostringstream out;
    for (const auto & item : targets_) {
      out << describeTarget(item.second) << "\n";
    }
    response->success = true;
    response->message = out.str();
  }

  void stopCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    arm_->stop();
    response->success = true;
    response->message = "MoveGroup stop() requested";
  }

  void runTargetCallback(
    const std::shared_ptr<RunTarget::Request> request,
    std::shared_ptr<RunTarget::Response> response)
  {
    const auto target_it = targets_.find(request->workflow_name);
    if (target_it == targets_.end()) {
      response->success = false;
      response->message = "unknown target: " + request->workflow_name + ". Call /pose_tuner/list_targets first.";
      return;
    }

    const bool dry_run = request->dry_run || !execute_motion_ || !target_it->second.allow_execute;

    std::lock_guard<std::mutex> lock(move_mutex_);
    const auto result = runTarget(target_it->second, dry_run);
    response->success = result.first;
    response->message = result.second;
  }

  std::pair<bool, std::string> runTarget(
    const panthera_pose_tuner::TuningTarget & target,
    bool dry_run)
  {
    arm_->setMaxVelocityScalingFactor(clampScale(target.velocity_scale));
    arm_->setMaxAccelerationScalingFactor(clampScale(target.acceleration_scale));
    arm_->setPlanningTime(std::max(0.5, target.planning_time_sec));
    arm_->setNumPlanningAttempts(std::max(1, target.planning_attempts));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (target.kind == panthera_pose_tuner::TargetKind::Joint) {
      std::vector<double> joints(target.joints_rad.begin(), target.joints_rad.end());
      arm_->setJointValueTarget(joints);
    } else {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = target.base_frame.empty() ? default_base_frame_ : target.base_frame;
      pose.header.stamp = node_->now();
      pose.pose.position.x = target.xyz_mm.x / 1000.0;
      pose.pose.position.y = target.xyz_mm.y / 1000.0;
      pose.pose.position.z = target.xyz_mm.z / 1000.0;
      pose.pose.orientation = quaternionFromRpyDeg(target.rpy_deg);

      arm_->setPoseReferenceFrame(pose.header.frame_id);
      arm_->setEndEffectorLink(target.hand_frame.empty() ? default_hand_frame_ : target.hand_frame);
      arm_->setGoalPositionTolerance(std::max(0.0005, target.position_tolerance_mm / 1000.0));
      arm_->setGoalOrientationTolerance(std::max(degToRad(0.2), degToRad(target.orientation_tolerance_deg)));
      arm_->setPoseTarget(pose, target.hand_frame.empty() ? default_hand_frame_ : target.hand_frame);
    }

    RCLCPP_INFO(
      logger_,
      "planning target '%s' dry_run=%s",
      target.name.c_str(),
      dry_run ? "true" : "false");

    const auto plan_result = arm_->plan(plan);
    arm_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      return {false, "planning failed for target: " + target.name};
    }

    if (dry_run) {
      return {true, "plan ok, not executed: " + target.name};
    }

    const auto execute_result = arm_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      return {false, "execution failed for target: " + target.name};
    }

    return {true, "executed target: " + target.name};
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string arm_group_;
  std::string default_base_frame_;
  std::string default_hand_frame_;
  bool execute_motion_{true};

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::map<std::string, panthera_pose_tuner::TuningTarget> targets_;
  std::mutex move_mutex_;

  rclcpp::Service<RunTarget>::SharedPtr run_target_srv_;
  rclcpp::Service<Trigger>::SharedPtr list_targets_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "panthera_pose_tuner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto app = std::make_shared<PantheraPoseTuner>(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  app.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
