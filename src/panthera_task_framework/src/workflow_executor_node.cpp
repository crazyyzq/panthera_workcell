#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <panthera_interfaces/msg/external_signal.hpp>
#include <panthera_interfaces/msg/workflow_status.hpp>
#include <panthera_interfaces/srv/run_workflow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

using RunWorkflow = panthera_interfaces::srv::RunWorkflow;
using Trigger = std_srvs::srv::Trigger;
using ExternalSignal = panthera_interfaces::msg::ExternalSignal;
using WorkflowStatus = panthera_interfaces::msg::WorkflowStatus;

namespace
{

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

std::string describePose(const geometry_msgs::msg::PoseStamped & pose)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(3);
  out << "frame=" << pose.header.frame_id
      << " xyz=[" << pose.pose.position.x
      << ", " << pose.pose.position.y
      << ", " << pose.pose.position.z << "]";
  return out.str();
}

}  // namespace

struct PoseSource
{
  std::string name;
  std::string topic;
  double max_age_sec = 1.0;

  std::mutex mutex;
  geometry_msgs::msg::PoseStamped latest_pose;
  rclcpp::Time received_time;
  bool has_pose = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
};

struct BoolSignalSource
{
  std::string topic;

  std::mutex mutex;
  bool latest_value = false;
  rclcpp::Time received_time;
  bool has_value = false;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription;
};

struct ExternalSignalSource
{
  std::string topic;

  std::mutex mutex;
  ExternalSignal latest_signal;
  rclcpp::Time received_time;
  bool has_signal = false;

  rclcpp::Subscription<ExternalSignal>::SharedPtr subscription;
};

class PantheraWorkflowExecutor
{
public:
  explicit PantheraWorkflowExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    logger_(node_->get_logger())
  {
    input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    arm_group_ = getOrDeclareParameter<std::string>("arm_group", "arm");
    hand_frame_ = getOrDeclareParameter<std::string>("hand_frame", "gripper_center");
    base_frame_ = getOrDeclareParameter<std::string>("base_frame", "base_link");
    default_workflow_ = getOrDeclareParameter<std::string>("default_workflow", "cup_pick_place");

    gripper_command_topic_ = getOrDeclareParameter<std::string>(
      "gripper_command_topic", "/gripper_controller/joint_trajectory");
    gripper_joint_ = getOrDeclareParameter<std::string>("gripper_joint", "L_finger_joint");
    gripper_motion_duration_ = getOrDeclareParameter<double>("gripper_motion_duration", 2.0);
    gripper_min_position_ = getOrDeclareParameter<double>("gripper_min_position", 0.0);
    gripper_max_position_ = getOrDeclareParameter<double>("gripper_max_position", 0.05);
    if (gripper_min_position_ > gripper_max_position_) {
      throw std::runtime_error("gripper_min_position must be <= gripper_max_position");
    }

    velocity_scale_ = getOrDeclareParameter<double>("velocity_scale", 0.10);
    acceleration_scale_ = getOrDeclareParameter<double>("acceleration_scale", 0.10);
    planning_time_ = getOrDeclareParameter<double>("planning_time", 5.0);
    planning_attempts_ = getOrDeclareParameter<int>("planning_attempts", 5);
    execute_motion_ = getOrDeclareParameter<bool>("execute_motion", true);
    workflow_status_topic_ = getOrDeclareParameter<std::string>(
      "workflow_status_topic", "/workflow/status");

    workflow_file_ = getOrDeclareParameter<std::string>("workflow_file", defaultWorkflowFile());
    if (workflow_file_.empty()) {
      throw std::runtime_error("workflow_file parameter is empty");
    }

    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
    arm_->setEndEffectorLink(hand_frame_);
    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setMaxVelocityScalingFactor(velocity_scale_);
    arm_->setMaxAccelerationScalingFactor(acceleration_scale_);
    arm_->setPlanningTime(planning_time_);
    arm_->setNumPlanningAttempts(planning_attempts_);

    gripper_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      gripper_command_topic_, 10);
    workflow_status_pub_ = node_->create_publisher<WorkflowStatus>(workflow_status_topic_, 10);

    loadWorkflowFile();

    run_workflow_srv_ = node_->create_service<RunWorkflow>(
      "run_workflow",
      std::bind(
        &PantheraWorkflowExecutor::runWorkflowCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    run_default_srv_ = node_->create_service<Trigger>(
      "run_default_workflow",
      std::bind(
        &PantheraWorkflowExecutor::runDefaultCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    reload_workflows_srv_ = node_->create_service<Trigger>(
      "reload_workflows",
      std::bind(
        &PantheraWorkflowExecutor::reloadWorkflowsCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_workflow_srv_ = node_->create_service<Trigger>(
      "stop_workflow",
      std::bind(
        &PantheraWorkflowExecutor::stopWorkflowCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(logger_, "panthera workflow executor started");
    RCLCPP_INFO(logger_, "workflow_file: %s", workflow_file_.c_str());
    RCLCPP_INFO(logger_, "default_workflow: %s", default_workflow_.c_str());
    RCLCPP_INFO(logger_, "arm_group: %s", arm_group_.c_str());
    RCLCPP_INFO(logger_, "hand_frame: %s", hand_frame_.c_str());
    RCLCPP_INFO(logger_, "gripper_command_topic: %s", gripper_command_topic_.c_str());
    RCLCPP_INFO(
      logger_,
      "gripper limits: [%.3f, %.3f]",
      gripper_min_position_,
      gripper_max_position_);
    RCLCPP_INFO(logger_, "workflow_status_topic: %s", workflow_status_topic_.c_str());
    RCLCPP_INFO(
      logger_,
      "services: /run_workflow, /run_default_workflow, /reload_workflows, /stop_workflow");

    publishWorkflowStatus(
      "",
      "",
      0,
      0,
      WorkflowStatus::STATE_IDLE,
      "workflow executor ready");
  }

private:
  template<typename T>
  T getOrDeclareParameter(
    const std::string & name,
    const T & default_value)
  {
    if (node_->has_parameter(name)) {
      rclcpp::Parameter parameter;
      if (node_->get_parameter(name, parameter)) {
        return parameter.get_value<T>();
      }
    }

    return node_->declare_parameter<T>(name, default_value);
  }

  std::string defaultWorkflowFile() const
  {
    try {
      return ament_index_cpp::get_package_share_directory("panthera_task_framework") +
             "/config/cup_pick_place_workflows.yaml";
    } catch (const std::exception &) {
      return "";
    }
  }

  void loadWorkflowFile()
  {
    config_ = YAML::LoadFile(workflow_file_);
    pose_sources_.clear();

    const auto sources = config_["pose_sources"];
    if (!sources || !sources.IsMap()) {
      throw std::runtime_error("workflow YAML must contain a pose_sources map");
    }

    for (const auto & item : sources) {
      const std::string name = item.first.as<std::string>();
      const YAML::Node source_cfg = item.second;

      if (!source_cfg["topic"]) {
        throw std::runtime_error("pose source '" + name + "' is missing topic");
      }

      auto source = std::make_shared<PoseSource>();
      source->name = name;
      source->topic = source_cfg["topic"].as<std::string>();
      source->max_age_sec = readDouble(source_cfg, "max_age_sec", 1.0);
      source->received_time = node_->now();

      rclcpp::SubscriptionOptions options;
      options.callback_group = input_callback_group_;

      source->subscription = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        source->topic,
        10,
        [this, source](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(source->mutex);
          source->latest_pose = *msg;
          if (source->latest_pose.header.frame_id.empty()) {
            source->latest_pose.header.frame_id = base_frame_;
          }
          source->received_time = node_->now();
          source->has_pose = true;
        },
        options);

      pose_sources_[name] = source;

      RCLCPP_INFO(
        logger_,
        "pose source '%s': topic=%s max_age=%.2fs",
        name.c_str(),
        source->topic.c_str(),
        source->max_age_sec);
    }

    const auto workflows = config_["workflows"];
    if (!workflows || !workflows.IsMap()) {
      throw std::runtime_error("workflow YAML must contain a workflows map");
    }

    preloadSignalSources(workflows);
    logAvailableWorkflows(workflows);
  }

  void logAvailableWorkflows(const YAML::Node & workflows) const
  {
    std::vector<std::string> names;
    for (const auto & item : workflows) {
      names.push_back(item.first.as<std::string>());
    }
    std::sort(names.begin(), names.end());

    std::ostringstream out;
    for (size_t i = 0; i < names.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << names[i];
    }
    RCLCPP_INFO(logger_, "available workflows: %s", out.str().c_str());
  }

  void preloadSignalSources(const YAML::Node & workflows)
  {
    for (const auto & workflow_item : workflows) {
      const auto workflow = workflow_item.second;
      const auto steps = workflow["steps"];
      if (!steps || !steps.IsSequence()) {
        continue;
      }

      for (const auto & step : steps) {
        const std::string type = readString(step, "type", "");
        if (type == "wait_for_bool") {
          const std::string topic = readString(step, "topic", "");
          if (!topic.empty()) {
            getBoolSignalSource(topic);
          }
        } else if (type == "wait_for_signal") {
          getExternalSignalSource(readString(step, "topic", "/workflow/external_signal"));
        }
      }
    }
  }

  static std::string readString(
    const YAML::Node & node,
    const std::string & key,
    const std::string & default_value)
  {
    if (!node || !node[key]) {
      return default_value;
    }
    return node[key].as<std::string>();
  }

  static double readDouble(
    const YAML::Node & node,
    const std::string & key,
    double default_value)
  {
    if (!node || !node[key]) {
      return default_value;
    }
    return node[key].as<double>();
  }

  static bool readBool(
    const YAML::Node & node,
    const std::string & key,
    bool default_value)
  {
    if (!node || !node[key]) {
      return default_value;
    }
    return node[key].as<bool>();
  }

  static std::array<double, 3> readVec3(
    const YAML::Node & node,
    const std::string & key,
    const std::array<double, 3> & default_value)
  {
    if (!node || !node[key]) {
      return default_value;
    }
    return readVec3Node(node[key], key);
  }

  static std::array<double, 3> readVec3Node(
    const YAML::Node & node,
    const std::string & label)
  {
    if (!node || !node.IsSequence() || node.size() != 3) {
      throw std::runtime_error(label + " must be a 3 element numeric array");
    }
    return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
  }

  static std::array<double, 4> readQuatNode(
    const YAML::Node & node,
    const std::string & label)
  {
    if (!node || !node.IsSequence() || node.size() != 4) {
      throw std::runtime_error(label + " must be a 4 element xyzw numeric array");
    }
    return {
      node[0].as<double>(),
      node[1].as<double>(),
      node[2].as<double>(),
      node[3].as<double>()};
  }

  static std::vector<double> readDoubleVector(
    const YAML::Node & node,
    const std::string & key)
  {
    if (!node[key] || !node[key].IsSequence()) {
      throw std::runtime_error(key + " must be a numeric array");
    }

    std::vector<double> values;
    values.reserve(node[key].size());
    for (const auto & value : node[key]) {
      values.push_back(value.as<double>());
    }
    return values;
  }

  std::optional<std::array<double, 4>> defaultOrientation() const
  {
    const auto defaults = config_["defaults"];
    if (defaults && defaults["orientation_xyzw"]) {
      return readQuatNode(defaults["orientation_xyzw"], "defaults.orientation_xyzw");
    }
    return std::nullopt;
  }

  void setOrientation(
    geometry_msgs::msg::PoseStamped & pose,
    const std::array<double, 4> & xyzw) const
  {
    pose.pose.orientation.x = xyzw[0];
    pose.pose.orientation.y = xyzw[1];
    pose.pose.orientation.z = xyzw[2];
    pose.pose.orientation.w = xyzw[3];
  }

  bool getPoseFromSource(
    const std::string & source_name,
    double max_age_sec,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & error_msg)
  {
    const auto it = pose_sources_.find(source_name);
    if (it == pose_sources_.end()) {
      error_msg = "unknown pose source: " + source_name;
      return false;
    }

    const auto & source = it->second;
    std::lock_guard<std::mutex> lock(source->mutex);

    if (!source->has_pose) {
      error_msg = "pose source has not received data yet: " + source_name;
      return false;
    }

    const double effective_max_age = max_age_sec > 0.0 ? max_age_sec : source->max_age_sec;
    const double age = (node_->now() - source->received_time).seconds();
    if (effective_max_age > 0.0 && age > effective_max_age) {
      std::ostringstream out;
      out.setf(std::ios::fixed);
      out.precision(2);
      out << "pose source '" << source_name << "' is stale: age="
          << age << "s max=" << effective_max_age << "s";
      error_msg = out.str();
      return false;
    }

    pose = source->latest_pose;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = base_frame_;
    }
    return true;
  }

  bool waitForPose(
    const YAML::Node & step,
    std::string & error_msg)
  {
    const std::string source_name = readString(step, "source", "");
    if (source_name.empty()) {
      error_msg = "wait_for_pose step is missing source";
      return false;
    }

    const double timeout_sec = readDouble(step, "timeout_sec", 5.0);
    const double max_age_sec = readDouble(step, "max_age_sec", -1.0);
    const rclcpp::Time start = node_->now();

    while (rclcpp::ok()) {
      geometry_msgs::msg::PoseStamped pose;
      std::string pose_error;
      if (getPoseFromSource(source_name, max_age_sec, pose, pose_error)) {
        RCLCPP_INFO(
          logger_,
          "[POSE] source '%s' ready: %s",
          source_name.c_str(),
          describePose(pose).c_str());
        return true;
      }

      if ((node_->now() - start).seconds() >= timeout_sec) {
        error_msg = "timed out waiting for pose source '" + source_name + "': " + pose_error;
        return false;
      }

      std::this_thread::sleep_for(100ms);
    }

    error_msg = "ROS shutdown while waiting for pose";
    return false;
  }

  std::shared_ptr<BoolSignalSource> getBoolSignalSource(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(signal_sources_mutex_);

    const auto it = bool_signal_sources_.find(topic);
    if (it != bool_signal_sources_.end()) {
      return it->second;
    }

    auto source = std::make_shared<BoolSignalSource>();
    source->topic = topic;
    source->received_time = node_->now();

    rclcpp::SubscriptionOptions options;
    options.callback_group = input_callback_group_;

    source->subscription = node_->create_subscription<std_msgs::msg::Bool>(
      topic,
      10,
      [this, source](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> source_lock(source->mutex);
        source->latest_value = msg->data;
        source->received_time = node_->now();
        source->has_value = true;
      },
      options);

    bool_signal_sources_[topic] = source;
    RCLCPP_INFO(logger_, "bool signal source: topic=%s", topic.c_str());
    return source;
  }

  std::shared_ptr<ExternalSignalSource> getExternalSignalSource(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(signal_sources_mutex_);

    const auto it = external_signal_sources_.find(topic);
    if (it != external_signal_sources_.end()) {
      return it->second;
    }

    auto source = std::make_shared<ExternalSignalSource>();
    source->topic = topic;
    source->received_time = node_->now();

    rclcpp::SubscriptionOptions options;
    options.callback_group = input_callback_group_;

    source->subscription = node_->create_subscription<ExternalSignal>(
      topic,
      10,
      [this, source](const ExternalSignal::SharedPtr msg) {
        std::lock_guard<std::mutex> source_lock(source->mutex);
        source->latest_signal = *msg;
        source->received_time = node_->now();
        source->has_signal = true;
        RCLCPP_INFO(
          logger_,
          "[SIGNAL] received external topic '%s' source=%s name=%s code=%d active=%s workflow=%s",
          source->topic.c_str(),
          msg->source.c_str(),
          msg->name.c_str(),
          msg->code,
          msg->active ? "true" : "false",
          msg->workflow_name.c_str());
      },
      options);

    external_signal_sources_[topic] = source;
    RCLCPP_INFO(logger_, "external signal source: topic=%s", topic.c_str());
    return source;
  }

  bool isFreshEnough(
    const rclcpp::Time & received_time,
    const rclcpp::Time & start_time,
    bool require_after_start,
    double max_age_sec) const
  {
    if (require_after_start && (received_time - start_time).nanoseconds() < 0) {
      return false;
    }
    if (max_age_sec > 0.0 && (node_->now() - received_time).seconds() > max_age_sec) {
      return false;
    }
    return true;
  }

  bool waitForBoolSignal(
    const YAML::Node & step,
    std::string & error_msg)
  {
    const std::string topic = readString(step, "topic", "");
    if (topic.empty()) {
      error_msg = "wait_for_bool step is missing topic";
      return false;
    }

    const bool expected = readBool(step, "expected", true);
    const double timeout_sec = readDouble(step, "timeout_sec", 10.0);
    const bool fresh = readBool(step, "fresh", true);
    const double max_age_sec = readDouble(step, "max_age_sec", 0.0);
    const auto source = getBoolSignalSource(topic);
    const rclcpp::Time start = node_->now();

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(source->mutex);
        if (source->has_value &&
          source->latest_value == expected &&
          isFreshEnough(source->received_time, start, fresh, max_age_sec))
        {
          RCLCPP_INFO(
            logger_,
            "[SIGNAL] bool topic '%s' matched expected=%s",
            topic.c_str(),
            expected ? "true" : "false");
          return true;
        }
      }

      if ((node_->now() - start).seconds() >= timeout_sec) {
        error_msg = "timed out waiting for bool signal topic '" + topic + "'";
        return false;
      }

      std::this_thread::sleep_for(50ms);
    }

    error_msg = "ROS shutdown while waiting for bool signal";
    return false;
  }

  bool externalSignalMatches(
    const ExternalSignal & signal,
    const YAML::Node & step) const
  {
    std::string expected_source = readString(step, "signal_source", "");
    if (expected_source.empty()) {
      expected_source = readString(step, "source", "");
    }
    if (!expected_source.empty() && signal.source != expected_source) {
      return false;
    }

    std::string expected_name = readString(step, "signal_name", "");
    if (expected_name.empty()) {
      expected_name = readString(step, "input_name", "");
    }
    if (expected_name.empty()) {
      expected_name = readString(step, "name", "");
    }
    if (!expected_name.empty() && signal.name != expected_name) {
      return false;
    }

    const std::string expected_workflow = readString(step, "workflow_name", "");
    if (!expected_workflow.empty() && signal.workflow_name != expected_workflow) {
      return false;
    }

    if (step["code"] && signal.code != step["code"].as<int>()) {
      return false;
    }

    const bool expected_active = readBool(step, "active", true);
    if (signal.active != expected_active) {
      return false;
    }

    return true;
  }

  bool waitForExternalSignal(
    const YAML::Node & step,
    std::string & error_msg)
  {
    const std::string topic = readString(step, "topic", "/workflow/external_signal");
    const double timeout_sec = readDouble(step, "timeout_sec", 10.0);
    const bool fresh = readBool(step, "fresh", true);
    const double max_age_sec = readDouble(step, "max_age_sec", 0.0);
    const auto source = getExternalSignalSource(topic);
    const rclcpp::Time start = node_->now();
    rclcpp::Time last_diagnostic = start;

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(source->mutex);
        if (source->has_signal) {
          const bool matches = externalSignalMatches(source->latest_signal, step);
          const bool fresh_enough = isFreshEnough(source->received_time, start, fresh, max_age_sec);
          if (matches && fresh_enough) {
            RCLCPP_INFO(
              logger_,
              "[SIGNAL] external topic '%s' matched name=%s code=%d active=%s workflow=%s",
              topic.c_str(),
              source->latest_signal.name.c_str(),
              source->latest_signal.code,
              source->latest_signal.active ? "true" : "false",
              source->latest_signal.workflow_name.c_str());
            return true;
          }

          if ((node_->now() - last_diagnostic).seconds() >= 1.0) {
            std::string expected_source = readString(step, "signal_source", "");
            if (expected_source.empty()) {
              expected_source = readString(step, "source", "");
            }
            std::string expected_name = readString(step, "signal_name", "");
            if (expected_name.empty()) {
              expected_name = readString(step, "input_name", "");
            }
            if (expected_name.empty()) {
              expected_name = readString(step, "name", "");
            }
            const std::string expected_workflow = readString(step, "workflow_name", "");
            const bool expected_active = readBool(step, "active", true);
            const bool has_expected_code = static_cast<bool>(step["code"]);
            const int expected_code =
              has_expected_code ? step["code"].as<int>() : source->latest_signal.code;
            RCLCPP_WARN(
              logger_,
              "[SIGNAL] latest external topic '%s' not ready: matches=%s fresh=%s "
              "expected[source=%s name=%s code=%d active=%s workflow=%s] "
              "actual[source=%s name=%s code=%d active=%s workflow=%s]",
              topic.c_str(),
              matches ? "true" : "false",
              fresh_enough ? "true" : "false",
              expected_source.c_str(),
              expected_name.c_str(),
              expected_code,
              expected_active ? "true" : "false",
              expected_workflow.c_str(),
              source->latest_signal.source.c_str(),
              source->latest_signal.name.c_str(),
              source->latest_signal.code,
              source->latest_signal.active ? "true" : "false",
              source->latest_signal.workflow_name.c_str());
            last_diagnostic = node_->now();
          }
        }
      }

      if ((node_->now() - start).seconds() >= timeout_sec) {
        error_msg = "timed out waiting for external signal topic '" + topic + "'";
        return false;
      }

      std::this_thread::sleep_for(50ms);
    }

    error_msg = "ROS shutdown while waiting for external signal";
    return false;
  }

  geometry_msgs::msg::PoseStamped buildPoseFromStep(
    const YAML::Node & step)
  {
    geometry_msgs::msg::PoseStamped target;
    bool has_source_pose = false;

    if (step["source"]) {
      const std::string source_name = step["source"].as<std::string>();
      const double max_age_sec = readDouble(step, "max_age_sec", -1.0);
      std::string error_msg;
      if (!getPoseFromSource(source_name, max_age_sec, target, error_msg)) {
        throw std::runtime_error(error_msg);
      }
      has_source_pose = true;
      target.header.stamp = node_->now();

      const auto offset = readVec3(step, "offset_xyz", {0.0, 0.0, 0.0});
      target.pose.position.x += offset[0];
      target.pose.position.y += offset[1];
      target.pose.position.z += offset[2];
    } else {
      const auto position = readVec3Node(step["position"], "position");
      target.header.frame_id = readString(step, "frame", base_frame_);
      target.header.stamp = node_->now();
      target.pose.position.x = position[0];
      target.pose.position.y = position[1];
      target.pose.position.z = position[2];
      target.pose.orientation.w = 1.0;
    }

    if (step["orientation_xyzw"]) {
      setOrientation(target, readQuatNode(step["orientation_xyzw"], "orientation_xyzw"));
    } else if (const auto quat = defaultOrientation()) {
      setOrientation(target, quat.value());
    } else if (!has_source_pose) {
      target.pose.orientation.w = 1.0;
    }

    if (target.header.frame_id.empty()) {
      target.header.frame_id = base_frame_;
    }

    return target;
  }

  bool moveArmToPose(
    const std::string & name,
    const geometry_msgs::msg::PoseStamped & target,
    bool execute_step,
    std::string & error_msg)
  {
    RCLCPP_INFO(logger_, "[ARM] %s -> %s", name.c_str(), describePose(target).c_str());

    arm_->setStartStateToCurrentState();
    arm_->setPoseReferenceFrame(target.header.frame_id);
    arm_->setPoseTarget(target, hand_frame_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_->plan(plan);
    arm_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt plan failed for pose step: " + name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] plan success: %s", name.c_str());

    if (!execute_step) {
      RCLCPP_WARN(logger_, "[ARM] execution disabled, skip: %s", name.c_str());
      return true;
    }

    const auto exec_result = arm_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt execute failed for pose step: " + name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] execute success: %s", name.c_str());
    return true;
  }

  bool moveArmCartesianToPose(
    const YAML::Node & step,
    const std::string & name,
    const geometry_msgs::msg::PoseStamped & target,
    bool execute_step,
    std::string & error_msg)
  {
    const auto defaults = config_["defaults"];
    const double eef_step = readDouble(
      step,
      "eef_step",
      readDouble(defaults, "cartesian_eef_step", 0.005));
    const double jump_threshold = readDouble(
      step,
      "jump_threshold",
      readDouble(defaults, "cartesian_jump_threshold", 0.0));
    const double min_fraction = readDouble(
      step,
      "min_fraction",
      readDouble(defaults, "cartesian_min_fraction", 0.95));

    RCLCPP_INFO(
      logger_,
      "[ARM] %s cartesian -> %s",
      name.c_str(),
      describePose(target).c_str());

    arm_->setStartStateToCurrentState();
    arm_->setPoseReferenceFrame(target.header.frame_id);

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target.pose);

    const double fraction = arm_->computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory,
      true);

    if (fraction < min_fraction) {
      std::ostringstream out;
      out.setf(std::ios::fixed);
      out.precision(3);
      out << "Cartesian path fraction too low for " << name
          << ": fraction=" << fraction
          << " min=" << min_fraction;
      error_msg = out.str();
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] cartesian plan success: %s fraction=%.3f", name.c_str(), fraction);

    if (!execute_step) {
      RCLCPP_WARN(logger_, "[ARM] execution disabled, skip cartesian: %s", name.c_str());
      return true;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto exec_result = arm_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt execute failed for cartesian step: " + name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] cartesian execute success: %s", name.c_str());
    return true;
  }

  bool moveArmToNamedState(
    const std::string & name,
    const std::string & state_name,
    bool execute_step,
    std::string & error_msg)
  {
    RCLCPP_INFO(logger_, "[ARM] %s -> named state '%s'", name.c_str(), state_name.c_str());

    arm_->setStartStateToCurrentState();
    if (!arm_->setNamedTarget(state_name)) {
      error_msg = "failed to set named target: " + state_name;
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt plan failed for named state: " + state_name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] plan success: %s -> %s", name.c_str(), state_name.c_str());

    if (!execute_step) {
      RCLCPP_WARN(logger_, "[ARM] execution disabled, skip: %s", name.c_str());
      return true;
    }

    const auto exec_result = arm_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt execute failed for named state: " + state_name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] execute success: %s -> %s", name.c_str(), state_name.c_str());
    return true;
  }

  bool moveArmToJointTarget(
    const std::string & name,
    const std::vector<double> & joint_positions,
    bool execute_step,
    std::string & error_msg)
  {
    RCLCPP_INFO(
      logger_, "[ARM] %s -> joint target with %zu joints",
      name.c_str(), joint_positions.size());

    arm_->setStartStateToCurrentState();
    if (!arm_->setJointValueTarget(joint_positions)) {
      error_msg = "failed to set joint target for step: " + name;
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt plan failed for joint step: " + name;
      return false;
    }

    if (!execute_step) {
      RCLCPP_WARN(logger_, "[ARM] execution disabled, skip joint target: %s", name.c_str());
      return true;
    }

    const auto exec_result = arm_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_msg = "MoveIt execute failed for joint step: " + name;
      return false;
    }

    RCLCPP_INFO(logger_, "[ARM] execute success: %s", name.c_str());
    return true;
  }

  bool moveArmRelative(
    const YAML::Node & step,
    const std::string & name,
    bool execute_step,
    std::string & error_msg)
  {
    auto target = arm_->getCurrentPose(hand_frame_);
    const auto offset = readVec3Node(step["offset_xyz"], "offset_xyz");
    target.header.stamp = node_->now();
    target.pose.position.x += offset[0];
    target.pose.position.y += offset[1];
    target.pose.position.z += offset[2];

    if (step["orientation_xyzw"]) {
      setOrientation(target, readQuatNode(step["orientation_xyzw"], "orientation_xyzw"));
    }

    const bool cartesian = readBool(step, "cartesian", true);
    if (cartesian) {
      return moveArmCartesianToPose(step, name, target, execute_step, error_msg);
    }
    return moveArmToPose(name, target, execute_step, error_msg);
  }

  bool sendGripperTo(
    const std::string & name,
    double position,
    double duration_sec,
    bool execute_step,
    std::string & error_msg)
  {
    if (position < gripper_min_position_ || position > gripper_max_position_) {
      std::ostringstream out;
      out.setf(std::ios::fixed);
      out.precision(3);
      out << "gripper position " << position
          << " out of configured limits ["
          << gripper_min_position_ << ", "
          << gripper_max_position_ << "]";
      error_msg = out.str();
      return false;
    }

    RCLCPP_INFO(
      logger_,
      "[GRIPPER] %s -> %.3f duration=%.2fs",
      name.c_str(),
      position,
      duration_sec);

    if (!execute_step) {
      RCLCPP_WARN(logger_, "[GRIPPER] execution disabled, skip: %s", name.c_str());
      return true;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    traj.joint_names.push_back(gripper_joint_);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.velocities.push_back(0.0);
    point.time_from_start = secondsToDuration(duration_sec);
    traj.points.push_back(point);

    gripper_pub_->publish(traj);

    const double sleep_sec = std::max(0.0, duration_sec + 0.2);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(sleep_sec * 1000.0)));

    return true;
  }

  bool applyCollisionObject(
    const YAML::Node & step,
    std::string & error_msg)
  {
    const std::string operation = readString(step, "operation", "add");
    const std::string id = readString(step, "id", "");
    if (id.empty()) {
      error_msg = "collision_object step is missing id";
      return false;
    }

    if (operation == "remove") {
      planning_scene_.removeCollisionObjects({id});
      RCLCPP_INFO(logger_, "[SCENE] removed collision object: %s", id.c_str());
      std::this_thread::sleep_for(200ms);
      return true;
    }

    if (operation != "add") {
      error_msg = "unsupported collision_object operation: " + operation;
      return false;
    }

    const auto pose = buildPoseFromStep(step);
    const std::string shape = readString(step, "shape", "box");

    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = pose.header.frame_id;
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    if (shape == "cylinder") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        readDouble(step, "height", 0.05);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        readDouble(step, "radius", 0.03);
    } else if (shape == "sphere") {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
        readDouble(step, "radius", 0.03);
    } else if (shape == "box") {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions.resize(3);
      const auto size = readVec3(step, "size_xyz", {0.05, 0.05, 0.05});
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size[0];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size[1];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size[2];
    } else {
      error_msg = "unsupported collision object shape: " + shape;
      return false;
    }

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose.pose);

    planning_scene_.applyCollisionObject(object);
    RCLCPP_INFO(logger_, "[SCENE] added collision object '%s' shape=%s", id.c_str(), shape.c_str());
    std::this_thread::sleep_for(200ms);
    return true;
  }

  bool attachObject(
    const YAML::Node & step,
    bool attach,
    std::string & error_msg)
  {
    const std::string id = readString(step, "id", "");
    if (id.empty()) {
      error_msg = attach ? "attach_object step is missing id" : "detach_object step is missing id";
      return false;
    }

    if (attach) {
      std::vector<std::string> touch_links;
      if (step["touch_links"]) {
        if (!step["touch_links"].IsSequence()) {
          error_msg = "touch_links must be a string array";
          return false;
        }
        for (const auto & link : step["touch_links"]) {
          touch_links.push_back(link.as<std::string>());
        }
      }

      const std::string link = readString(step, "link", hand_frame_);
      arm_->attachObject(id, link, touch_links);
      RCLCPP_INFO(logger_, "[SCENE] attached object '%s' to %s", id.c_str(), link.c_str());
    } else {
      arm_->detachObject(id);
      RCLCPP_INFO(logger_, "[SCENE] detached object '%s'", id.c_str());
    }

    std::this_thread::sleep_for(200ms);
    return true;
  }

  bool executeStep(
    const YAML::Node & step,
    bool execute_step,
    std::string & error_msg)
  {
    if (!readBool(step, "enabled", true)) {
      RCLCPP_INFO(
        logger_, "[STEP] skip disabled step: %s", readString(
          step, "name",
          "<unnamed>").c_str());
      return true;
    }

    const std::string type = readString(step, "type", "");
    const std::string name = readString(step, "name", type.empty() ? "<unnamed>" : type);

    if (type.empty()) {
      error_msg = "workflow step is missing type";
      return false;
    }

    RCLCPP_INFO(logger_, "[STEP] %s (%s)", name.c_str(), type.c_str());

    if (type == "wait_for_pose") {
      return waitForPose(step, error_msg);
    }

    if (type == "wait_for_bool") {
      return waitForBoolSignal(step, error_msg);
    }

    if (type == "wait_for_signal") {
      return waitForExternalSignal(step, error_msg);
    }

    if (type == "move_pose") {
      const auto target = buildPoseFromStep(step);
      const bool cartesian = readBool(step, "cartesian", false);
      if (cartesian) {
        return moveArmCartesianToPose(step, name, target, execute_step, error_msg);
      }
      return moveArmToPose(name, target, execute_step, error_msg);
    }

    if (type == "move_relative") {
      return moveArmRelative(step, name, execute_step, error_msg);
    }

    if (type == "move_named") {
      const std::string state = readString(step, "state", "");
      if (state.empty()) {
        error_msg = "move_named step is missing state";
        return false;
      }
      return moveArmToNamedState(name, state, execute_step, error_msg);
    }

    if (type == "move_joint") {
      return moveArmToJointTarget(
        name,
        readDoubleVector(step, "positions"),
        execute_step,
        error_msg);
    }

    if (type == "gripper") {
      const double position = readDouble(step, "position", 0.0);
      const double duration = readDouble(step, "duration_sec", gripper_motion_duration_);
      return sendGripperTo(name, position, duration, execute_step, error_msg);
    }

    if (type == "sleep") {
      const double seconds = readDouble(step, "seconds", 0.0);
      if (execute_step && seconds > 0.0) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(seconds * 1000.0)));
      }
      return true;
    }

    if (type == "collision_object") {
      return applyCollisionObject(step, error_msg);
    }

    if (type == "attach_object") {
      return attachObject(step, true, error_msg);
    }

    if (type == "detach_object") {
      return attachObject(step, false, error_msg);
    }

    error_msg = "unsupported workflow step type: " + type;
    return false;
  }

  bool runWorkflow(
    const std::string & workflow_name,
    bool dry_run,
    std::string & message)
  {
    std::unique_lock<std::mutex> run_lock(run_mutex_, std::try_to_lock);
    if (!run_lock.owns_lock()) {
      message = "another workflow is already running";
      publishWorkflowStatus(
        workflow_name,
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        message);
      return false;
    }

    const auto workflow = config_["workflows"][workflow_name];
    if (!workflow) {
      message = "unknown workflow: " + workflow_name;
      publishWorkflowStatus(
        workflow_name,
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        message);
      return false;
    }

    const auto steps = workflow["steps"];
    if (!steps || !steps.IsSequence()) {
      message = "workflow '" + workflow_name + "' must contain a steps array";
      publishWorkflowStatus(
        workflow_name,
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        message);
      return false;
    }

    const bool execute_step = execute_motion_ && !dry_run;
    stop_requested_.store(false);
    RCLCPP_INFO(
      logger_,
      "run workflow '%s': steps=%zu execute=%s dry_run=%s",
      workflow_name.c_str(),
      steps.size(),
      execute_step ? "true" : "false",
      dry_run ? "true" : "false");

    size_t index = 0;
    for (const auto & step : steps) {
      ++index;
      std::string error_msg;
      const std::string step_type = readString(step, "type", "");
      const std::string step_name = readString(
        step,
        "name",
        step_type.empty() ? "<unnamed>" : step_type);

      if (stop_requested_.load()) {
        message = "workflow '" + workflow_name + "' stopped before step " + std::to_string(index);
        RCLCPP_WARN(logger_, "%s", message.c_str());
        publishWorkflowStatus(
          workflow_name,
          step_name,
          static_cast<uint32_t>(index),
          static_cast<uint32_t>(steps.size()),
          WorkflowStatus::STATE_FAILED,
          message);
        return false;
      }

      publishWorkflowStatus(
        workflow_name,
        step_name,
        static_cast<uint32_t>(index),
        static_cast<uint32_t>(steps.size()),
        WorkflowStatus::STATE_RUNNING,
        "running");

      try {
        if (!executeStep(step, execute_step, error_msg)) {
          std::ostringstream out;
          out << "workflow '" << workflow_name << "' failed at step "
              << index << ": " << error_msg;
          message = out.str();
          RCLCPP_ERROR(logger_, "%s", message.c_str());
          publishWorkflowStatus(
            workflow_name,
            step_name,
            static_cast<uint32_t>(index),
            static_cast<uint32_t>(steps.size()),
            WorkflowStatus::STATE_FAILED,
            message);
          return false;
        }
      } catch (const std::exception & e) {
        std::ostringstream out;
        out << "workflow '" << workflow_name << "' exception at step "
            << index << ": " << e.what();
        message = out.str();
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        publishWorkflowStatus(
          workflow_name,
          step_name,
          static_cast<uint32_t>(index),
          static_cast<uint32_t>(steps.size()),
          WorkflowStatus::STATE_FAILED,
          message);
        return false;
      }
    }

    message = "workflow '" + workflow_name + "' finished";
    RCLCPP_INFO(logger_, "%s", message.c_str());
    publishWorkflowStatus(
      workflow_name,
      "",
      static_cast<uint32_t>(steps.size()),
      static_cast<uint32_t>(steps.size()),
      WorkflowStatus::STATE_SUCCEEDED,
      message);
    return true;
  }

  void stopWorkflowCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    stop_requested_.store(true);
    try {
      if (arm_) {
        arm_->stop();
        arm_->clearPoseTargets();
      }
      response->success = true;
      response->message = "workflow stop requested";
      publishWorkflowStatus(
        "",
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        "workflow stop requested");
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = std::string("workflow stop failed: ") + exc.what();
    }
  }

  void publishWorkflowStatus(
    const std::string & workflow_name,
    const std::string & current_step,
    uint32_t step_index,
    uint32_t step_count,
    uint8_t state,
    const std::string & message)
  {
    if (!workflow_status_pub_) {
      return;
    }

    WorkflowStatus status;
    status.header.stamp = node_->now();
    status.workflow_name = workflow_name;
    status.current_step = current_step;
    status.step_index = step_index;
    status.step_count = step_count;
    status.state = state;
    status.message = message;
    workflow_status_pub_->publish(status);
  }

  void runWorkflowCallback(
    const std::shared_ptr<RunWorkflow::Request> request,
    std::shared_ptr<RunWorkflow::Response> response)
  {
    const std::string workflow_name =
      request->workflow_name.empty() ? default_workflow_ : request->workflow_name;

    response->success = startWorkflowAsync(workflow_name, request->dry_run, response->message);
  }

  void runDefaultCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    response->success = startWorkflowAsync(default_workflow_, false, response->message);
  }

  void reloadWorkflowsCallback(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (workflow_running_.load()) {
      response->success = false;
      response->message = "cannot reload workflows while a workflow is running";
      return;
    }

    std::unique_lock<std::mutex> run_lock(run_mutex_, std::try_to_lock);
    if (!run_lock.owns_lock()) {
      response->success = false;
      response->message = "cannot reload workflows while a workflow is running";
      return;
    }

    try {
      {
        std::lock_guard<std::mutex> signal_lock(signal_sources_mutex_);
        bool_signal_sources_.clear();
        external_signal_sources_.clear();
      }
      loadWorkflowFile();
      response->success = true;
      response->message = "reloaded workflow file: " + workflow_file_;
      publishWorkflowStatus(
        "",
        "",
        0,
        0,
        WorkflowStatus::STATE_IDLE,
        response->message);
    } catch (const std::exception & e) {
      response->success = false;
      response->message = e.what();
      publishWorkflowStatus(
        "",
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        response->message);
    }
  }

  bool startWorkflowAsync(
    const std::string & workflow_name,
    bool dry_run,
    std::string & message)
  {
    bool expected = false;
    if (!workflow_running_.compare_exchange_strong(expected, true)) {
      message = "another workflow is already running";
      return false;
    }

    const auto workflow = config_["workflows"][workflow_name];
    if (!workflow) {
      message = "unknown workflow: " + workflow_name;
      workflow_running_.store(false);
      publishWorkflowStatus(
        workflow_name,
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        message);
      return false;
    }

    const auto steps = workflow["steps"];
    if (!steps || !steps.IsSequence()) {
      message = "workflow '" + workflow_name + "' must contain a steps array";
      workflow_running_.store(false);
      publishWorkflowStatus(
        workflow_name,
        "",
        0,
        0,
        WorkflowStatus::STATE_FAILED,
        message);
      return false;
    }

    message = "started workflow '" + workflow_name + "'";
    std::thread(
      [this, workflow_name, dry_run]() {
        std::string result_message;
        runWorkflow(workflow_name, dry_run, result_message);
        workflow_running_.store(false);
      }).detach();
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  std::string workflow_file_;
  YAML::Node config_;

  std::string arm_group_;
  std::string hand_frame_;
  std::string base_frame_;
  std::string default_workflow_;
  std::string gripper_command_topic_;
  std::string gripper_joint_;
  std::string workflow_status_topic_;

  double gripper_motion_duration_;
  double gripper_min_position_;
  double gripper_max_position_;
  double velocity_scale_;
  double acceleration_scale_;
  double planning_time_;
  int planning_attempts_;
  bool execute_motion_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<WorkflowStatus>::SharedPtr workflow_status_pub_;
  rclcpp::Service<RunWorkflow>::SharedPtr run_workflow_srv_;
  rclcpp::Service<Trigger>::SharedPtr run_default_srv_;
  rclcpp::Service<Trigger>::SharedPtr reload_workflows_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_workflow_srv_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;

  std::map<std::string, std::shared_ptr<PoseSource>> pose_sources_;
  std::map<std::string, std::shared_ptr<BoolSignalSource>> bool_signal_sources_;
  std::map<std::string, std::shared_ptr<ExternalSignalSource>> external_signal_sources_;
  std::mutex signal_sources_mutex_;
  std::mutex run_mutex_;
  std::atomic_bool workflow_running_{false};
  std::atomic_bool stop_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "panthera_workflow_executor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto executor_object = std::make_shared<PantheraWorkflowExecutor>(node);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  executor.remove_node(node);
  executor_object.reset();
  rclcpp::shutdown();
  return 0;
}
