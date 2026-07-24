#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <panthera_interfaces/action/execute_motion.hpp>
#include <panthera_interfaces/srv/set_speed_scale.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "panthera_motion/Catalog.hpp"
#include "panthera_motion/Compiler.hpp"

using namespace std::chrono_literals;

namespace panthera_motion
{
namespace
{

using ExecuteMotion = panthera_interfaces::action::ExecuteMotion;
using GoalHandleMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleController = rclcpp_action::ClientGoalHandle<FollowTrajectory>;
using Trigger = std_srvs::srv::Trigger;
using SetSpeedScale = panthera_interfaces::srv::SetSpeedScale;

std::string defaultCatalogPath()
{
  return ament_index_cpp::get_package_share_directory("panthera_motion") +
         "/config/motion_catalog.yaml";
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

double maxAbsVelocity(
  const sensor_msgs::msg::JointState & state,
  const std::vector<std::string> & joint_names)
{
  double maximum = 0.0;
  for (const auto & joint_name : joint_names) {
    const auto it = std::find(state.name.begin(), state.name.end(), joint_name);
    if (it == state.name.end()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto index = static_cast<std::size_t>(std::distance(state.name.begin(), it));
    if (index >= state.velocity.size() || !std::isfinite(state.velocity[index])) {
      return std::numeric_limits<double>::infinity();
    }
    maximum = std::max(maximum, std::abs(state.velocity[index]));
  }
  return maximum;
}

builtin_interfaces::msg::Duration durationFromNanoseconds(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

std::int64_t durationNanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL + duration.nanosec;
}

bool addContinuousRoute(
  std::map<std::string, CompiledRoute> & routes,
  const std::string & name,
  const std::vector<std::string> & part_names,
  std::string & error)
{
  CompiledRoute combined;
  combined.name = name;
  std::int64_t offset_ns = 0;
  for (const auto & part_name : part_names) {
    const auto part_it = routes.find(part_name);
    if (part_it == routes.end()) {
      error = "continuous route '" + name + "' is missing part '" + part_name + "'";
      return false;
    }
    const auto & part = part_it->second;
    if (part.trajectory.points.size() < 2) {
      error = "continuous route part '" + part_name + "' has fewer than 2 points";
      return false;
    }
    if (combined.trajectory.points.empty()) {
      combined.trajectory = part.trajectory;
      combined.start_joints = part.start_joints;
      offset_ns = durationNanoseconds(combined.trajectory.points.back().time_from_start);
    } else {
      if (combined.trajectory.joint_names != part.trajectory.joint_names ||
        combined.end_joints.size() != part.start_joints.size())
      {
        error = "continuous route part '" + part_name + "' has incompatible joints";
        return false;
      }
      double maximum_difference = 0.0;
      std::size_t maximum_joint = 0;
      for (std::size_t joint = 0; joint < combined.end_joints.size(); ++joint) {
        const double difference =
          std::abs(combined.end_joints[joint] - part.start_joints[joint]);
        if (difference > maximum_difference) {
          maximum_difference = difference;
          maximum_joint = joint;
        }
      }
      if (maximum_difference > 1e-3) {
        std::ostringstream out;
        out << "continuous route part '" << part_name
            << "' start mismatch at " << combined.trajectory.joint_names[maximum_joint]
            << "=" << maximum_difference << "rad";
        error = out.str();
        return false;
      }
      combined.trajectory.points.back().positions = part.start_joints;
      combined.end_joints = part.start_joints;
      // Skip the duplicate zero-time start point. The preceding route's final
      // point is identical and becomes the next spline's start, eliminating
      // controller-goal round-trip dwell without changing either phase profile.
      for (std::size_t index = 1; index < part.trajectory.points.size(); ++index) {
        auto point = part.trajectory.points[index];
        point.time_from_start = durationFromNanoseconds(
          offset_ns + durationNanoseconds(point.time_from_start));
        combined.trajectory.points.push_back(std::move(point));
      }
      offset_ns = durationNanoseconds(combined.trajectory.points.back().time_from_start);
    }
    combined.end_joints = part.end_joints;
    combined.segment_names.insert(
      combined.segment_names.end(), part.segment_names.begin(), part.segment_names.end());
    combined.content_hash += part.content_hash;
  }
  combined.duration_sec = static_cast<double>(offset_ns) * 1e-9;
  routes.emplace(name, std::move(combined));
  return true;
}

bool isPlausibleArmJointState(
  const sensor_msgs::msg::JointState & state,
  const sensor_msgs::msg::JointState * previous)
{
  const std::array<std::string, 6> joint_names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  for (const auto & name : joint_names) {
    const auto it = std::find(state.name.begin(), state.name.end(), name);
    if (it == state.name.end()) {
      return false;
    }
    const auto index = static_cast<std::size_t>(std::distance(state.name.begin(), it));
    if (index >= state.position.size() || !std::isfinite(state.position[index]) ||
      std::abs(state.position[index]) > 10.0)
    {
      return false;
    }
    if (index < state.velocity.size() &&
      (!std::isfinite(state.velocity[index]) || std::abs(state.velocity[index]) > 10.0))
    {
      return false;
    }
    if (previous) {
      const auto previous_it = std::find(previous->name.begin(), previous->name.end(), name);
      if (previous_it == previous->name.end()) {
        return false;
      }
      const auto previous_index = static_cast<std::size_t>(
        std::distance(previous->name.begin(), previous_it));
      if (previous_index >= previous->position.size() ||
        !std::isfinite(previous->position[previous_index]) ||
        std::abs(state.position[index] - previous->position[previous_index]) > 0.75)
      {
        return false;
      }
    }
  }
  return true;
}

std::string jointVectorString(const std::vector<double> & values)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      out << ',';
    }
    out << values[index];
  }
  out << ']';
  return out.str();
}

}  // namespace

class MotionServer
{
public:
  explicit MotionServer(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    logger_(node_->get_logger()),
    catalog_file_(getOrDeclareParameter<std::string>(
        node_, "catalog_file", defaultCatalogPath())),
    controller_action_name_(getOrDeclareParameter<std::string>(
        node_, "controller_action", "/arm_controller/follow_joint_trajectory")),
    joint_state_topic_(getOrDeclareParameter<std::string>(
        node_, "joint_state_topic", "/joint_states")),
    start_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "start_tolerance_rad", 0.05)),
    joint_state_max_age_sec_(getOrDeclareParameter<double>(
        node_, "joint_state_max_age_sec", 0.50)),
    settled_velocity_rad_sec_(getOrDeclareParameter<double>(
        node_, "settled_velocity_rad_sec", 0.05)),
    controller_wait_timeout_sec_(getOrDeclareParameter<double>(
        node_, "controller_wait_timeout_sec", 3.0)),
    execution_margin_sec_(getOrDeclareParameter<double>(
        node_, "execution_margin_sec", 5.0)),
    final_state_timeout_sec_(getOrDeclareParameter<double>(
        node_, "final_state_timeout_sec", 2.0)),
    cancel_settle_timeout_sec_(getOrDeclareParameter<double>(
        node_, "cancel_settle_timeout_sec", 2.0)),
    trajectory_start_delay_sec_(getOrDeclareParameter<double>(
        node_, "trajectory_start_delay_sec", 0.10)),
    goal_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "goal_position_tolerance_rad", 0.03)),
    path_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "path_position_tolerance_rad", 0.10)),
    pour_path_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "pour_path_position_tolerance_rad", 0.20)),
    wrist_path_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "wrist_path_position_tolerance_rad", 0.35)),
    goal_velocity_tolerance_rad_sec_(getOrDeclareParameter<double>(
        node_, "goal_velocity_tolerance_rad_sec", 0.05)),
    default_speed_scale_(getOrDeclareParameter<double>(
        node_, "default_speed_scale", 1.0)),
    compiler_(std::make_unique<TrajectoryCompiler>(node_))
  {
    validateParameters();
    speed_scale_.store(std::clamp(default_speed_scale_, 0.10, 1.0));

    controller_client_ = rclcpp_action::create_client<FollowTrajectory>(
      node_, controller_action_name_);

    joint_state_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = joint_state_callback_group_;
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        const bool previous_is_fresh = has_joint_state_ &&
        (node_->now() - latest_joint_state_received_).seconds() <= 1.0;
        if (!isPlausibleArmJointState(
          *message, previous_is_fresh ? &latest_joint_state_ : nullptr))
        {
          RCLCPP_WARN_THROTTLE(
            logger_, *node_->get_clock(), 1000,
            "rejected implausible arm joint-state sample; retaining last trusted state");
          return;
        }
        latest_joint_state_ = *message;
        latest_joint_state_received_ = node_->now();
        joint_state_history_.push_back(*message);
        joint_state_history_times_.push_back(latest_joint_state_received_);
        if (joint_state_history_.size() > 11) {
          joint_state_history_.pop_front();
          joint_state_history_times_.pop_front();
        }
        has_joint_state_ = true;
      },
      state_options);

    execute_server_ = rclcpp_action::create_server<ExecuteMotion>(
      node_,
      "/motion/execute",
      std::bind(&MotionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&MotionServer::handleAccepted, this, std::placeholders::_1));

    reload_service_ = node_->create_service<Trigger>(
      "/motion/reload",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        if (busy_.load()) {
          response->success = false;
          response->message = "cannot reload motion catalog while a route is active";
          return;
        }
        response->success = loadAndCompile(response->message);
      });

    list_service_ = node_->create_service<Trigger>(
      "/motion/list_routes",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        std::ostringstream out;
        for (const auto & item : compiled_routes_) {
          const auto & points = item.second.trajectory.points;
          out << item.first
              << " duration=" << item.second.duration_sec
              << "s points=" << points.size()
              << " hash=" << item.second.content_hash;
          if (!points.empty()) {
            out << " start=" << jointVectorString(points.front().positions)
                << " end=" << jointVectorString(points.back().positions);
          }
          out << '\n';
        }
        response->success = !compiled_routes_.empty();
        response->message = out.str();
      });

    stop_service_ = node_->create_service<Trigger>(
      "/motion/stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        cancel_requested_.store(true);
        std::shared_ptr<GoalHandleController> controller_goal;
        {
          std::lock_guard<std::mutex> lock(controller_goal_mutex_);
          controller_goal = controller_goal_;
        }
        if (controller_goal) {
          controller_client_->async_cancel_goal(controller_goal);
        }
        response->success = true;
        response->message = busy_.load() ? "motion stop requested" : "motion server is idle";
      });

    speed_service_ = node_->create_service<SetSpeedScale>(
      "/motion/set_speed_scale",
      [this](
        const std::shared_ptr<SetSpeedScale::Request> request,
        std::shared_ptr<SetSpeedScale::Response> response)
      {
        if (!std::isfinite(request->scale) || request->scale < 0.10 || request->scale > 1.0) {
          response->success = false;
          response->message = "speed scale must be finite and in [0.10, 1.0]";
          response->applied_scale = speed_scale_.load();
          return;
        }
        speed_scale_.store(request->scale);
        response->success = true;
        response->applied_scale = request->scale;
        response->message = "motion speed scale set; applies to the next route goal";
      });

    health_service_ = node_->create_service<Trigger>(
      "/motion/health",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        std::vector<std::string> joint_names;
        {
          std::lock_guard<std::mutex> lock(catalog_mutex_);
          joint_names = catalog_.joint_names;
        }
        std::vector<double> positions;
        std::string state_error;
        const bool state_ok = currentJointState(joint_names, positions, state_error);
        const bool controller_ok = controller_client_->action_server_is_ready();
        response->success = state_ok && controller_ok && !busy_.load();
        std::ostringstream out;
        out << "busy=" << (busy_.load() ? "true" : "false")
            << " controller=" << (controller_ok ? "ready" : "unavailable")
            << " state=" << (state_ok ? "settled" : state_error);
        response->message = out.str();
      });

    std::string compile_message;
    if (!loadAndCompile(compile_message)) {
      throw std::runtime_error(compile_message);
    }

    RCLCPP_INFO(
      logger_,
      "motion server ready catalog=%s controller=%s start_tolerance=%.3frad routes=%zu",
      catalog_file_.c_str(),
      controller_action_name_.c_str(),
      start_tolerance_rad_,
      compiled_routes_.size());
  }

  ~MotionServer()
  {
    cancel_requested_.store(true);
    std::shared_ptr<GoalHandleController> controller_goal;
    {
      std::lock_guard<std::mutex> lock(controller_goal_mutex_);
      controller_goal = controller_goal_;
    }
    if (controller_goal) {
      controller_client_->async_cancel_goal(controller_goal);
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void validateParameters() const
  {
    if (catalog_file_.empty() || controller_action_name_.empty() || joint_state_topic_.empty()) {
      throw std::runtime_error("catalog/controller/joint-state parameters must not be empty");
    }
    if (start_tolerance_rad_ <= 0.0 || start_tolerance_rad_ > 0.25 ||
      joint_state_max_age_sec_ <= 0.0 || settled_velocity_rad_sec_ <= 0.0 ||
      controller_wait_timeout_sec_ <= 0.0 || execution_margin_sec_ < 0.0 ||
      final_state_timeout_sec_ <= 0.0 || cancel_settle_timeout_sec_ <= 0.0 ||
      trajectory_start_delay_sec_ < 0.0 || goal_position_tolerance_rad_ <= 0.0 ||
      path_position_tolerance_rad_ <= 0.0 || pour_path_position_tolerance_rad_ <= 0.0 ||
      wrist_path_position_tolerance_rad_ <= 0.0 ||
      goal_velocity_tolerance_rad_sec_ <= 0.0)
    {
      throw std::runtime_error("motion server safety/timing parameters are invalid");
    }
  }

  bool loadAndCompile(std::string & message)
  {
    try {
      MotionCatalog candidate_catalog = MotionCatalog::loadFromFile(catalog_file_);
      std::map<std::string, CompiledRoute> candidate_routes;
      const auto result = compiler_->compileAll(candidate_catalog, candidate_routes);
      if (!result.success) {
        message = "motion catalog compile rejected; previous cache retained: " + result.message;
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        return false;
      }
      std::string continuous_error;
      if (!addContinuousRoute(
          candidate_routes,
          "outlet_1_grasp_to_spectrometer_place_continuous",
          {"outlet_1_grasp_to_hover_fast", "outlet_1_hover_to_spectrometer_place"},
          continuous_error) ||
        !addContinuousRoute(
          candidate_routes,
          "spectrometer_pick_to_brush_entry_continuous",
          {"spectrometer_pick_to_pick_hover_fast",
            "spectrometer_pick_hover_to_clean_dump",
            "clean_dump_to_pour",
            "clean_dump_pour_shake_once",
            "clean_dump_pour_to_brush_entry"},
          continuous_error) ||
        !addContinuousRoute(
          candidate_routes,
          "brush_entry_to_outlet_1_return_continuous",
          {"brush_entry_to_clean_dump_pour",
            "clean_dump_pour_to_clean_dump",
            "clean_dump_to_clean_hover",
            "clean_hover_to_outlet_1_return"},
          continuous_error))
      {
        message = "continuous route composition rejected: " + continuous_error;
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        return false;
      }
      {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        catalog_ = std::move(candidate_catalog);
        compiled_routes_ = std::move(candidate_routes);
      }
      message = result.message;
      RCLCPP_INFO(logger_, "%s", message.c_str());
      return true;
    } catch (const std::exception & error) {
      message = std::string("motion catalog load/compile failed; previous cache retained: ") +
        error.what();
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return false;
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMotion::Goal> goal)
  {
    if (!goal || goal->route_name.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->speed_scale < 0.0 || goal->speed_scale > 1.0 ||
      !std::isfinite(goal->speed_scale))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    {
      std::lock_guard<std::mutex> lock(catalog_mutex_);
      if (compiled_routes_.count(goal->route_name) == 0) {
        RCLCPP_WARN(logger_, "reject unknown route '%s'", goal->route_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(logger_, "reject route '%s': motion server busy", goal->route_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleMotion>)
  {
    cancel_requested_.store(true);
    std::shared_ptr<GoalHandleController> controller_goal;
    {
      std::lock_guard<std::mutex> lock(controller_goal_mutex_);
      controller_goal = controller_goal_;
    }
    if (controller_goal) {
      try {
        controller_client_->async_cancel_goal(controller_goal);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          logger_, "controller goal was already terminal while cancelling: %s", error.what());
      }
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleMotion> goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  bool currentJointState(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions,
    std::string & error)
  {
    sensor_msgs::msg::JointState state;
    std::deque<sensor_msgs::msg::JointState> history;
    std::deque<rclcpp::Time> history_times;
    rclcpp::Time received;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!has_joint_state_) {
        error = "no joint state has been received";
        return false;
      }
      state = latest_joint_state_;
      history = joint_state_history_;
      history_times = joint_state_history_times_;
      received = latest_joint_state_received_;
    }

    const double age_sec = (node_->now() - received).seconds();
    if (age_sec < 0.0 || age_sec > joint_state_max_age_sec_) {
      std::ostringstream out;
      out << "joint state is stale: age=" << age_sec
          << "s max=" << joint_state_max_age_sec_ << "s";
      error = out.str();
      return false;
    }

    double maximum_velocity = maxAbsVelocity(state, joint_names);
    if (history.size() >= 5 && history.size() == history_times.size()) {
      std::vector<double> sample_times;
      std::vector<std::vector<double>> ordered_positions;
      sample_times.reserve(history.size());
      ordered_positions.reserve(history.size());
      const double first_time = history_times.front().seconds();
      for (std::size_t sample_index = 0; sample_index < history.size(); ++sample_index) {
        std::vector<double> ordered;
        ordered.reserve(joint_names.size());
        for (const auto & joint_name : joint_names) {
          const auto it = std::find(
            history[sample_index].name.begin(), history[sample_index].name.end(), joint_name);
          if (it == history[sample_index].name.end()) {
            ordered.clear();
            break;
          }
          const auto index = static_cast<std::size_t>(
            std::distance(history[sample_index].name.begin(), it));
          if (index >= history[sample_index].position.size()) {
            ordered.clear();
            break;
          }
          ordered.push_back(history[sample_index].position[index]);
        }
        if (ordered.size() == joint_names.size()) {
          sample_times.push_back(history_times[sample_index].seconds() - first_time);
          ordered_positions.push_back(std::move(ordered));
        }
      }
      if (ordered_positions.size() >= 5) {
        maximum_velocity = maxAbsPositionSlope(sample_times, ordered_positions);
      }
    }
    if (!std::isfinite(maximum_velocity)) {
      error = "joint state does not contain finite velocities for every arm joint";
      return false;
    }
    if (maximum_velocity > settled_velocity_rad_sec_) {
      std::ostringstream out;
      out << "arm is not settled: max velocity=" << maximum_velocity
          << "rad/s limit=" << settled_velocity_rad_sec_ << "rad/s";
      error = out.str();
      return false;
    }

    positions.clear();
    positions.reserve(joint_names.size());
    for (const auto & name : joint_names) {
      const auto it = std::find(state.name.begin(), state.name.end(), name);
      if (it == state.name.end()) {
        error = "joint state is missing arm joint '" + name + "'";
        return false;
      }
      const auto index = static_cast<std::size_t>(std::distance(state.name.begin(), it));
      if (index >= state.position.size() || !std::isfinite(state.position[index])) {
        error = "joint state has invalid position for arm joint '" + name + "'";
        return false;
      }
      positions.push_back(state.position[index]);
    }
    return true;
  }

  bool startMatches(
    const CompiledRoute & route,
    const std::vector<double> & current,
    std::string & error) const
  {
    if (current.size() != route.start_joints.size()) {
      error = "current/start joint dimensions differ";
      return false;
    }
    double maximum = 0.0;
    std::size_t maximum_index = 0;
    for (std::size_t index = 0; index < current.size(); ++index) {
      const double difference = std::abs(current[index] - route.start_joints[index]);
      if (difference > maximum) {
        maximum = difference;
        maximum_index = index;
      }
    }
    if (maximum > start_tolerance_rad_) {
      std::ostringstream out;
      out << "route start mismatch at " << route.trajectory.joint_names[maximum_index]
          << ": error=" << maximum << "rad limit=" << start_tolerance_rad_ << "rad";
      error = out.str();
      return false;
    }
    return true;
  }

  bool waitForJointTarget(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target,
    double tolerance,
    double timeout_sec,
    std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    do {
      std::vector<double> current;
      std::string state_error;
      if (currentJointState(joint_names, current, state_error)) {
        if (current.size() != target.size()) {
          error = "joint target dimensions differ";
          return false;
        }
        double maximum_error = 0.0;
        std::size_t maximum_index = 0;
        for (std::size_t index = 0; index < current.size(); ++index) {
          const double difference = std::abs(current[index] - target[index]);
          if (difference > maximum_error) {
            maximum_error = difference;
            maximum_index = index;
          }
        }
        if (maximum_error <= tolerance) {
          return true;
        }
        std::ostringstream out;
        out << "final position error at " << joint_names[maximum_index]
            << "=" << maximum_error << "rad limit=" << tolerance << "rad";
        error = out.str();
      } else {
        error = state_error;
      }
      std::this_thread::sleep_for(20ms);
    } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
    return false;
  }

  bool waitForArmSettled(
    const std::vector<std::string> & joint_names,
    std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(cancel_settle_timeout_sec_);
    do {
      std::vector<double> positions;
      std::string state_error;
      if (currentJointState(joint_names, positions, state_error)) {
        return true;
      }
      error = state_error;
      std::this_thread::sleep_for(20ms);
    } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
    return false;
  }

  bool cancelControllerGoalAndWait(
    const std::shared_ptr<GoalHandleController> & controller_goal,
    const std::vector<std::string> & joint_names,
    std::string & error)
  {
    try {
      const auto cancel_future = controller_client_->async_cancel_goal(controller_goal);
      if (cancel_future.wait_for(std::chrono::duration<double>(controller_wait_timeout_sec_)) !=
        std::future_status::ready)
      {
        error = "controller cancel acknowledgement timeout";
      }
    } catch (const std::exception & exception) {
      // A result can become terminal between the result/cancel checks.  That
      // race is normal and must never terminate the long-running server.
      error = std::string("controller goal already terminal: ") + exception.what();
    }
    return waitForArmSettled(joint_names, error);
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandleMotion> & goal_handle,
    const CompiledRoute & route,
    double progress)
  {
    auto feedback = std::make_shared<ExecuteMotion::Feedback>();
    feedback->route_name = route.name;
    feedback->segment_count = static_cast<std::uint32_t>(route.segment_names.size());
    const std::size_t segment_count = route.segment_names.size();
    const std::size_t segment_index = segment_count == 0 ? 0 :
      std::min(segment_count - 1, static_cast<std::size_t>(progress * segment_count));
    feedback->segment_index = static_cast<std::uint32_t>(segment_index + 1);
    feedback->segment_name = segment_count == 0 ? "" : route.segment_names[segment_index];
    feedback->progress = static_cast<float>(std::clamp(progress, 0.0, 1.0));
    goal_handle->publish_feedback(feedback);
  }

  void finish(
    const std::shared_ptr<GoalHandleMotion> & goal_handle,
    bool success,
    std::int32_t error_code,
    const std::string & message,
    const std::chrono::steady_clock::time_point & start)
  {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;
    result->elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

    // Clear the internal active state before publishing the terminal action result.
    // A client is allowed to submit the next route as soon as it receives that result;
    // publishing first created a small race where the next valid stage was rejected busy.
    {
      std::lock_guard<std::mutex> lock(controller_goal_mutex_);
      controller_goal_.reset();
    }
    busy_.store(false);
    try {
      if (success) {
        goal_handle->succeed(result);
      } else if (
        error_code == ExecuteMotion::Result::ERROR_CANCELLED &&
        goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
      } else {
        // A controller goal can be cancelled independently (for example when
        // superseded). The parent route is still EXECUTING in that case, so
        // CANCELED would be an invalid action-state transition.
        goal_handle->abort(result);
      }
    } catch (const rclcpp::exceptions::RCLError & error) {
      RCLCPP_ERROR(
        logger_, "failed to publish terminal motion result without crashing: %s", error.what());
    }
    RCLCPP_INFO(
      logger_,
      "motion result success=%s code=%d elapsed=%.3fs message=%s",
      success ? "true" : "false",
      error_code,
      result->elapsed_sec,
      message.c_str());
  }

  void execute(const std::shared_ptr<GoalHandleMotion> goal_handle)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    CompiledRoute route;
    {
      std::lock_guard<std::mutex> lock(catalog_mutex_);
      const auto it = compiled_routes_.find(goal->route_name);
      if (it == compiled_routes_.end()) {
        finish(
          goal_handle, false, ExecuteMotion::Result::ERROR_ROUTE_NOT_FOUND,
          "route disappeared from compiled cache", started);
        return;
      }
      route = it->second;
    }

    const double requested_scale = goal->speed_scale > 0.0 ?
      goal->speed_scale : speed_scale_.load();
    if (!std::isfinite(requested_scale) || requested_scale < 0.10 || requested_scale > 1.0) {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_INVALID_GOAL,
        "effective speed scale must be in [0.10, 1.0]", started);
      return;
    }

    if (goal->dry_run) {
      publishFeedback(goal_handle, route, 1.0);
      std::ostringstream out;
      out << "dry-run valid route=" << route.name
          << " hash=" << route.content_hash
          << " duration=" << route.duration_sec / requested_scale << "s";
      finish(goal_handle, true, ExecuteMotion::Result::ERROR_NONE, out.str(), started);
      return;
    }

    std::vector<double> current;
    std::string state_error;
    if (!currentJointState(route.trajectory.joint_names, current, state_error)) {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_STATE_UNAVAILABLE,
        state_error, started);
      return;
    }
    if (!startMatches(route, current, state_error)) {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_START_MISMATCH,
        state_error, started);
      return;
    }

    if (!controller_client_->wait_for_action_server(
        std::chrono::duration<double>(controller_wait_timeout_sec_)))
    {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_UNAVAILABLE,
        "arm trajectory controller action is unavailable", started);
      return;
    }

    FollowTrajectory::Goal controller_request;
    try {
      controller_request.trajectory = scaleTrajectory(route.trajectory, requested_scale);
      const double start_correction = alignTrajectoryStart(
        controller_request.trajectory, current);
      if (start_correction > 1e-3) {
        RCLCPP_INFO(
          logger_, "route '%s' starts from measured state; max correction=%.4frad",
          route.name.c_str(), start_correction);
      }
    } catch (const std::exception & error) {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_INVALID_GOAL,
        error.what(), started);
      return;
    }
    controller_request.trajectory.header.stamp =
      node_->now() + rclcpp::Duration::from_seconds(trajectory_start_delay_sec_);
    const auto execution_margin_ns = static_cast<int64_t>(
      std::llround(execution_margin_sec_ * 1e9));
    controller_request.goal_time_tolerance.sec = static_cast<int32_t>(
      execution_margin_ns / 1000000000LL);
    controller_request.goal_time_tolerance.nanosec = static_cast<uint32_t>(
      execution_margin_ns % 1000000000LL);
    for (const auto & joint_name : controller_request.trajectory.joint_names) {
      control_msgs::msg::JointTolerance path_tolerance;
      path_tolerance.name = joint_name;
      const double route_path_tolerance =
        route.name == "clean_dump_to_pour" ||
        route.name == "clean_dump_pour_shake_once" ||
        route.name == "clean_dump_pour_to_brush_entry" ||
        route.name == "spectrometer_pick_to_brush_entry_continuous" ?
        pour_path_position_tolerance_rad_ : path_position_tolerance_rad_;
      // The process only requires the wrist to reach its final angle. Keep strict path
      // protection on joints 1-5, while allowing joint6 to track through transient lag.
      path_tolerance.position = joint_name == "joint6" ?
        std::max(route_path_tolerance, wrist_path_position_tolerance_rad_) :
        route_path_tolerance;
      controller_request.path_tolerance.push_back(path_tolerance);

      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = joint_name;
      tolerance.position = goal_position_tolerance_rad_;
      tolerance.velocity = goal_velocity_tolerance_rad_sec_;
      controller_request.goal_tolerance.push_back(tolerance);
    }

    const double expected_duration = trajectoryDurationSec(controller_request.trajectory);
    const auto send_future = controller_client_->async_send_goal(controller_request);
    if (send_future.wait_for(std::chrono::duration<double>(controller_wait_timeout_sec_)) !=
      std::future_status::ready)
    {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_UNAVAILABLE,
        "timed out sending trajectory to controller", started);
      return;
    }
    auto controller_goal = send_future.get();
    if (!controller_goal) {
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_REJECTED,
        "arm trajectory controller rejected the goal", started);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(controller_goal_mutex_);
      controller_goal_ = controller_goal;
    }

    const auto result_future = controller_client_->async_get_result(controller_goal);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(expected_duration + execution_margin_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (cancel_requested_.load() || goal_handle->is_canceling()) {
        std::string settle_error;
        const bool settled = cancelControllerGoalAndWait(
          controller_goal, route.trajectory.joint_names, settle_error);
        finish(
          goal_handle, false, ExecuteMotion::Result::ERROR_CANCELLED,
          settled ? "motion route cancelled and arm settled" :
          "motion route cancelled; arm settle not confirmed: " + settle_error,
          started);
        return;
      }
      if (result_future.wait_for(50ms) == std::future_status::ready) {
        const auto wrapped = result_future.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result &&
          wrapped.result->error_code == FollowTrajectory::Result::SUCCESSFUL)
        {
          std::string final_error;
          if (!waitForJointTarget(
              route.trajectory.joint_names,
              route.end_joints,
              goal_position_tolerance_rad_,
              final_state_timeout_sec_,
              final_error))
          {
            finish(
              goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_FAILED,
              "controller reported success but final state was not confirmed: " + final_error,
              started);
            return;
          }
          publishFeedback(goal_handle, route, 1.0);
          finish(
            goal_handle, true, ExecuteMotion::Result::ERROR_NONE,
            "motion route completed: " + route.name, started);
          return;
        }
        if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
          std::string settle_error;
          const bool settled = waitForArmSettled(route.trajectory.joint_names, settle_error);
          finish(
            goal_handle, false, ExecuteMotion::Result::ERROR_CANCELLED,
            settled ? "arm trajectory controller cancelled the route and arm settled" :
            "arm trajectory controller cancelled the route; settle not confirmed: " +
            settle_error,
            started);
          return;
        }
        std::string controller_message = wrapped.result ?
          wrapped.result->error_string : "no controller result";
        std::string settle_error;
        if (!waitForArmSettled(route.trajectory.joint_names, settle_error)) {
          controller_message += "; arm settle not confirmed: " + settle_error;
        }
        finish(
          goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_FAILED,
          "arm trajectory controller failed: " + controller_message, started);
        return;
      }

      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      publishFeedback(
        goal_handle,
        route,
        expected_duration > 0.0 ? std::min(0.99, elapsed / expected_duration) : 0.0);
    }

    std::string settle_error;
    cancelControllerGoalAndWait(
      controller_goal, route.trajectory.joint_names, settle_error);
    finish(
      goal_handle, false, ExecuteMotion::Result::ERROR_TIMEOUT,
      "motion route exceeded controller execution timeout; settle=" + settle_error, started);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string catalog_file_;
  std::string controller_action_name_;
  std::string joint_state_topic_;
  double start_tolerance_rad_;
  double joint_state_max_age_sec_;
  double settled_velocity_rad_sec_;
  double controller_wait_timeout_sec_;
  double execution_margin_sec_;
  double final_state_timeout_sec_;
  double cancel_settle_timeout_sec_;
  double trajectory_start_delay_sec_;
  double goal_position_tolerance_rad_;
  double path_position_tolerance_rad_;
  double pour_path_position_tolerance_rad_;
  double wrist_path_position_tolerance_rad_;
  double goal_velocity_tolerance_rad_sec_;
  double default_speed_scale_;

  std::unique_ptr<TrajectoryCompiler> compiler_;
  MotionCatalog catalog_;
  std::map<std::string, CompiledRoute> compiled_routes_;
  std::mutex catalog_mutex_;

  rclcpp_action::Client<FollowTrajectory>::SharedPtr controller_client_;
  rclcpp_action::Server<ExecuteMotion>::SharedPtr execute_server_;
  rclcpp::Service<Trigger>::SharedPtr reload_service_;
  rclcpp::Service<Trigger>::SharedPtr list_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<SetSpeedScale>::SharedPtr speed_service_;
  rclcpp::Service<Trigger>::SharedPtr health_service_;

  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::deque<sensor_msgs::msg::JointState> joint_state_history_;
  std::deque<rclcpp::Time> joint_state_history_times_;
  rclcpp::Time latest_joint_state_received_;
  bool has_joint_state_{false};
  std::mutex joint_state_mutex_;

  std::shared_ptr<GoalHandleController> controller_goal_;
  std::mutex controller_goal_mutex_;
  std::atomic_bool busy_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic<double> speed_scale_{1.0};
  std::thread worker_;
  std::mutex worker_mutex_;
};

}  // namespace panthera_motion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "panthera_motion_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  try {
    auto motion_server = std::make_shared<panthera_motion::MotionServer>(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    motion_server.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "motion server startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
