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
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <panthera_interfaces/action/execute_motion.hpp>
#include <panthera_interfaces/srv/stage_jog.hpp>
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
using StageJog = panthera_interfaces::srv::StageJog;

constexpr char kStagedJogPrefix[] = "__debug_jog_";
constexpr double kTeachJogMinimumDurationSec = 0.45;
constexpr double kTeachJogIkTimeoutSec = 0.02;
constexpr int kTeachJogIkAttempts = 2;
constexpr double kMeasuredJointLimitToleranceRad = 0.005;

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

std::array<double, 3> matrixToRpy(const Eigen::Matrix3d & rotation)
{
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return {roll, pitch, yaw};
}

bool aliasContinuousRoute(
  std::map<std::string, CompiledRoute> & routes,
  const std::string & name,
  const std::string & source_name,
  std::string & error)
{
  const auto source = routes.find(source_name);
  if (source == routes.end()) {
    error = "continuous route '" + name + "' is missing source '" + source_name + "'";
    return false;
  }
  auto route = source->second;
  route.name = name;
  routes.emplace(name, std::move(route));
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
        node_, "path_position_tolerance_rad", 0.15)),
    pour_path_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "pour_path_position_tolerance_rad", 0.20)),
    wrist_path_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "wrist_path_position_tolerance_rad", 0.45)),
    goal_velocity_tolerance_rad_sec_(getOrDeclareParameter<double>(
        node_, "goal_velocity_tolerance_rad_sec", 0.05)),
    process_stage_max_translation_m_(getOrDeclareParameter<double>(
        node_, "process_stage_max_translation_m", 0.16)),
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

    stage_jog_service_ = node_->create_service<StageJog>(
      "/motion/stage_jog",
      [this](
        const std::shared_ptr<StageJog::Request> request,
        std::shared_ptr<StageJog::Response> response)
      {
        stageJog(*request, *response);
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
            << " state=" << (state_ok ? "settled" : state_error)
            << " mode=" << (commissioning_only_.load() ? "commissioning_only" : "production");
        response->message = out.str();
      });

    std::string compile_message;
    if (!loadAndCompile(compile_message)) {
      const std::string strict_error = compile_message;
      if (!loadCommissioningCatalog(compile_message)) {
        throw std::runtime_error(strict_error + "; " + compile_message);
      }
      RCLCPP_WARN(logger_, "%s; %s", strict_error.c_str(), compile_message.c_str());
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
      goal_velocity_tolerance_rad_sec_ <= 0.0 ||
      process_stage_max_translation_m_ < 0.021 ||
      process_stage_max_translation_m_ > 0.25)
    {
      throw std::runtime_error("motion server safety/timing parameters are invalid");
    }
  }

  bool loadAndCompile(std::string & message)
  {
    try {
      MotionCatalog candidate_catalog = MotionCatalog::loadFromFile(catalog_file_);
      std::map<std::string, CompiledRoute> candidate_routes;
      ValidationResult result;
      {
        std::lock_guard<std::mutex> lock(compiler_mutex_);
        result = compiler_->compileAll(candidate_catalog, candidate_routes);
      }
      if (!result.success) {
        message = "motion catalog compile rejected; previous cache retained: " + result.message;
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        return false;
      }
      std::string continuous_error;
      if (!aliasContinuousRoute(
          candidate_routes,
          "outlet_1_grasp_to_spectrometer_place_continuous",
          "outlet_1_grasp_to_spectrometer_place",
          continuous_error) ||
        !aliasContinuousRoute(
          candidate_routes,
          "spectrometer_pick_to_brush_entry_continuous",
          "spectrometer_pick_to_brush_entry_smooth",
          continuous_error) ||
        !aliasContinuousRoute(
          candidate_routes,
          "spectrometer_sensor_pick_hover_to_brush_entry_recovery",
          "spectrometer_pick_hover_to_brush_entry_smooth",
          continuous_error) ||
        !aliasContinuousRoute(
          candidate_routes,
          "brush_entry_to_outlet_1_return_continuous",
          "brush_entry_to_outlet_1_return_smooth",
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
      commissioning_only_.store(false);
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

  bool loadCommissioningCatalog(std::string & message)
  {
    try {
      MotionCatalog candidate_catalog = MotionCatalog::loadFromFile(catalog_file_);
      std::map<std::string, CompiledRoute> candidate_routes;
      std::vector<std::string> skipped;
      for (const auto & item : candidate_catalog.routes) {
        const auto & route = item.second;
        const bool commissioning_route =
          route.name.rfind("debug_", 0) == 0 ||
          route.name == "home_to_safe_center" ||
          route.name == "safe_center_to_home";
        if (!route.enabled || !commissioning_route) {
          continue;
        }
        CompiledRoute compiled;
        ValidationResult result;
        {
          std::lock_guard<std::mutex> lock(compiler_mutex_);
          result = compiler_->compileRoute(candidate_catalog, route, compiled);
        }
        if (!result.success) {
          skipped.push_back(route.name + ": " + result.message);
          continue;
        }
        candidate_routes.emplace(route.name, std::move(compiled));
      }
      if (candidate_routes.count("home_to_safe_center") == 0 ||
        candidate_routes.count("safe_center_to_home") == 0)
      {
        message = "commissioning fallback rejected: Home/safe routes are unavailable";
        return false;
      }
      {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        catalog_ = std::move(candidate_catalog);
        compiled_routes_ = std::move(candidate_routes);
      }
      commissioning_only_.store(true);
      std::ostringstream out;
      out << "commissioning-only catalog loaded routes=" << compiled_routes_.size()
          << " skipped=" << skipped.size();
      if (!skipped.empty()) {
        out << " first=" << skipped.front();
      }
      message = out.str();
      return true;
    } catch (const std::exception & error) {
      message = std::string("commissioning fallback failed: ") + error.what();
      return false;
    }
  }

  void stageJog(const StageJog::Request & request, StageJog::Response & response)
  {
    if (commissioning_only_.load() && request.process_profile) {
      response.success = false;
      response.message = "production process moves are disabled until all routes compile";
      return;
    }
    const double max_translation_m =
      request.process_profile ? process_stage_max_translation_m_ : 0.021;
    constexpr double max_rotation_rad = 0.17453292519943295;
    const std::array<double, 6> deltas{
      request.delta_x_m, request.delta_y_m, request.delta_z_m,
      request.delta_roll_rad, request.delta_pitch_rad, request.delta_yaw_rad};
    const std::array<double, 6> absolute_target{
      request.absolute_target_xyz_m[0], request.absolute_target_xyz_m[1],
      request.absolute_target_xyz_m[2], request.absolute_target_rpy_rad[0],
      request.absolute_target_rpy_rad[1], request.absolute_target_rpy_rad[2]};
    const std::array<double, 3> point_offset{
      request.point_offset_xyz_m[0],
      request.point_offset_xyz_m[1],
      request.point_offset_xyz_m[2]};
    const bool uses_catalog_point = !request.base_point_name.empty();
    const bool uses_catalog_route = !request.base_route_name.empty();
    if (busy_.load() ||
      !std::all_of(deltas.begin(), deltas.end(), [](double value) {return std::isfinite(value);}) ||
      !std::all_of(
        point_offset.begin(), point_offset.end(),
        [](double value) {return std::isfinite(value);}) ||
      (request.use_absolute_target &&
      !std::all_of(
        absolute_target.begin(), absolute_target.end(),
        [](double value) {return std::isfinite(value);})))
    {
      response.success = false;
      response.message = busy_.load() ? "motion server is busy" : "Cartesian target must be finite";
      return;
    }
    if ((uses_catalog_point && uses_catalog_route) ||
      ((uses_catalog_point || uses_catalog_route) && request.use_absolute_target))
    {
      response.success = false;
      response.message = "catalog point, catalog route and absolute target are mutually exclusive";
      return;
    }
    const auto nonzero = std::count_if(
      deltas.begin(), deltas.end(), [](double value) {return std::abs(value) > 1e-9;});
    if (!uses_catalog_point && !uses_catalog_route &&
      !request.use_absolute_target && nonzero == 0)
    {
      response.success = false;
      response.message = "at least one Cartesian delta must be non-zero";
      return;
    }
    if (uses_catalog_route &&
      (!request.process_profile || nonzero != 0 || request.offset_point_names.empty()))
    {
      response.success = false;
      response.message =
        "catalog route staging requires process_profile, offset points and zero jog deltas";
      return;
    }

    MotionCatalog candidate;
    {
      std::lock_guard<std::mutex> lock(catalog_mutex_);
      candidate = catalog_;
    }
    std::vector<double> current;
    std::string state_error;
    if (!currentJointState(candidate.joint_names, current, state_error)) {
      response.success = false;
      response.message = "cannot stage jog: " + state_error;
      return;
    }

    PoseDefinition measured_pose;
    ValidationResult result;
    {
      std::lock_guard<std::mutex> lock(compiler_mutex_);
      result = compiler_->normalizeMeasuredJoints(
        candidate, current, kMeasuredJointLimitToleranceRad);
      if (result.success) {
        result = compiler_->forwardKinematics(candidate, current, measured_pose);
      }
    }
    if (!result.success) {
      response.success = false;
      response.message = "cannot stage jog FK: " + result.message;
      return;
    }

    const bool relative_teach_jog =
      !request.process_profile && !uses_catalog_point && !uses_catalog_route &&
      !request.use_absolute_target;
    std::vector<double> holding_offset(current.size(), 0.0);
    if (relative_teach_jog) {
      std::vector<double> commanded;
      {
        std::lock_guard<std::mutex> lock(commanded_state_mutex_);
        commanded = last_commanded_joints_;
      }
      std::vector<double> holding_start;
      result = selectHoldingCommandStart(
        current, commanded, start_tolerance_rad_, holding_start);
      if (!result.success) {
        response.success = false;
        response.message = "cannot preserve MIT holding command for single-axis jog: " +
          result.message;
        return;
      }
      for (std::size_t index = 0; index < current.size(); ++index) {
        holding_offset[index] = holding_start[index] - current[index];
      }
    }

    if (uses_catalog_route) {
      const auto source_route = candidate.findRoute(request.base_route_name);
      if (!source_route) {
        response.success = false;
        response.message = "catalog route is missing: " + request.base_route_name;
        return;
      }
      if (Eigen::Vector3d(point_offset[0], point_offset[1], point_offset[2]).norm() >
        process_stage_max_translation_m_ + 1e-12)
      {
        response.success = false;
        response.message = "catalog route offset exceeds process translation limit";
        return;
      }

      const auto route_id = jog_sequence_.fetch_add(1);
      const std::string route_name = kStagedJogPrefix + std::to_string(route_id);
      RouteDefinition route = *source_route;
      route.name = route_name;

      const std::set<std::string> offset_names(
        request.offset_point_names.begin(), request.offset_point_names.end());
      std::set<std::string> used_offset_names;
      const bool offsets_start = offset_names.count(source_route->start) != 0;
      Eigen::Vector3d offset_start_residual = Eigen::Vector3d::Zero();
      if (offsets_start) {
        const auto source_start = candidate.findPoint(source_route->start);
        if (!source_start || !source_start->pose) {
          response.success = false;
          response.message = "offset route start is missing Cartesian pose";
          return;
        }
        const Eigen::Vector3d expected_start(
          source_start->pose->xyz[0] + point_offset[0],
          source_start->pose->xyz[1] + point_offset[1],
          source_start->pose->xyz[2] + point_offset[2]);
        offset_start_residual =
          Eigen::Vector3d(
          measured_pose.xyz[0], measured_pose.xyz[1], measured_pose.xyz[2]) -
          expected_start;
        if (offset_start_residual.norm() > 0.015) {
          response.success = false;
          response.message = "measured TCP does not match laser-adjusted route start";
          return;
        }
        used_offset_names.insert(source_route->start);
      } else {
        std::vector<double> expected_start;
        {
          std::lock_guard<std::mutex> lock(catalog_mutex_);
          const auto compiled_source = compiled_routes_.find(request.base_route_name);
          if (compiled_source == compiled_routes_.end()) {
            response.success = false;
            response.message = "compiled catalog route is missing: " + request.base_route_name;
            return;
          }
          expected_start = compiled_source->second.start_joints;
        }
        if (expected_start.size() != current.size()) {
          response.success = false;
          response.message = "compiled catalog route start has invalid joint count";
          return;
        }
        double max_error = 0.0;
        for (std::size_t index = 0; index < current.size(); ++index) {
          max_error = std::max(max_error, std::abs(current[index] - expected_start[index]));
        }
        if (max_error > start_tolerance_rad_) {
          response.success = false;
          std::ostringstream out;
          out << "catalog route start mismatch: max_error=" << max_error
              << "rad limit=" << start_tolerance_rad_ << "rad";
          response.message = out.str();
          return;
        }
      }

      PointDefinition measured_start;
      measured_start.name = route_name + "_start";
      measured_start.joints = current;
      candidate.points[measured_start.name] = measured_start;
      route.start = measured_start.name;
      std::string previous_point = measured_start.name;
      for (auto & segment : route.segments) {
        if (offset_names.count(segment.to) != 0) {
          const auto source_point = candidate.findPoint(segment.to);
          if (!source_point || !source_point->pose) {
            response.success = false;
            response.message = "offset route point is missing Cartesian pose: " + segment.to;
            return;
          }
          PointDefinition adjusted = *source_point;
          const std::string source_name = segment.to;
          adjusted.name = route_name + "_" + source_name;
          adjusted.pose->xyz[0] += point_offset[0];
          adjusted.pose->xyz[1] += point_offset[1];
          adjusted.pose->xyz[2] += point_offset[2];
          if (offsets_start) {
            adjusted.pose->xyz[0] += offset_start_residual.x();
            adjusted.pose->xyz[1] += offset_start_residual.y();
            adjusted.pose->xyz[2] += offset_start_residual.z();
          }
          adjusted.ik_seed = previous_point;
          candidate.points[adjusted.name] = adjusted;
          segment.to = adjusted.name;
          used_offset_names.insert(source_name);
        }
        previous_point = segment.to;
      }
      if (used_offset_names != offset_names) {
        response.success = false;
        response.message = "offset point list contains a point not used by the route";
        return;
      }

      const auto compile_started = std::chrono::steady_clock::now();
      CompiledRoute compiled;
      {
        std::lock_guard<std::mutex> lock(compiler_mutex_);
        result = compiler_->compileRoute(candidate, route, compiled);
      }
      if (!result.success) {
        response.success = false;
        response.message = "offset route validation failed: " + result.message;
        return;
      }
      RCLCPP_INFO(
        logger_,
        "staged continuous offset route=%s source=%s offset=[%.4f,%.4f,%.4f] "
        "duration=%.3fs compile=%.3fs",
        route_name.c_str(), request.base_route_name.c_str(),
        point_offset[0], point_offset[1], point_offset[2],
        compiled.duration_sec,
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - compile_started).count());
      const auto final_point = candidate.findPoint(route.segments.back().to);
      {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        for (auto it = compiled_routes_.begin(); it != compiled_routes_.end(); ) {
          if (it->first.rfind(kStagedJogPrefix, 0) == 0) {
            it = compiled_routes_.erase(it);
          } else {
            ++it;
          }
        }
        compiled_routes_[route_name] = std::move(compiled);
      }
      response.success = true;
      response.message = "continuous laser-adjusted process route validated and staged";
      response.route_name = route_name;
      if (final_point && final_point->pose) {
        response.target_xyz_m = final_point->pose->xyz;
        response.target_rpy_rad = final_point->pose->rpy;
      }
      return;
    }

    const Eigen::Matrix3d measured_rotation =
      (Eigen::AngleAxisd(measured_pose.rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(measured_pose.rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(measured_pose.rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    if (uses_catalog_point) {
      const auto point = candidate.findPoint(request.base_point_name);
      if (!point || !point->pose) {
        response.success = false;
        response.message =
          "catalog point is missing or has no Cartesian pose: " + request.base_point_name;
        return;
      }
      target.translation() = Eigen::Vector3d(
        point->pose->xyz[0] + point_offset[0],
        point->pose->xyz[1] + point_offset[1],
        point->pose->xyz[2] + point_offset[2]);
      target.linear() = request.process_profile ? measured_rotation :
        (Eigen::AngleAxisd(point->pose->rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(point->pose->rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(point->pose->rpy[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
    } else if (request.use_absolute_target) {
      target.translation() = Eigen::Vector3d(
        request.absolute_target_xyz_m[0], request.absolute_target_xyz_m[1],
        request.absolute_target_xyz_m[2]);
      target.linear() =
        (Eigen::AngleAxisd(request.absolute_target_rpy_rad[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(request.absolute_target_rpy_rad[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(request.absolute_target_rpy_rad[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
    } else {
      const Eigen::Matrix3d base_delta =
        (Eigen::AngleAxisd(request.delta_yaw_rad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(request.delta_pitch_rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(request.delta_roll_rad, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
      target.translation() = Eigen::Vector3d(
        measured_pose.xyz[0] + request.delta_x_m,
        measured_pose.xyz[1] + request.delta_y_m,
        measured_pose.xyz[2] + request.delta_z_m);
      target.linear() = base_delta * measured_rotation;
    }
    const double translation_norm =
      (target.translation() -
      Eigen::Vector3d(measured_pose.xyz[0], measured_pose.xyz[1], measured_pose.xyz[2])).norm();
    const double rotation_angle = Eigen::AngleAxisd(target.linear() * measured_rotation.transpose())
      .angle();
    if (translation_norm > max_translation_m + 1e-12 ||
      rotation_angle > max_rotation_rad + 1e-12)
    {
      response.success = false;
      std::ostringstream out;
      out << "Cartesian target exceeds "
          << static_cast<int>(std::lround(max_translation_m * 1000.0))
          << "mm or 10deg from measured TCP";
      response.message = out.str();
      return;
    }
    const auto target_rpy = matrixToRpy(target.rotation());

    const auto route_id = jog_sequence_.fetch_add(1);
    const std::string route_name = kStagedJogPrefix + std::to_string(route_id);
    PointDefinition start;
    start.name = route_name + "_start";
    start.joints = current;
    PointDefinition goal;
    goal.name = route_name + "_goal";
    goal.ik_seed = start.name;
    goal.pose = PoseDefinition{
      {target.translation().x(), target.translation().y(), target.translation().z()},
      target_rpy};
    candidate.points[start.name] = start;
    candidate.points[goal.name] = goal;

    RouteDefinition route;
    route.name = route_name;
    route.start = start.name;
    route.velocity_scale = request.process_profile ? 0.80 : 0.45;
    route.acceleration_scale = request.process_profile ? 0.60 : 0.30;
    SegmentDefinition segment;
    segment.name = "cartesian_teach_jog";
    segment.type = SegmentType::LINEAR;
    segment.to = goal.name;
    segment.cartesian_step_m = request.process_profile ? 0.005 : 0.002;
    segment.max_joint_jump_rad = request.process_profile ? 0.20 : 0.15;
    segment.constraints.keep_orientation = request.process_profile;
    route.segments.push_back(segment);

    if (!request.process_profile) {
      candidate.defaults.ik_timeout_sec = std::min(
        candidate.defaults.ik_timeout_sec, kTeachJogIkTimeoutSec);
      candidate.defaults.ik_attempts = std::min(
        candidate.defaults.ik_attempts, kTeachJogIkAttempts);
    }
    const auto compile_started = std::chrono::steady_clock::now();
    CompiledRoute compiled;
    {
      std::lock_guard<std::mutex> lock(compiler_mutex_);
      result = compiler_->compileRoute(candidate, route, compiled);
    }
    if (!result.success) {
      response.success = false;
      response.message = "jog validation failed: " + result.message;
      return;
    }
    if (!request.process_profile &&
      compiled.duration_sec < kTeachJogMinimumDurationSec)
    {
      compiled.trajectory = scaleTrajectory(
        compiled.trajectory,
        compiled.duration_sec / kTeachJogMinimumDurationSec);
      compiled.duration_sec = trajectoryDurationSec(compiled.trajectory);
    }
    if (relative_teach_jog) {
      for (auto & point : compiled.trajectory.points) {
        for (std::size_t index = 0; index < point.positions.size(); ++index) {
          point.positions[index] += holding_offset[index];
        }
      }
      {
        std::lock_guard<std::mutex> lock(compiler_mutex_);
        result = compiler_->validateTrajectoryStates(candidate, compiled.trajectory);
      }
      if (!result.success) {
        response.success = false;
        response.message = "holding-compensated jog validation failed: " + result.message;
        return;
      }
      compiled.start_joints = compiled.trajectory.points.front().positions;
      compiled.end_joints = compiled.trajectory.points.back().positions;
    }
    double max_joint_delta = 0.0;
    for (std::size_t index = 0; index < compiled.start_joints.size(); ++index) {
      max_joint_delta = std::max(
        max_joint_delta,
        std::abs(compiled.end_joints[index] - compiled.start_joints[index]));
    }
    RCLCPP_INFO(
      logger_,
      "staged Cartesian jog route=%s delta=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] "
      "duration=%.3fs compile=%.3fs max_joint_delta=%.4frad",
      route_name.c_str(), deltas[0], deltas[1], deltas[2], deltas[3], deltas[4], deltas[5],
      compiled.duration_sec,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - compile_started).count(),
      max_joint_delta);
    {
      std::lock_guard<std::mutex> lock(catalog_mutex_);
      for (auto it = compiled_routes_.begin(); it != compiled_routes_.end(); ) {
        if (it->first.rfind(kStagedJogPrefix, 0) == 0) {
          it = compiled_routes_.erase(it);
        } else {
          ++it;
        }
      }
      compiled_routes_[route_name] = std::move(compiled);
    }
    response.success = true;
    response.message = request.process_profile ?
      "Cartesian process move validated and staged" :
      "Cartesian jog validated and staged";
    response.route_name = route_name;
    if (relative_teach_jog) {
      const Eigen::Matrix3d base_delta =
        (Eigen::AngleAxisd(request.delta_yaw_rad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(request.delta_pitch_rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(request.delta_roll_rad, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
      response.target_xyz_m = {
        measured_pose.xyz[0] + request.delta_x_m,
        measured_pose.xyz[1] + request.delta_y_m,
        measured_pose.xyz[2] + request.delta_z_m};
      response.target_rpy_rad = matrixToRpy(base_delta * measured_rotation);
    } else {
      response.target_xyz_m = goal.pose->xyz;
      response.target_rpy_rad = goal.pose->rpy;
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

    if (busy_.load()) {
      RCLCPP_WARN(logger_, "reject route '%s': motion server busy", goal->route_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
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
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = false;
      result->error_code = ExecuteMotion::Result::ERROR_BUSY;
      result->message = "motion server is busy";
      result->elapsed_sec = 0.0;
      goal_handle->abort(result);
      return;
    }
    cancel_requested_.store(false);
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
    constexpr char debug_exit_suffix[] = "_to_safe";
    const bool debug_exit =
      route.name.rfind("debug_", 0) == 0 &&
      route.name.size() >= sizeof(debug_exit_suffix) - 1 &&
      route.name.compare(
      route.name.size() - (sizeof(debug_exit_suffix) - 1),
      sizeof(debug_exit_suffix) - 1,
      debug_exit_suffix) == 0;
    const bool sensor_pick_recovery =
      route.name == "spectrometer_sensor_pick_hover_to_brush_entry_recovery";
    const double tolerance = debug_exit || sensor_pick_recovery ?
      std::max(start_tolerance_rad_, 0.50) : start_tolerance_rad_;
    if (maximum > tolerance) {
      std::ostringstream out;
      out << "route start mismatch at " << route.trajectory.joint_names[maximum_index]
          << ": error=" << maximum << "rad limit=" << tolerance << "rad";
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
      if (it->first.rfind(kStagedJogPrefix, 0) == 0) {
        compiled_routes_.erase(it);
      }
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

    const bool is_staged_jog = route.name.rfind(kStagedJogPrefix, 0) == 0;
    const double final_position_tolerance = goal_position_tolerance_rad_;
    FollowTrajectory::Goal controller_request;
    try {
      controller_request.trajectory = scaleTrajectory(route.trajectory, requested_scale);
      const double start_correction = is_staged_jog ? 0.0 :
        alignTrajectoryStart(controller_request.trajectory, current);
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
      // Wrist feedback arrives in batches. Keep strict path protection on joints 1-4,
      // while allowing joints 5-6 to track through transient lag; final tolerance stays strict.
      path_tolerance.position = joint_name == "joint5" || joint_name == "joint6" ?
        std::max(route_path_tolerance, wrist_path_position_tolerance_rad_) :
        route_path_tolerance;
      controller_request.path_tolerance.push_back(path_tolerance);

      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = joint_name;
      tolerance.position = final_position_tolerance;
      tolerance.velocity = goal_velocity_tolerance_rad_sec_;
      controller_request.goal_tolerance.push_back(tolerance);
    }

    for (int controller_attempt = 0; controller_attempt < 2; ++controller_attempt) {
      controller_request.trajectory.header.stamp =
        node_->now() + rclcpp::Duration::from_seconds(trajectory_start_delay_sec_);
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
        std::chrono::duration<double>(expected_duration + execution_margin_sec_ + 1.0);
      bool resume_requested = false;
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
                final_position_tolerance,
                final_state_timeout_sec_,
                final_error))
            {
              finish(
                goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_FAILED,
                "controller reported success but final state was not confirmed: " + final_error,
                started);
              return;
            }
            {
              std::lock_guard<std::mutex> lock(commanded_state_mutex_);
              last_commanded_joints_ = route.end_joints;
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
          const bool settled = waitForArmSettled(route.trajectory.joint_names, settle_error);
          if (controller_attempt == 0 && settled && wrapped.result &&
            wrapped.result->error_code == FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED)
          {
            std::vector<double> resume_state;
            std::string state_error;
            if (currentJointState(
                route.trajectory.joint_names, resume_state, state_error))
            {
              try {
                controller_request.trajectory = makeResumeTrajectory(
                  controller_request.trajectory, resume_state, 0.50);
                RCLCPP_WARN(
                  logger_,
                  "route '%s' transient path tracking fault; resuming once with %zu points",
                  route.name.c_str(), controller_request.trajectory.points.size());
                resume_requested = true;
                break;
              } catch (const std::exception & error) {
                controller_message += "; automatic resume rejected: ";
                controller_message += error.what();
              }
            } else {
              controller_message += "; automatic resume state unavailable: " + state_error;
            }
          }
          if (!settled) {
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
      if (resume_requested) {
        continue;
      }

      std::string settle_error;
      cancelControllerGoalAndWait(
        controller_goal, route.trajectory.joint_names, settle_error);
      finish(
        goal_handle, false, ExecuteMotion::Result::ERROR_TIMEOUT,
        "motion route exceeded controller execution timeout; settle=" + settle_error, started);
      return;
    }
    finish(
      goal_handle, false, ExecuteMotion::Result::ERROR_CONTROLLER_FAILED,
      "automatic trajectory resume was exhausted", started);
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
  double process_stage_max_translation_m_;
  double default_speed_scale_;

  std::unique_ptr<TrajectoryCompiler> compiler_;
  std::mutex compiler_mutex_;
  MotionCatalog catalog_;
  std::map<std::string, CompiledRoute> compiled_routes_;
  std::mutex catalog_mutex_;
  std::mutex commanded_state_mutex_;
  std::vector<double> last_commanded_joints_;

  rclcpp_action::Client<FollowTrajectory>::SharedPtr controller_client_;
  rclcpp_action::Server<ExecuteMotion>::SharedPtr execute_server_;
  rclcpp::Service<Trigger>::SharedPtr reload_service_;
  rclcpp::Service<Trigger>::SharedPtr list_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<SetSpeedScale>::SharedPtr speed_service_;
  rclcpp::Service<StageJog>::SharedPtr stage_jog_service_;
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
  std::atomic_bool commissioning_only_{false};
  std::atomic_bool cancel_requested_{false};
  std::atomic<double> speed_scale_{1.0};
  std::atomic<std::uint64_t> jog_sequence_{1};
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
    if (!rclcpp::ok()) {
      return 0;
    }
    RCLCPP_FATAL(node->get_logger(), "motion server startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
