#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    trajectory_start_delay_sec_(getOrDeclareParameter<double>(
        node_, "trajectory_start_delay_sec", 0.10)),
    goal_position_tolerance_rad_(getOrDeclareParameter<double>(
        node_, "goal_position_tolerance_rad", 0.03)),
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
        latest_joint_state_ = *message;
        latest_joint_state_received_ = node_->now();
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
          out << item.first
              << " duration=" << item.second.duration_sec
              << "s points=" << item.second.trajectory.points.size()
              << " hash=" << item.second.content_hash << '\n';
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
      trajectory_start_delay_sec_ < 0.0 || goal_position_tolerance_rad_ <= 0.0 ||
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
      controller_client_->async_cancel_goal(controller_goal);
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
    rclcpp::Time received;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!has_joint_state_) {
        error = "no joint state has been received";
        return false;
      }
      state = latest_joint_state_;
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

    const double maximum_velocity = maxAbsVelocity(state, joint_names);
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

    if (success) {
      goal_handle->succeed(result);
    } else if (error_code == ExecuteMotion::Result::ERROR_CANCELLED) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    {
      std::lock_guard<std::mutex> lock(controller_goal_mutex_);
      controller_goal_.reset();
    }
    busy_.store(false);
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
        controller_client_->async_cancel_goal(controller_goal);
        finish(
          goal_handle, false, ExecuteMotion::Result::ERROR_CANCELLED,
          "motion route cancelled", started);
        return;
      }
      if (result_future.wait_for(50ms) == std::future_status::ready) {
        const auto wrapped = result_future.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result &&
          wrapped.result->error_code == FollowTrajectory::Result::SUCCESSFUL)
        {
          publishFeedback(goal_handle, route, 1.0);
          finish(
            goal_handle, true, ExecuteMotion::Result::ERROR_NONE,
            "motion route completed: " + route.name, started);
          return;
        }
        if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
          finish(
            goal_handle, false, ExecuteMotion::Result::ERROR_CANCELLED,
            "arm trajectory controller cancelled the route", started);
          return;
        }
        std::string controller_message = wrapped.result ?
          wrapped.result->error_string : "no controller result";
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

    controller_client_->async_cancel_goal(controller_goal);
    finish(
      goal_handle, false, ExecuteMotion::Result::ERROR_TIMEOUT,
      "motion route exceeded controller execution timeout", started);
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
  double trajectory_start_delay_sec_;
  double goal_position_tolerance_rad_;
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

  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
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
