#include "panthera_motion/Compiler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace panthera_motion
{
namespace
{

builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<std::uint32_t>(
    std::round((seconds - static_cast<double>(duration.sec)) * 1e9));
  if (duration.nanosec >= 1000000000u) {
    ++duration.sec;
    duration.nanosec -= 1000000000u;
  }
  return duration;
}

double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

Eigen::Isometry3d poseToEigen(const PoseDefinition & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
  transform.linear() =
    (Eigen::AngleAxisd(pose.rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pose.rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose.rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

double rotationDistance(
  const Eigen::Matrix3d & first,
  const Eigen::Matrix3d & second)
{
  return std::abs(Eigen::AngleAxisd(first.transpose() * second).angle());
}

double maximumJointDelta(
  const std::vector<double> & first,
  const std::vector<double> & second)
{
  if (first.size() != second.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (std::size_t i = 0; i < first.size(); ++i) {
    maximum = std::max(maximum, std::abs(first[i] - second[i]));
  }
  return maximum;
}

std::string hashTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  std::ostringstream content;
  content << std::setprecision(17);
  for (const auto & name : trajectory.joint_names) {
    content << name << '|';
  }
  for (const auto & point : trajectory.points) {
    content << durationToSeconds(point.time_from_start) << ':';
    for (const double value : point.positions) {
      content << value << ',';
    }
    content << ':';
    for (const double value : point.velocities) {
      content << value << ',';
    }
    content << ';';
  }

  std::uint64_t hash = 1469598103934665603ull;
  for (const unsigned char byte : content.str()) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ull;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

std::size_t interpolationSteps(double magnitude, double step)
{
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(magnitude / step)));
}

}  // namespace

struct TrajectoryCompiler::Impl
{
  explicit Impl(const rclcpp::Node::SharedPtr & input_node)
  : node(input_node),
    logger(input_node->get_logger().get_child("trajectory_compiler")),
    model_loader(std::make_unique<robot_model_loader::RobotModelLoader>(
        input_node, "robot_description")),
    model(model_loader->getModel())
  {
    if (!model) {
      throw std::runtime_error("robot model loader returned no model");
    }
  }

  ValidationResult configure(const MotionCatalog & catalog)
  {
    joint_group = model->getJointModelGroup(catalog.group_name);
    if (!joint_group) {
      return ValidationResult::fail(
        "robot model does not contain joint group '" + catalog.group_name + "'");
    }
    if (!model->hasLinkModel(catalog.tool_frame)) {
      return ValidationResult::fail(
        "robot model does not contain tool link '" + catalog.tool_frame + "'");
    }

    const auto model_joint_names = joint_group->getVariableNames();
    if (model_joint_names != catalog.joint_names) {
      std::ostringstream out;
      out << "catalog joint_names do not match robot group order; model=[";
      for (std::size_t i = 0; i < model_joint_names.size(); ++i) {
        out << (i == 0 ? "" : ",") << model_joint_names[i];
      }
      out << "]";
      return ValidationResult::fail(out.str());
    }

    if (catalog.base_frame != model->getModelFrame()) {
      RCLCPP_WARN(
        logger,
        "catalog base_frame=%s differs from robot model frame=%s; targets are interpreted in model frame",
        catalog.base_frame.c_str(),
        model->getModelFrame().c_str());
    }

    scene = std::make_shared<planning_scene::PlanningScene>(model);
    point_joints.clear();
    resolving_points.clear();
    return ValidationResult::ok();
  }

  ValidationResult checkState(
    moveit::core::RobotState & state,
    const MotionCatalog & catalog,
    const std::string & context)
  {
    state.update();
    if (!state.satisfiesBounds(joint_group)) {
      return ValidationResult::fail(context + ": joint limits violated");
    }
    if (scene->isStateColliding(state, catalog.group_name, false)) {
      return ValidationResult::fail(context + ": robot state is in self/environment collision");
    }
    return ValidationResult::ok();
  }

  ValidationResult solvePose(
    const MotionCatalog & catalog,
    const PointDefinition & point,
    const std::vector<double> & seed,
    std::vector<double> & output)
  {
    if (!point.pose) {
      return ValidationResult::fail("point '" + point.name + "' has no pose for IK");
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_group, seed);
    state.update();
    const auto target = poseToEigen(*point.pose);
    const double timeout = catalog.defaults.ik_timeout_sec *
      static_cast<double>(catalog.defaults.ik_attempts);
    if (!state.setFromIK(joint_group, target, catalog.tool_frame, timeout)) {
      return ValidationResult::fail(
        "point '" + point.name + "' IK failed using seed '" + point.ik_seed + "'");
    }

    const auto state_check = checkState(state, catalog, "point '" + point.name + "'");
    if (!state_check.success) {
      return state_check;
    }
    state.copyJointGroupPositions(joint_group, output);

    const auto actual = state.getGlobalLinkTransform(catalog.tool_frame);
    const double position_error = (actual.translation() - target.translation()).norm();
    const double orientation_error = rotationDistance(actual.rotation(), target.rotation());
    if (position_error > 0.002 || orientation_error > 0.02) {
      std::ostringstream out;
      out << "point '" << point.name << "' IK reconstruction error position="
          << position_error << "m orientation=" << orientation_error << "rad";
      return ValidationResult::fail(out.str());
    }
    return ValidationResult::ok();
  }

  ValidationResult resolvePoint(
    const MotionCatalog & catalog,
    const std::string & point_name,
    std::vector<double> & output)
  {
    const auto cached = point_joints.find(point_name);
    if (cached != point_joints.end()) {
      output = cached->second;
      return ValidationResult::ok();
    }
    if (!resolving_points.insert(point_name).second) {
      return ValidationResult::fail("IK seed cycle while resolving point '" + point_name + "'");
    }

    const auto * point = catalog.findPoint(point_name);
    if (!point) {
      resolving_points.erase(point_name);
      return ValidationResult::fail("unknown point '" + point_name + "'");
    }

    ValidationResult result = ValidationResult::ok();
    if (!point->joints.empty()) {
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      state.setJointGroupPositions(joint_group, point->joints);
      result = checkState(state, catalog, "point '" + point_name + "'");
      if (result.success && point->pose) {
        const auto expected = poseToEigen(*point->pose);
        const auto actual = state.getGlobalLinkTransform(catalog.tool_frame);
        const double position_error = (actual.translation() - expected.translation()).norm();
        const double orientation_error = rotationDistance(actual.rotation(), expected.rotation());
        if (position_error > 0.005 || orientation_error > 0.05) {
          std::ostringstream out;
          out << "point '" << point_name << "' stored joints disagree with stored pose: position="
              << position_error << "m orientation=" << orientation_error << "rad";
          result = ValidationResult::fail(out.str());
        }
      }
      if (result.success) {
        output = point->joints;
      }
    } else {
      std::vector<double> seed = catalog.default_ik_seed;
      if (!point->ik_seed.empty()) {
        result = resolvePoint(catalog, point->ik_seed, seed);
      }
      if (result.success) {
        result = solvePose(catalog, *point, seed, output);
      }
    }

    resolving_points.erase(point_name);
    if (result.success) {
      point_joints[point_name] = output;
    }
    return result;
  }

  ValidationResult appendJointSegment(
    const MotionCatalog & catalog,
    const RouteDefinition & route,
    const SegmentDefinition & segment,
    std::vector<double> & current,
    robot_trajectory::RobotTrajectory & trajectory)
  {
    std::vector<double> target;
    auto result = resolvePoint(catalog, segment.to, target);
    if (!result.success) {
      return ValidationResult::fail(
        "route '" + route.name + "' segment '" + segment.name + "': " + result.message);
    }

    const std::size_t steps = interpolationSteps(
      maximumJointDelta(current, target), segment.joint_step_rad);
    const std::vector<double> start = current;
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (std::size_t index = 1; index <= steps; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      std::vector<double> sample(start.size(), 0.0);
      for (std::size_t joint = 0; joint < sample.size(); ++joint) {
        sample[joint] = start[joint] + (target[joint] - start[joint]) * ratio;
      }
      state.setJointGroupPositions(joint_group, sample);
      result = checkState(
        state,
        catalog,
        "route '" + route.name + "' segment '" + segment.name + "' sample " +
        std::to_string(index) + "/" + std::to_string(steps));
      if (!result.success) {
        return result;
      }
      trajectory.addSuffixWayPoint(state, 0.0);
      current = sample;
    }
    return ValidationResult::ok();
  }

  ValidationResult validateVerticalConstraint(
    const RouteDefinition & route,
    const SegmentDefinition & segment,
    const Eigen::Isometry3d & start,
    const Eigen::Isometry3d & target)
  {
    const auto & axis = segment.constraints.vertical_axis;
    if (axis.empty()) {
      return ValidationResult::ok();
    }
    std::array<int, 2> lateral_axes{};
    if (axis == "x") {
      lateral_axes = {1, 2};
    } else if (axis == "y") {
      lateral_axes = {0, 2};
    } else {
      lateral_axes = {0, 1};
    }
    for (const int index : lateral_axes) {
      const double error = std::abs(target.translation()[index] - start.translation()[index]);
      if (error > segment.constraints.max_lateral_error_m) {
        std::ostringstream out;
        out << "route '" << route.name << "' segment '" << segment.name
            << "' violates vertical " << axis << " constraint: lateral error=" << error
            << "m limit=" << segment.constraints.max_lateral_error_m << "m";
        return ValidationResult::fail(out.str());
      }
    }
    return ValidationResult::ok();
  }

  ValidationResult appendLinearSegment(
    const MotionCatalog & catalog,
    const RouteDefinition & route,
    const SegmentDefinition & segment,
    std::vector<double> & current,
    robot_trajectory::RobotTrajectory & trajectory)
  {
    const auto * target_point = catalog.findPoint(segment.to);
    if (!target_point || !target_point->pose) {
      return ValidationResult::fail(
        "route '" + route.name + "' segment '" + segment.name + "' target has no pose");
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_group, current);
    state.update();
    const Eigen::Isometry3d start_pose = state.getGlobalLinkTransform(catalog.tool_frame);
    Eigen::Isometry3d target_pose = poseToEigen(*target_point->pose);

    auto result = validateVerticalConstraint(route, segment, start_pose, target_pose);
    if (!result.success) {
      return result;
    }

    const double orientation_distance = rotationDistance(
      start_pose.rotation(), target_pose.rotation());
    if (segment.constraints.keep_orientation) {
      if (orientation_distance > 0.02) {
        std::ostringstream out;
        out << "route '" << route.name << "' segment '" << segment.name
            << "' keep_orientation target differs by " << orientation_distance << "rad";
        return ValidationResult::fail(out.str());
      }
      target_pose.linear() = start_pose.rotation();
    }

    const double distance = (target_pose.translation() - start_pose.translation()).norm();
    const std::size_t translation_steps = interpolationSteps(distance, segment.cartesian_step_m);
    const std::size_t rotation_steps = interpolationSteps(orientation_distance, 0.05);
    const std::size_t steps = std::max(translation_steps, rotation_steps);
    const Eigen::Quaterniond start_rotation(start_pose.rotation());
    const Eigen::Quaterniond target_rotation(target_pose.rotation());

    std::vector<double> previous = current;
    for (std::size_t index = 1; index <= steps; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      Eigen::Isometry3d sample_pose = Eigen::Isometry3d::Identity();
      sample_pose.translation() =
        start_pose.translation() + ratio * (target_pose.translation() - start_pose.translation());
      sample_pose.linear() =
        start_rotation.slerp(ratio, target_rotation).normalized().toRotationMatrix();

      state.setJointGroupPositions(joint_group, previous);
      state.update();
      if (!state.setFromIK(
          joint_group,
          sample_pose,
          catalog.tool_frame,
          catalog.defaults.ik_timeout_sec))
      {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name + "' IK failed at sample " +
          std::to_string(index) + "/" + std::to_string(steps));
      }

      std::vector<double> sample_joints;
      state.copyJointGroupPositions(joint_group, sample_joints);
      const double jump = maximumJointDelta(previous, sample_joints);
      if (jump > segment.max_joint_jump_rad) {
        std::ostringstream out;
        out << "route '" << route.name << "' segment '" << segment.name
            << "' IK branch jump=" << jump << "rad at sample " << index
            << "/" << steps << " limit=" << segment.max_joint_jump_rad;
        return ValidationResult::fail(out.str());
      }

      result = checkState(
        state,
        catalog,
        "route '" + route.name + "' segment '" + segment.name + "' sample " +
        std::to_string(index) + "/" + std::to_string(steps));
      if (!result.success) {
        return result;
      }

      const auto reconstructed = state.getGlobalLinkTransform(catalog.tool_frame);
      const double position_error =
        (reconstructed.translation() - sample_pose.translation()).norm();
      const double angle_error =
        rotationDistance(reconstructed.rotation(), sample_pose.rotation());
      if (position_error > 0.002 || angle_error > 0.02) {
        std::ostringstream out;
        out << "route '" << route.name << "' segment '" << segment.name
            << "' Cartesian reconstruction error at sample " << index << "/" << steps
            << ": position=" << position_error << "m orientation=" << angle_error << "rad";
        return ValidationResult::fail(out.str());
      }

      trajectory.addSuffixWayPoint(state, 0.0);
      previous = sample_joints;
    }
    current = previous;
    return ValidationResult::ok();
  }

  ValidationResult compileRoute(
    const MotionCatalog & catalog,
    const RouteDefinition & route,
    CompiledRoute & output)
  {
    std::vector<double> current;
    auto result = resolvePoint(catalog, route.start, current);
    if (!result.success) {
      return ValidationResult::fail("route '" + route.name + "' start: " + result.message);
    }

    robot_trajectory::RobotTrajectory trajectory(model, catalog.group_name);
    moveit::core::RobotState start_state(model);
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(joint_group, current);
    result = checkState(start_state, catalog, "route '" + route.name + "' start");
    if (!result.success) {
      return result;
    }
    trajectory.addSuffixWayPoint(start_state, 0.0);

    output.name = route.name;
    output.start_joints = current;
    for (const auto & segment : route.segments) {
      output.segment_names.push_back(segment.name);
      if (segment.type == SegmentType::JOINT) {
        result = appendJointSegment(catalog, route, segment, current, trajectory);
      } else {
        result = appendLinearSegment(catalog, route, segment, current, trajectory);
      }
      if (!result.success) {
        return result;
      }
    }

    if (trajectory.getWayPointCount() < 2) {
      return ValidationResult::fail("route '" + route.name + "' compiled fewer than 2 waypoints");
    }

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, route.velocity_scale, route.acceleration_scale))
    {
      return ValidationResult::fail("route '" + route.name + "' time parameterization failed");
    }

    moveit_msgs::msg::RobotTrajectory message;
    trajectory.getRobotTrajectoryMsg(message);
    output.trajectory = std::move(message.joint_trajectory);
    output.trajectory.header.frame_id = catalog.base_frame;
    output.end_joints = current;
    output.duration_sec = trajectoryDurationSec(output.trajectory);
    output.content_hash = hashTrajectory(output.trajectory);
    return ValidationResult::ok();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Logger logger;
  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader;
  moveit::core::RobotModelPtr model;
  const moveit::core::JointModelGroup * joint_group{nullptr};
  planning_scene::PlanningScenePtr scene;
  std::map<std::string, std::vector<double>> point_joints;
  std::set<std::string> resolving_points;
};

TrajectoryCompiler::TrajectoryCompiler(const rclcpp::Node::SharedPtr & node)
: impl_(std::make_unique<Impl>(node))
{
}

TrajectoryCompiler::~TrajectoryCompiler() = default;

ValidationResult TrajectoryCompiler::compileAll(
  const MotionCatalog & catalog,
  std::map<std::string, CompiledRoute> & output)
{
  auto result = impl_->configure(catalog);
  if (!result.success) {
    return result;
  }

  std::map<std::string, CompiledRoute> candidate;
  for (const auto & item : catalog.routes) {
    const auto & route = item.second;
    if (!route.enabled) {
      continue;
    }
    CompiledRoute compiled;
    result = impl_->compileRoute(catalog, route, compiled);
    if (!result.success) {
      return result;
    }
    RCLCPP_INFO(
      impl_->logger,
      "compiled route=%s points=%zu duration=%.3fs hash=%s",
      route.name.c_str(),
      compiled.trajectory.points.size(),
      compiled.duration_sec,
      compiled.content_hash.c_str());
    candidate.emplace(route.name, std::move(compiled));
  }
  if (candidate.empty()) {
    return ValidationResult::fail("motion catalog has no enabled routes to compile");
  }
  output = std::move(candidate);
  return ValidationResult::ok(
    "compiled " + std::to_string(output.size()) + " motion routes");
}

trajectory_msgs::msg::JointTrajectory scaleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  double speed_scale)
{
  if (!std::isfinite(speed_scale) || speed_scale <= 0.0 || speed_scale > 1.0) {
    throw std::invalid_argument("speed_scale must be finite and in (0, 1]");
  }

  auto output = source;
  for (auto & point : output.points) {
    point.time_from_start = secondsToDuration(
      durationToSeconds(point.time_from_start) / speed_scale);
    for (auto & velocity : point.velocities) {
      velocity *= speed_scale;
    }
    for (auto & acceleration : point.accelerations) {
      acceleration *= speed_scale * speed_scale;
    }
  }
  return output;
}

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  return durationToSeconds(trajectory.points.back().time_from_start);
}

}  // namespace panthera_motion
