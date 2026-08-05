#include "panthera_motion/Compiler.hpp"

#include <algorithm>
#include <array>
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
#include <tuple>
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

struct QuinticDynamicBounds
{
  double velocity{0.0};
  double acceleration{0.0};
  double jerk{0.0};
};

void appendUnitRoot(std::vector<double> & roots, double root)
{
  constexpr double tolerance = 1e-10;
  if (!std::isfinite(root) || root < -tolerance || root > 1.0 + tolerance) {
    return;
  }
  root = std::clamp(root, 0.0, 1.0);
  if (std::none_of(
      roots.begin(), roots.end(),
      [root](double existing) {return std::abs(existing - root) <= 1e-8;}))
  {
    roots.push_back(root);
  }
}

std::vector<double> quadraticUnitRoots(
  double coefficient2, double coefficient1,
  double coefficient0)
{
  std::vector<double> roots;
  const double scale = std::max(
    1.0, std::abs(coefficient2) + std::abs(coefficient1) + std::abs(coefficient0));
  const double tolerance = 1e-12 * scale;
  if (std::abs(coefficient2) <= tolerance) {
    if (std::abs(coefficient1) > tolerance) {
      appendUnitRoot(roots, -coefficient0 / coefficient1);
    }
    return roots;
  }

  double discriminant = coefficient1 * coefficient1 -
    4.0 * coefficient2 * coefficient0;
  if (discriminant < -tolerance * scale) {
    return roots;
  }
  discriminant = std::max(0.0, discriminant);
  const double square_root = std::sqrt(discriminant);
  const double q = -0.5 * (coefficient1 + std::copysign(square_root, coefficient1));
  if (std::abs(q) <= tolerance) {
    appendUnitRoot(roots, -coefficient1 / (2.0 * coefficient2));
  } else {
    appendUnitRoot(roots, q / coefficient2);
    appendUnitRoot(roots, coefficient0 / q);
  }
  return roots;
}

std::vector<double> cubicUnitRoots(
  double coefficient3, double coefficient2, double coefficient1, double coefficient0)
{
  const double scale = std::max(
    1.0, std::abs(coefficient3) + std::abs(coefficient2) +
    std::abs(coefficient1) + std::abs(coefficient0));
  const double tolerance = 1e-11 * scale;
  if (std::abs(coefficient3) <= tolerance) {
    return quadraticUnitRoots(coefficient2, coefficient1, coefficient0);
  }

  const auto evaluate = [ = ](double value) {
      return ((coefficient3 * value + coefficient2) * value + coefficient1) * value +
             coefficient0;
    };
  std::vector<double> knots{0.0, 1.0};
  for (const double root : quadraticUnitRoots(
      3.0 * coefficient3, 2.0 * coefficient2, coefficient1))
  {
    appendUnitRoot(knots, root);
  }
  std::sort(knots.begin(), knots.end());

  std::vector<double> roots;
  for (const double knot : knots) {
    if (std::abs(evaluate(knot)) <= tolerance) {
      appendUnitRoot(roots, knot);
    }
  }
  for (std::size_t index = 1; index < knots.size(); ++index) {
    double lower = knots[index - 1];
    double upper = knots[index];
    double lower_value = evaluate(lower);
    const double upper_value = evaluate(upper);
    if (lower_value * upper_value >= 0.0) {
      continue;
    }
    for (int iteration = 0; iteration < 80; ++iteration) {
      const double middle = 0.5 * (lower + upper);
      const double middle_value = evaluate(middle);
      if (lower_value * middle_value <= 0.0) {
        upper = middle;
      } else {
        lower = middle;
        lower_value = middle_value;
      }
    }
    appendUnitRoot(roots, 0.5 * (lower + upper));
  }
  return roots;
}

QuinticDynamicBounds quinticDynamicBounds(
  double position0,
  double velocity0,
  double acceleration0,
  double position1,
  double velocity1,
  double acceleration1,
  double duration)
{
  if (!std::isfinite(duration) || duration <= 1e-9) {
    const double infinity = std::numeric_limits<double>::infinity();
    return {infinity, infinity, infinity};
  }
  const double duration2 = duration * duration;
  const double delta = position1 - position0;
  // q(s) = sum(coefficients[i] * s^i), s in [0, 1].  Evaluating the
  // derivative polynomial at every real stationary point gives the true
  // interval extrema without the excessive slowdown of a convex-hull bound.
  const std::array<double, 6> coefficients{
    position0,
    velocity0 * duration,
    0.5 * acceleration0 * duration2,
    10.0 * delta - (6.0 * velocity0 + 4.0 * velocity1) * duration -
    (1.5 * acceleration0 - 0.5 * acceleration1) * duration2,
    -15.0 * delta + (8.0 * velocity0 + 7.0 * velocity1) * duration +
    (1.5 * acceleration0 - acceleration1) * duration2,
    6.0 * delta - 3.0 * (velocity0 + velocity1) * duration -
    0.5 * (acceleration0 - acceleration1) * duration2};

  const auto velocity = [&](double s) {
      return (coefficients[1] + s * (2.0 * coefficients[2] + s *
             (3.0 * coefficients[3] + s *
             (4.0 * coefficients[4] + s * 5.0 * coefficients[5])))) / duration;
    };
  const auto acceleration = [&](double s) {
      return (2.0 * coefficients[2] + s * (6.0 * coefficients[3] + s *
             (12.0 * coefficients[4] + s * 20.0 * coefficients[5]))) / duration2;
    };
  const auto jerk = [&](double s) {
      return (6.0 * coefficients[3] + s *
             (24.0 * coefficients[4] + s * 60.0 * coefficients[5])) /
             (duration2 * duration);
    };

  QuinticDynamicBounds bounds;
  std::vector<double> velocity_candidates{0.0, 1.0};
  for (const double root : cubicUnitRoots(
      20.0 * coefficients[5], 12.0 * coefficients[4],
      6.0 * coefficients[3], 2.0 * coefficients[2]))
  {
    appendUnitRoot(velocity_candidates, root);
  }
  for (const double candidate : velocity_candidates) {
    bounds.velocity = std::max(bounds.velocity, std::abs(velocity(candidate)));
  }

  std::vector<double> acceleration_candidates{0.0, 1.0};
  for (const double root : quadraticUnitRoots(
      60.0 * coefficients[5], 24.0 * coefficients[4], 6.0 * coefficients[3]))
  {
    appendUnitRoot(acceleration_candidates, root);
  }
  for (const double candidate : acceleration_candidates) {
    bounds.acceleration = std::max(
      bounds.acceleration, std::abs(acceleration(candidate)));
  }

  std::vector<double> jerk_candidates{0.0, 1.0};
  if (std::abs(coefficients[5]) > 1e-14) {
    appendUnitRoot(jerk_candidates, -coefficients[4] / (5.0 * coefficients[5]));
  }
  for (const double candidate : jerk_candidates) {
    bounds.jerk = std::max(bounds.jerk, std::abs(jerk(candidate)));
  }
  return bounds;
}

ValidationResult enforceQuinticDynamicsLimits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & max_velocities_rad_sec,
  const std::vector<double> & max_accelerations_rad_sec2,
  const std::vector<double> & max_jerks_rad_sec3)
{
  const std::size_t joint_count = trajectory.joint_names.size();
  if (trajectory.points.size() < 2 || max_jerks_rad_sec3.size() != trajectory.points.size() ||
    max_velocities_rad_sec.size() != joint_count ||
    max_accelerations_rad_sec2.size() != joint_count)
  {
    return ValidationResult::fail("trajectory dynamics limit input is invalid");
  }
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    if (!std::isfinite(max_velocities_rad_sec[joint]) ||
      max_velocities_rad_sec[joint] <= 0.0 ||
      !std::isfinite(max_accelerations_rad_sec2[joint]) ||
      max_accelerations_rad_sec2[joint] <= 0.0)
    {
      return ValidationResult::fail("trajectory velocity/acceleration limit is invalid");
    }
  }
  if (std::any_of(
      max_jerks_rad_sec3.begin() + 1, max_jerks_rad_sec3.end(),
      [](double limit) {return !std::isfinite(limit) || limit <= 0.0;}))
  {
    return ValidationResult::fail("trajectory jerk limit is invalid");
  }

  const double initial_duration = durationToSeconds(trajectory.points.back().time_from_start);
  std::size_t limiting_index = 0;
  std::size_t limiting_joint = 0;
  double limiting_peak = 0.0;
  double limiting_limit = 0.0;
  const char * limiting_kind = "none";
  bool limits_satisfied = false;
  double residual_stretch = 1.0;
  for (int pass = 0; pass < 20; ++pass) {
    std::vector<double> interval_stretch(trajectory.points.size(), 1.0);
    double maximum_stretch = 1.0;
    for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
      const auto & previous = trajectory.points[index - 1];
      const auto & current = trajectory.points[index];
      const double interval =
        durationToSeconds(current.time_from_start) -
        durationToSeconds(previous.time_from_start);
      if (interval <= 1e-9 || previous.positions.size() != joint_count ||
        current.positions.size() != joint_count || previous.velocities.size() != joint_count ||
        current.velocities.size() != joint_count ||
        previous.accelerations.size() != joint_count ||
        current.accelerations.size() != joint_count)
      {
        return ValidationResult::fail(
          "trajectory points must contain increasing time and complete "
          "position/velocity/acceleration dynamics");
      }
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const auto bounds = quinticDynamicBounds(
          previous.positions[joint], previous.velocities[joint],
          previous.accelerations[joint], current.positions[joint],
          current.velocities[joint], current.accelerations[joint], interval);
        if (!std::isfinite(bounds.velocity) || !std::isfinite(bounds.acceleration) ||
          !std::isfinite(bounds.jerk))
        {
          return ValidationResult::fail("trajectory contains non-finite quintic dynamics");
        }
        const std::array<std::tuple<double, const char *, double, double>, 3> candidates{{
          {bounds.velocity / max_velocities_rad_sec[joint], "velocity",
            bounds.velocity, max_velocities_rad_sec[joint]},
          {std::sqrt(bounds.acceleration / max_accelerations_rad_sec2[joint]), "acceleration",
            bounds.acceleration, max_accelerations_rad_sec2[joint]},
          {std::cbrt(bounds.jerk / max_jerks_rad_sec3[index]), "jerk",
            bounds.jerk, max_jerks_rad_sec3[index]}}};
        for (const auto & candidate : candidates) {
          if (std::get<0>(candidate) > interval_stretch[index]) {
            interval_stretch[index] = std::get<0>(candidate);
          }
          if (std::get<0>(candidate) > maximum_stretch) {
            maximum_stretch = std::get<0>(candidate);
            limiting_index = index;
            limiting_joint = joint;
            limiting_peak = std::get<2>(candidate);
            limiting_limit = std::get<3>(candidate);
            limiting_kind = std::get<1>(candidate);
          }
        }
      }
    }
    residual_stretch = maximum_stretch;
    if (maximum_stretch <= 1.001) {
      limits_satisfied = true;
      break;
    }

    for (auto & stretch : interval_stretch) {
      stretch = std::max(1.0, stretch * 1.001);
    }
    constexpr double maximum_neighbor_ratio = 1.25;
    for (std::size_t index = 2; index < interval_stretch.size(); ++index) {
      if (max_jerks_rad_sec3[index] != max_jerks_rad_sec3[index - 1]) {
        continue;
      }
      interval_stretch[index] = std::max(
        interval_stretch[index], interval_stretch[index - 1] / maximum_neighbor_ratio);
    }
    for (std::size_t index = interval_stretch.size() - 1; index > 1; --index) {
      if (max_jerks_rad_sec3[index] != max_jerks_rad_sec3[index - 1]) {
        continue;
      }
      interval_stretch[index - 1] = std::max(
        interval_stretch[index - 1], interval_stretch[index] / maximum_neighbor_ratio);
    }

    std::vector<double> old_times;
    old_times.reserve(trajectory.points.size());
    for (const auto & point : trajectory.points) {
      old_times.push_back(durationToSeconds(point.time_from_start));
    }
    double new_time = 0.0;
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
      if (index > 0) {
        new_time += (old_times[index] - old_times[index - 1]) * interval_stretch[index];
      }
      auto & point = trajectory.points[index];
      point.time_from_start = secondsToDuration(new_time);
      const double incoming = index == 0 ? interval_stretch[1] : interval_stretch[index];
      const double outgoing = index + 1 < interval_stretch.size() ?
        interval_stretch[index + 1] : incoming;
      const double waypoint_stretch = std::max(incoming, outgoing);
      for (auto & velocity : point.velocities) {
        velocity /= waypoint_stretch;
      }
      for (auto & acceleration : point.accelerations) {
        acceleration /= waypoint_stretch * waypoint_stretch;
      }
    }
  }
  if (!limits_satisfied) {
    residual_stretch *= 1.001;
    for (auto & point : trajectory.points) {
      point.time_from_start = secondsToDuration(
        durationToSeconds(point.time_from_start) * residual_stretch);
      for (auto & velocity : point.velocities) {
        velocity /= residual_stretch;
      }
      for (auto & acceleration : point.accelerations) {
        acceleration /= residual_stretch * residual_stretch;
      }
    }
  }
  const double final_duration = durationToSeconds(trajectory.points.back().time_from_start);
  std::ostringstream summary;
  summary << std::fixed << std::setprecision(3)
          << "local_stretch=" << final_duration / initial_duration
          << " limiter=" << limiting_kind
          << " interval=" << limiting_index
          << " joint=" << trajectory.joint_names[limiting_joint]
          << " peak=" << limiting_peak
          << " limit=" << limiting_limit;
  return ValidationResult::ok(summary.str());
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

std::array<double, 3> matrixToRpy(const Eigen::Matrix3d & rotation)
{
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return {roll, pitch, yaw};
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

ValidationResult enforceTrajectoryDynamicsLimits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & max_velocities_rad_sec,
  const std::vector<double> & max_accelerations_rad_sec2,
  double max_jerk_rad_sec3)
{
  return enforceTrajectoryDynamicsLimits(
    trajectory, max_velocities_rad_sec, max_accelerations_rad_sec2,
    std::vector<double>(trajectory.points.size(), max_jerk_rad_sec3));
}

ValidationResult enforceTrajectoryDynamicsLimits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & max_velocities_rad_sec,
  const std::vector<double> & max_accelerations_rad_sec2,
  const std::vector<double> & max_jerks_rad_sec3)
{
  return enforceQuinticDynamicsLimits(
    trajectory, max_velocities_rad_sec, max_accelerations_rad_sec2,
    max_jerks_rad_sec3);
}

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
    const auto * requested_joint_group =
      model->getJointModelGroup(catalog.group_name);
    if (!requested_joint_group) {
      return ValidationResult::fail(
        "robot model does not contain joint group '" + catalog.group_name + "'");
    }
    if (!model->hasLinkModel(catalog.tool_frame)) {
      return ValidationResult::fail(
        "robot model does not contain tool link '" + catalog.tool_frame + "'");
    }

    const auto model_joint_names = requested_joint_group->getVariableNames();
    if (model_joint_names != catalog.joint_names) {
      std::ostringstream out;
      out << "catalog joint_names do not match robot group order; model=[";
      for (std::size_t i = 0; i < model_joint_names.size(); ++i) {
        out << (i == 0 ? "" : ",") << model_joint_names[i];
      }
      out << "]";
      return ValidationResult::fail(out.str());
    }

    if (!scene || requested_joint_group != joint_group) {
      std::vector<double> requested_velocity_limits;
      std::vector<double> requested_acceleration_limits;
      for (const auto & joint_name : model_joint_names) {
        const auto & bounds = model->getVariableBounds(joint_name);
        if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_ ||
          !std::isfinite(bounds.max_velocity_) || bounds.max_velocity_ <= 0.0 ||
          !std::isfinite(bounds.max_acceleration_) || bounds.max_acceleration_ <= 0.0)
        {
          return ValidationResult::fail(
            "robot model is missing positive velocity/acceleration limits for '" +
            joint_name + "'");
        }
        requested_velocity_limits.push_back(bounds.max_velocity_);
        requested_acceleration_limits.push_back(bounds.max_acceleration_);
      }
      joint_group = requested_joint_group;
      joint_velocity_limits = std::move(requested_velocity_limits);
      joint_acceleration_limits = std::move(requested_acceleration_limits);
      scene = std::make_shared<planning_scene::PlanningScene>(model);
    }

    if (catalog.base_frame != model->getModelFrame()) {
      RCLCPP_WARN(
        logger,
        "catalog base_frame=%s differs from robot model frame=%s; targets are interpreted in model frame",
        catalog.base_frame.c_str(),
        model->getModelFrame().c_str());
    }

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

    if (maximumJointDelta(current, target) <= 1e-8) {
      current = target;
      return ValidationResult::ok();
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
      current = sample;
    }
    // Joint interpolation samples are for collision/bounds validation only.
    // A straight joint-space segment needs one controller waypoint at its
    // target; publishing every validation sample creates needless quintic
    // knots and prevents short fixed moves from reaching their speed limit.
    trajectory.addSuffixWayPoint(state, 0.0);
    return ValidationResult::ok();
  }

  void applyVerticalConstraint(
    const SegmentDefinition & segment,
    const Eigen::Isometry3d & start,
    Eigen::Isometry3d & target)
  {
    const auto & axis = segment.constraints.vertical_axis;
    if (axis.empty()) {
      return;
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
      target.translation()[index] = start.translation()[index];
    }
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

    applyVerticalConstraint(segment, start_pose, target_pose);

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
    if (distance <= 1e-8 && orientation_distance <= 1e-8) {
      return ValidationResult::ok();
    }
    const std::size_t translation_steps = interpolationSteps(distance, segment.cartesian_step_m);
    const std::size_t rotation_steps = interpolationSteps(orientation_distance, 0.05);
    const std::size_t steps = std::max(translation_steps, rotation_steps);
    // Keep validation dense while avoiding unnecessary controller knots.
    constexpr double controller_cartesian_step_m = 0.020;
    const std::size_t controller_stride = std::max<std::size_t>(
      1, static_cast<std::size_t>(
        std::ceil(controller_cartesian_step_m / segment.cartesian_step_m)));
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

      auto result = checkState(
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

      if (index == steps || index % controller_stride == 0) {
        trajectory.addSuffixWayPoint(state, 0.0);
      }
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

    struct SegmentProfile
    {
      std::size_t start_index;
      std::size_t end_index;
      double velocity_scale;
      double acceleration_scale;
      bool cartesian;
      bool stop_at_end;
    };
    std::vector<SegmentProfile> profiles;

    output.name = route.name;
    output.start_joints = current;
    for (const auto & segment : route.segments) {
      const std::size_t start_index = trajectory.getWayPointCount() - 1;
      output.segment_names.push_back(segment.name);
      if (segment.type == SegmentType::JOINT) {
        result = appendJointSegment(catalog, route, segment, current, trajectory);
      } else {
        result = appendLinearSegment(catalog, route, segment, current, trajectory);
      }
      if (!result.success) {
        return result;
      }
      profiles.push_back(
        SegmentProfile{
          start_index,
          trajectory.getWayPointCount() - 1,
          segment.velocity_scale.value_or(route.velocity_scale),
          segment.acceleration_scale.value_or(route.acceleration_scale),
          segment.type == SegmentType::LINEAR,
          segment.stop_at_end});
    }

    if (trajectory.getWayPointCount() < 2) {
      return ValidationResult::fail("route '" + route.name + "' compiled fewer than 2 waypoints");
    }

    double maximum_velocity_scale = route.velocity_scale;
    double maximum_acceleration_scale = route.acceleration_scale;
    for (const auto & profile : profiles) {
      maximum_velocity_scale = std::max(maximum_velocity_scale, profile.velocity_scale);
      maximum_acceleration_scale =
        std::max(maximum_acceleration_scale, profile.acceleration_scale);
    }

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, maximum_velocity_scale, maximum_acceleration_scale))
    {
      return ValidationResult::fail("route '" + route.name + "' time parameterization failed");
    }

    for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
      auto timed_state = trajectory.getWayPoint(index);
      result = checkState(
        timed_state,
        catalog,
        "route '" + route.name + "' timed sample " + std::to_string(index));
      if (!result.success) {
        return result;
      }
    }

    moveit_msgs::msg::RobotTrajectory message;
    trajectory.getRobotTrajectoryMsg(message);
    output.trajectory = std::move(message.joint_trajectory);
    output.trajectory.header.frame_id = catalog.base_frame;

    std::vector<double> interval_stretch(output.trajectory.points.size(), 1.0);
    for (const auto & profile : profiles) {
      const double stretch = std::max(
        maximum_velocity_scale / profile.velocity_scale,
        std::sqrt(maximum_acceleration_scale / profile.acceleration_scale));
      for (std::size_t index = profile.start_index + 1;
        index <= profile.end_index && index < interval_stretch.size(); ++index)
      {
        interval_stretch[index] = stretch;
      }
    }

    std::vector<double> original_times;
    original_times.reserve(output.trajectory.points.size());
    for (const auto & point : output.trajectory.points) {
      original_times.push_back(durationToSeconds(point.time_from_start));
    }
    double scaled_time = 0.0;
    for (std::size_t index = 0; index < output.trajectory.points.size(); ++index) {
      if (index > 0) {
        scaled_time +=
          (original_times[index] - original_times[index - 1]) * interval_stretch[index];
      }
      auto & point = output.trajectory.points[index];
      point.time_from_start = secondsToDuration(scaled_time);

      const double incoming = index == 0 ? interval_stretch[1] : interval_stretch[index];
      const double outgoing = index + 1 < interval_stretch.size() ?
        interval_stretch[index + 1] : incoming;
      const bool explicit_stop = std::any_of(
        profiles.begin(), profiles.end(),
        [index](const SegmentProfile & profile) {
          return profile.stop_at_end && profile.end_index == index;
        });
      if (index == 0 || index + 1 == interval_stretch.size() || explicit_stop) {
        std::fill(point.velocities.begin(), point.velocities.end(), 0.0);
        std::fill(point.accelerations.begin(), point.accelerations.end(), 0.0);
      } else {
        // Keep one continuous waypoint velocity across a speed-class boundary.
        // The slower adjacent interval wins, so a profile change no longer forces
        // an unnecessary full stop while still respecting both segment caps.
        const double waypoint_stretch = std::max(incoming, outgoing);
        for (auto & velocity : point.velocities) {
          velocity /= waypoint_stretch;
        }
        for (auto & acceleration : point.accelerations) {
          acceleration /= waypoint_stretch * waypoint_stretch;
        }
      }
    }

    std::vector<double> interval_jerk_limits(
      output.trajectory.points.size(), catalog.defaults.max_jerk_rad_sec3);
    for (const auto & profile : profiles) {
      if (!profile.cartesian) {
        continue;
      }
      for (std::size_t index = profile.start_index + 1;
        index <= profile.end_index && index < interval_jerk_limits.size(); ++index)
      {
        interval_jerk_limits[index] = std::min(
          catalog.defaults.max_jerk_rad_sec3,
          catalog.defaults.cartesian_max_jerk_rad_sec3);
      }
    }
    result = enforceTrajectoryDynamicsLimits(
      output.trajectory, joint_velocity_limits, joint_acceleration_limits,
      interval_jerk_limits);
    if (!result.success) {
      return ValidationResult::fail(
        "route '" + route.name + "' dynamics limiting failed: " + result.message);
    }
    RCLCPP_INFO(logger, "route=%s dynamics %s", route.name.c_str(), result.message.c_str());

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
  std::vector<double> joint_velocity_limits;
  std::vector<double> joint_acceleration_limits;
  planning_scene::PlanningScenePtr scene;
  std::map<std::string, std::vector<double>> point_joints;
  std::set<std::string> resolving_points;
};

TrajectoryCompiler::TrajectoryCompiler(const rclcpp::Node::SharedPtr & node)
: impl_(std::make_unique<Impl>(node))
{
}

TrajectoryCompiler::~TrajectoryCompiler() = default;

ValidationResult TrajectoryCompiler::compileRoute(
  const MotionCatalog & catalog,
  const RouteDefinition & route,
  CompiledRoute & output)
{
  auto result = impl_->configure(catalog);
  if (!result.success) {
    return result;
  }
  return impl_->compileRoute(catalog, route, output);
}

ValidationResult TrajectoryCompiler::forwardKinematics(
  const MotionCatalog & catalog,
  const std::vector<double> & joints,
  PoseDefinition & output)
{
  auto result = impl_->configure(catalog);
  if (!result.success) {
    return result;
  }
  if (joints.size() != catalog.joint_names.size() ||
    !std::all_of(joints.begin(), joints.end(), [](double value) {return std::isfinite(value);}))
  {
    return ValidationResult::fail("forward kinematics joints must match catalog and be finite");
  }

  moveit::core::RobotState state(impl_->model);
  state.setToDefaultValues();
  state.setJointGroupPositions(impl_->joint_group, joints);
  result = impl_->checkState(state, catalog, "forward kinematics state");
  if (!result.success) {
    return result;
  }
  const auto transform = state.getGlobalLinkTransform(catalog.tool_frame);
  output.xyz = {
    transform.translation().x(), transform.translation().y(), transform.translation().z()};
  output.rpy = matrixToRpy(transform.rotation());
  return ValidationResult::ok();
}

ValidationResult TrajectoryCompiler::normalizeMeasuredJoints(
  const MotionCatalog & catalog,
  std::vector<double> & joints,
  double tolerance_rad)
{
  auto result = impl_->configure(catalog);
  if (!result.success) {
    return result;
  }
  if (joints.size() != catalog.joint_names.size() ||
    !std::all_of(
      joints.begin(), joints.end(),
      [](double value) {return std::isfinite(value);}) ||
    !std::isfinite(tolerance_rad) || tolerance_rad < 0.0)
  {
    return ValidationResult::fail("measured joints and tolerance must be valid");
  }

  moveit::core::RobotState state(impl_->model);
  state.setToDefaultValues();
  state.setJointGroupPositions(impl_->joint_group, joints);
  state.update();
  if (!state.satisfiesBounds(impl_->joint_group, tolerance_rad)) {
    return ValidationResult::fail("measured joint state exceeds limit tolerance");
  }
  state.enforceBounds(impl_->joint_group);
  state.copyJointGroupPositions(impl_->joint_group, joints);
  return ValidationResult::ok();
}

ValidationResult TrajectoryCompiler::validateTrajectoryStates(
  const MotionCatalog & catalog,
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  auto result = impl_->configure(catalog);
  if (!result.success) {
    return result;
  }
  if (trajectory.joint_names != catalog.joint_names || trajectory.points.empty()) {
    return ValidationResult::fail("trajectory joint names or points are invalid");
  }
  moveit::core::RobotState state(impl_->model);
  state.setToDefaultValues();
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & positions = trajectory.points[index].positions;
    if (positions.size() != catalog.joint_names.size() ||
      !std::all_of(
        positions.begin(), positions.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      return ValidationResult::fail("trajectory position vector is invalid");
    }
    state.setJointGroupPositions(impl_->joint_group, positions);
    result = impl_->checkState(
      state, catalog, "trajectory state " + std::to_string(index));
    if (!result.success) {
      return result;
    }
  }
  return ValidationResult::ok();
}

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

double alignTrajectoryStart(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & current_positions)
{
  if (trajectory.points.empty() ||
    trajectory.points.front().positions.size() != current_positions.size() ||
    trajectory.joint_names.size() != current_positions.size())
  {
    throw std::invalid_argument("trajectory start/current joint dimensions differ");
  }

  auto & start = trajectory.points.front();
  double maximum_correction = 0.0;
  for (std::size_t index = 0; index < current_positions.size(); ++index) {
    if (!std::isfinite(current_positions[index])) {
      throw std::invalid_argument("current joint positions must be finite");
    }
    maximum_correction = std::max(
      maximum_correction, std::abs(start.positions[index] - current_positions[index]));
  }
  start.positions = current_positions;
  start.velocities.assign(current_positions.size(), 0.0);
  start.accelerations.assign(current_positions.size(), 0.0);
  if (trajectory.points.size() > 1) {
    constexpr double minimum_start_interval_sec = 0.25;
    const double first_interval =
      durationToSeconds(trajectory.points[1].time_from_start) -
      durationToSeconds(start.time_from_start);
    const double extension = std::max(0.0, minimum_start_interval_sec - first_interval);
    for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
      trajectory.points[index].time_from_start = secondsToDuration(
        durationToSeconds(trajectory.points[index].time_from_start) + extension);
    }
  }
  return maximum_correction;
}

ValidationResult selectHoldingCommandStart(
  const std::vector<double> & measured_positions,
  const std::vector<double> & commanded_positions,
  double maximum_error_rad,
  std::vector<double> & output)
{
  if (measured_positions.empty() ||
    measured_positions.size() != commanded_positions.size() ||
    !std::isfinite(maximum_error_rad) || maximum_error_rad <= 0.0)
  {
    return ValidationResult::fail("holding command reference is unavailable");
  }
  double maximum_error = 0.0;
  for (std::size_t index = 0; index < measured_positions.size(); ++index) {
    if (!std::isfinite(measured_positions[index]) ||
      !std::isfinite(commanded_positions[index]))
    {
      return ValidationResult::fail("holding command reference is not finite");
    }
    maximum_error = std::max(
      maximum_error,
      std::abs(measured_positions[index] - commanded_positions[index]));
  }
  if (maximum_error > maximum_error_rad) {
    std::ostringstream message;
    message << "holding command differs from measured state by " << maximum_error
            << "rad (limit " << maximum_error_rad << "rad)";
    return ValidationResult::fail(message.str());
  }
  output = commanded_positions;
  return ValidationResult::ok();
}

trajectory_msgs::msg::JointTrajectory makeResumeTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  const std::vector<double> & current_positions,
  double maximum_deviation_rad)
{
  if (source.points.size() < 2 || source.joint_names.size() != current_positions.size() ||
    !std::isfinite(maximum_deviation_rad) || maximum_deviation_rad <= 0.0)
  {
    throw std::invalid_argument("resume trajectory input is invalid");
  }
  std::size_t nearest_index = 0;
  double nearest_deviation = std::numeric_limits<double>::infinity();
  for (std::size_t point_index = 0; point_index < source.points.size(); ++point_index) {
    const auto & positions = source.points[point_index].positions;
    if (positions.size() != current_positions.size()) {
      throw std::invalid_argument("resume trajectory point dimensions differ");
    }
    double deviation = 0.0;
    for (std::size_t joint = 0; joint < positions.size(); ++joint) {
      if (!std::isfinite(positions[joint]) || !std::isfinite(current_positions[joint])) {
        throw std::invalid_argument("resume trajectory contains non-finite positions");
      }
      deviation = std::max(deviation, std::abs(positions[joint] - current_positions[joint]));
    }
    if (deviation <= nearest_deviation) {
      nearest_deviation = deviation;
      nearest_index = point_index;
    }
  }
  if (nearest_deviation > maximum_deviation_rad || nearest_index + 1 >= source.points.size()) {
    throw std::invalid_argument("measured state is not resumable on the remaining trajectory");
  }

  auto output = source;
  output.points.clear();
  auto start = source.points[nearest_index];
  start.positions = current_positions;
  start.velocities.assign(current_positions.size(), 0.0);
  start.accelerations.assign(current_positions.size(), 0.0);
  start.time_from_start = secondsToDuration(0.0);
  output.points.push_back(std::move(start));

  const double base_time = durationToSeconds(source.points[nearest_index].time_from_start);
  const double first_interval =
    durationToSeconds(source.points[nearest_index + 1].time_from_start) - base_time;
  const double time_offset = std::max(0.0, 0.25 - first_interval);
  for (std::size_t index = nearest_index + 1; index < source.points.size(); ++index) {
    auto point = source.points[index];
    point.time_from_start = secondsToDuration(
      durationToSeconds(point.time_from_start) - base_time + time_offset);
    output.points.push_back(std::move(point));
  }
  return output;
}

trajectory_msgs::msg::JointTrajectory makeEndpointConvergenceTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  const std::vector<double> & current_positions,
  double maximum_speed_rad_sec,
  double minimum_duration_sec)
{
  if (source.points.empty() || source.joint_names.size() != current_positions.size() ||
    !std::isfinite(maximum_speed_rad_sec) || maximum_speed_rad_sec <= 0.0 ||
    !std::isfinite(minimum_duration_sec) || minimum_duration_sec <= 0.0)
  {
    throw std::invalid_argument("endpoint convergence trajectory input is invalid");
  }
  const auto & target_positions = source.points.back().positions;
  if (target_positions.size() != current_positions.size()) {
    throw std::invalid_argument("endpoint convergence trajectory point dimensions differ");
  }

  double maximum_error = 0.0;
  for (std::size_t joint = 0; joint < current_positions.size(); ++joint) {
    if (!std::isfinite(current_positions[joint]) || !std::isfinite(target_positions[joint])) {
      throw std::invalid_argument("endpoint convergence trajectory contains non-finite positions");
    }
    maximum_error = std::max(
      maximum_error, std::abs(target_positions[joint] - current_positions[joint]));
  }

  auto output = source;
  output.points.clear();
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = current_positions;
  start.velocities.assign(current_positions.size(), 0.0);
  start.accelerations.assign(current_positions.size(), 0.0);
  start.time_from_start = secondsToDuration(0.0);
  output.points.push_back(std::move(start));

  auto target = source.points.back();
  target.velocities.assign(current_positions.size(), 0.0);
  target.accelerations.assign(current_positions.size(), 0.0);
  target.time_from_start = secondsToDuration(
    std::max(minimum_duration_sec, maximum_error / maximum_speed_rad_sec));
  output.points.push_back(std::move(target));
  return output;
}

double maxAbsPositionSlope(
  const std::vector<double> & sample_times_sec,
  const std::vector<std::vector<double>> & position_samples)
{
  if (sample_times_sec.size() < 2 || position_samples.size() != sample_times_sec.size() ||
    position_samples.front().empty())
  {
    throw std::invalid_argument("position history must contain matching samples and times");
  }
  const std::size_t joint_count = position_samples.front().size();
  double mean_time = 0.0;
  for (const double time : sample_times_sec) {
    if (!std::isfinite(time)) {
      throw std::invalid_argument("position sample times must be finite");
    }
    mean_time += time;
  }
  mean_time /= static_cast<double>(sample_times_sec.size());

  double time_variance = 0.0;
  for (const double time : sample_times_sec) {
    const double centered = time - mean_time;
    time_variance += centered * centered;
  }
  if (time_variance <= 0.0) {
    throw std::invalid_argument("position sample times must increase");
  }

  std::vector<double> mean_positions(joint_count, 0.0);
  for (const auto & sample : position_samples) {
    if (sample.size() != joint_count) {
      throw std::invalid_argument("position sample dimensions differ");
    }
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      if (!std::isfinite(sample[joint])) {
        throw std::invalid_argument("position samples must be finite");
      }
      mean_positions[joint] += sample[joint];
    }
  }
  for (auto & mean : mean_positions) {
    mean /= static_cast<double>(position_samples.size());
  }

  double maximum = 0.0;
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    double covariance = 0.0;
    for (std::size_t sample = 0; sample < position_samples.size(); ++sample) {
      covariance +=
        (sample_times_sec[sample] - mean_time) *
        (position_samples[sample][joint] - mean_positions[joint]);
    }
    maximum = std::max(maximum, std::abs(covariance / time_variance));
  }
  return maximum;
}

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  return durationToSeconds(trajectory.points.back().time_from_start);
}

}  // namespace panthera_motion
