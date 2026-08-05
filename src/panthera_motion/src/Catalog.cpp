#include "panthera_motion/Catalog.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace panthera_motion
{
namespace
{

double finiteDouble(
  const YAML::Node & node,
  const std::string & key,
  double default_value,
  const std::string & context)
{
  const double value = node && node[key] ? node[key].as<double>() : default_value;
  if (!std::isfinite(value)) {
    throw std::runtime_error(context + "." + key + " must be finite");
  }
  return value;
}

int readInt(
  const YAML::Node & node,
  const std::string & key,
  int default_value)
{
  return node && node[key] ? node[key].as<int>() : default_value;
}

bool readBool(
  const YAML::Node & node,
  const std::string & key,
  bool default_value)
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

std::vector<double> readDoubleVector(
  const YAML::Node & node,
  const std::string & context)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error(context + " must be a numeric array");
  }

  std::vector<double> output;
  output.reserve(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    const double value = node[i].as<double>();
    if (!std::isfinite(value)) {
      throw std::runtime_error(context + " contains a non-finite value");
    }
    output.push_back(value);
  }
  return output;
}

Vec3 readVec3(const YAML::Node & node, const std::string & context)
{
  const auto values = readDoubleVector(node, context);
  if (values.size() != 3) {
    throw std::runtime_error(context + " must contain exactly 3 values");
  }
  return Vec3{values[0], values[1], values[2]};
}

std::vector<std::string> readStringVector(
  const YAML::Node & node,
  const std::string & context)
{
  if (!node) {
    return {};
  }
  if (!node.IsSequence()) {
    throw std::runtime_error(context + " must be a string array");
  }

  std::vector<std::string> output;
  output.reserve(node.size());
  for (const auto & value : node) {
    output.push_back(value.as<std::string>());
  }
  return output;
}

SegmentType segmentTypeFromString(const std::string & value)
{
  if (value == "joint") {
    return SegmentType::JOINT;
  }
  if (value == "linear") {
    return SegmentType::LINEAR;
  }
  throw std::runtime_error("unsupported segment type: " + value);
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

bool allFinite(const Vec3 & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

}  // namespace

ValidationResult ValidationResult::ok(const std::string & message)
{
  return ValidationResult{true, message};
}

ValidationResult ValidationResult::fail(const std::string & message)
{
  return ValidationResult{false, message};
}

MotionCatalog MotionCatalog::loadFromFile(const std::string & path)
{
  MotionCatalog catalog;
  catalog.source_path = path;

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse motion catalog '" + path + "': " + error.what());
  }

  catalog.schema_version = readInt(root, "schema_version", 1);

  const auto robot = root["robot"];
  catalog.group_name = readString(robot, "group", catalog.group_name);
  catalog.base_frame = readString(robot, "base_frame", catalog.base_frame);
  catalog.tool_frame = readString(robot, "tool_frame", catalog.tool_frame);
  if (robot && robot["joint_names"]) {
    catalog.joint_names = readStringVector(robot["joint_names"], "robot.joint_names");
  }
  if (robot && robot["default_ik_seed"]) {
    catalog.default_ik_seed =
      readDoubleVector(robot["default_ik_seed"], "robot.default_ik_seed");
  }

  const auto defaults = root["defaults"];
  catalog.defaults.velocity_scale = finiteDouble(
    defaults, "velocity_scale", catalog.defaults.velocity_scale, "defaults");
  catalog.defaults.acceleration_scale = finiteDouble(
    defaults, "acceleration_scale", catalog.defaults.acceleration_scale, "defaults");
  catalog.defaults.max_jerk_rad_sec3 = finiteDouble(
    defaults, "max_jerk_rad_sec3", catalog.defaults.max_jerk_rad_sec3, "defaults");
  catalog.defaults.cartesian_max_jerk_rad_sec3 = finiteDouble(
    defaults, "cartesian_max_jerk_rad_sec3",
    catalog.defaults.cartesian_max_jerk_rad_sec3, "defaults");
  catalog.defaults.joint_step_rad = finiteDouble(
    defaults, "joint_step_rad", catalog.defaults.joint_step_rad, "defaults");
  catalog.defaults.cartesian_step_m = finiteDouble(
    defaults, "cartesian_step_m", catalog.defaults.cartesian_step_m, "defaults");
  catalog.defaults.max_joint_jump_rad = finiteDouble(
    defaults, "max_joint_jump_rad", catalog.defaults.max_joint_jump_rad, "defaults");
  catalog.defaults.ik_timeout_sec = finiteDouble(
    defaults, "ik_timeout_sec", catalog.defaults.ik_timeout_sec, "defaults");
  catalog.defaults.ik_attempts = readInt(
    defaults, "ik_attempts", catalog.defaults.ik_attempts);

  const auto points = root["points"];
  if (!points || !points.IsMap()) {
    throw std::runtime_error("motion catalog must contain a points map");
  }
  for (const auto & item : points) {
    PointDefinition point;
    point.name = item.first.as<std::string>();
    const auto node = item.second;
    point.description = readString(node, "description", "");
    point.ik_seed = readString(node, "ik_seed", "");
    point.tags = readStringVector(node["tags"], "points." + point.name + ".tags");
    if (node["joints"]) {
      point.joints = readDoubleVector(node["joints"], "points." + point.name + ".joints");
    }
    if (node["pose"]) {
      const auto pose_node = node["pose"];
      PoseDefinition pose;
      pose.xyz = readVec3(pose_node["xyz"], "points." + point.name + ".pose.xyz");
      pose.rpy = readVec3(pose_node["rpy"], "points." + point.name + ".pose.rpy");
      point.pose = pose;
    }
    catalog.points.emplace(point.name, std::move(point));
  }

  const auto routes = root["routes"];
  if (!routes || !routes.IsMap()) {
    throw std::runtime_error("motion catalog must contain a routes map");
  }
  for (const auto & item : routes) {
    RouteDefinition route;
    route.name = item.first.as<std::string>();
    const auto node = item.second;
    route.start = readString(node, "start", "");
    route.enabled = readBool(node, "enabled", true);
    route.velocity_scale = finiteDouble(
      node, "velocity_scale", catalog.defaults.velocity_scale, "routes." + route.name);
    route.acceleration_scale = finiteDouble(
      node, "acceleration_scale", catalog.defaults.acceleration_scale, "routes." + route.name);

    const auto segments = node["segments"];
    if (!segments || !segments.IsSequence()) {
      throw std::runtime_error("routes." + route.name + ".segments must be an array");
    }
    std::size_t index = 0;
    for (const auto & segment_node : segments) {
      SegmentDefinition segment;
      const std::string context =
        "routes." + route.name + ".segments[" + std::to_string(index) + "]";
      segment.name = readString(segment_node, "name", "segment_" + std::to_string(index + 1));
      segment.type = segmentTypeFromString(readString(segment_node, "type", "joint"));
      segment.to = readString(segment_node, "to", "");
      if (segment_node["velocity_scale"]) {
        segment.velocity_scale = finiteDouble(
          segment_node, "velocity_scale", route.velocity_scale, context);
      }
      if (segment_node["acceleration_scale"]) {
        segment.acceleration_scale = finiteDouble(
          segment_node, "acceleration_scale", route.acceleration_scale, context);
      }
      segment.stop_at_end = readBool(segment_node, "stop_at_end", false);
      segment.joint_step_rad = finiteDouble(
        segment_node, "joint_step_rad", catalog.defaults.joint_step_rad, context);
      segment.cartesian_step_m = finiteDouble(
        segment_node, "cartesian_step_m", catalog.defaults.cartesian_step_m, context);
      segment.max_joint_jump_rad = finiteDouble(
        segment_node, "max_joint_jump_rad", catalog.defaults.max_joint_jump_rad, context);

      const auto constraints = segment_node["constraints"];
      segment.constraints.vertical_axis = readString(constraints, "vertical_axis", "");
      segment.constraints.keep_orientation = readBool(constraints, "keep_orientation", true);
      route.segments.push_back(std::move(segment));
      ++index;
    }
    catalog.routes.emplace(route.name, std::move(route));
  }

  const auto validation = catalog.validate();
  if (!validation.success) {
    throw std::runtime_error(validation.message);
  }
  return catalog;
}

ValidationResult MotionCatalog::validate() const
{
  if (schema_version != 1) {
    return ValidationResult::fail(
      "unsupported motion catalog schema_version: " +
      std::to_string(schema_version));
  }
  if (group_name.empty() || base_frame.empty() || tool_frame.empty()) {
    return ValidationResult::fail("robot group/base_frame/tool_frame must not be empty");
  }
  if (joint_names.empty()) {
    return ValidationResult::fail("robot.joint_names must not be empty");
  }
  std::set<std::string> unique_joint_names;
  for (const auto & name : joint_names) {
    if (name.empty() || !unique_joint_names.insert(name).second) {
      return ValidationResult::fail("robot.joint_names contains an empty or duplicate name");
    }
  }
  if (default_ik_seed.size() != joint_names.size() || !allFinite(default_ik_seed)) {
    return ValidationResult::fail("robot.default_ik_seed must match joint_names and be finite");
  }
  if (defaults.velocity_scale <= 0.0 || defaults.velocity_scale > 1.0 ||
    defaults.acceleration_scale <= 0.0 || defaults.acceleration_scale > 1.0)
  {
    return ValidationResult::fail("default velocity/acceleration scales must be in (0, 1]");
  }
  if (defaults.joint_step_rad <= 0.0 || defaults.cartesian_step_m <= 0.0 ||
    defaults.max_joint_jump_rad <= 0.0 ||
    defaults.max_jerk_rad_sec3 <= 0.0 || defaults.cartesian_max_jerk_rad_sec3 <= 0.0 ||
    defaults.ik_timeout_sec <= 0.0 ||
    defaults.ik_attempts <= 0)
  {
    return ValidationResult::fail("motion catalog defaults contain invalid limits");
  }
  if (points.empty()) {
    return ValidationResult::fail("motion catalog points map is empty");
  }

  for (const auto & item : points) {
    const auto & point = item.second;
    if (point.name.empty() || point.name != item.first) {
      return ValidationResult::fail("point map key/name mismatch");
    }
    if (!point.pose && point.joints.empty()) {
      return ValidationResult::fail("point '" + point.name + "' needs pose or joints");
    }
    if (!point.joints.empty() &&
      (point.joints.size() != joint_names.size() || !allFinite(point.joints)))
    {
      return ValidationResult::fail(
        "point '" + point.name + "' joints must match robot joint_names and be finite");
    }
    if (point.pose && (!allFinite(point.pose->xyz) || !allFinite(point.pose->rpy))) {
      return ValidationResult::fail("point '" + point.name + "' pose must be finite");
    }
    if (!point.ik_seed.empty() && points.count(point.ik_seed) == 0) {
      return ValidationResult::fail(
        "point '" + point.name + "' has unknown ik_seed '" + point.ik_seed + "'");
    }
    if (point.ik_seed == point.name) {
      return ValidationResult::fail("point '" + point.name + "' cannot seed itself");
    }
  }

  std::map<std::string, int> visit_state;
  std::function<bool(const std::string &, std::string &)> visit =
    [&](const std::string & name, std::string & cycle) {
      if (visit_state[name] == 2) {
        return true;
      }
      if (visit_state[name] == 1) {
        cycle = name;
        return false;
      }
      visit_state[name] = 1;
      const auto & seed = points.at(name).ik_seed;
      if (!seed.empty() && !visit(seed, cycle)) {
        return false;
      }
      visit_state[name] = 2;
      return true;
    };
  for (const auto & item : points) {
    std::string cycle;
    if (!visit(item.first, cycle)) {
      return ValidationResult::fail("point ik_seed cycle detected at '" + cycle + "'");
    }
  }

  if (routes.empty()) {
    return ValidationResult::fail("motion catalog routes map is empty");
  }
  for (const auto & item : routes) {
    const auto & route = item.second;
    if (route.name.empty() || route.name != item.first) {
      return ValidationResult::fail("route map key/name mismatch");
    }
    if (points.count(route.start) == 0) {
      return ValidationResult::fail(
        "route '" + route.name + "' has unknown start point '" + route.start + "'");
    }
    if (route.segments.empty()) {
      return ValidationResult::fail("route '" + route.name + "' has no segments");
    }
    if (route.velocity_scale <= 0.0 || route.velocity_scale > 1.0 ||
      route.acceleration_scale <= 0.0 || route.acceleration_scale > 1.0)
    {
      return ValidationResult::fail(
        "route '" + route.name + "' velocity/acceleration scales must be in (0, 1]");
    }
    std::set<std::string> segment_names;
    for (const auto & segment : route.segments) {
      if (segment.name.empty() || !segment_names.insert(segment.name).second) {
        return ValidationResult::fail(
          "route '" + route.name + "' has an empty or duplicate segment name");
      }
      if (points.count(segment.to) == 0) {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name +
          "' references unknown point '" + segment.to + "'");
      }
      if (segment.type == SegmentType::LINEAR && !points.at(segment.to).pose) {
        return ValidationResult::fail(
          "linear segment '" + route.name + "." + segment.name +
          "' target needs a pose");
      }
      if (segment.joint_step_rad <= 0.0 || segment.cartesian_step_m <= 0.0 ||
        segment.max_joint_jump_rad <= 0.0)
      {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name + "' has invalid limits");
      }
      if ((segment.velocity_scale &&
        (*segment.velocity_scale <= 0.0 || *segment.velocity_scale > 1.0)) ||
        (segment.acceleration_scale &&
        (*segment.acceleration_scale <= 0.0 || *segment.acceleration_scale > 1.0)))
      {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name +
          "' velocity/acceleration scales must be in (0, 1]");
      }
      const auto & axis = segment.constraints.vertical_axis;
      if (!axis.empty() && axis != "x" && axis != "y" && axis != "z") {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name +
          "' vertical_axis must be x, y, z or empty");
      }
      if (!axis.empty() && segment.type != SegmentType::LINEAR) {
        return ValidationResult::fail(
          "route '" + route.name + "' segment '" + segment.name +
          "' vertical constraint is only valid for linear segments");
      }
    }
  }

  return ValidationResult::ok(
    "motion catalog valid: " + std::to_string(points.size()) + " points, " +
    std::to_string(routes.size()) + " routes");
}

const PointDefinition * MotionCatalog::findPoint(const std::string & name) const
{
  const auto it = points.find(name);
  return it == points.end() ? nullptr : &it->second;
}

const RouteDefinition * MotionCatalog::findRoute(const std::string & name) const
{
  const auto it = routes.find(name);
  return it == routes.end() ? nullptr : &it->second;
}

std::vector<std::string> MotionCatalog::enabledRouteNames() const
{
  std::vector<std::string> names;
  for (const auto & item : routes) {
    if (item.second.enabled) {
      names.push_back(item.first);
    }
  }
  return names;
}

}  // namespace panthera_motion
