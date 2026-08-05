#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace panthera_motion
{

using Vec3 = std::array<double, 3>;

struct PoseDefinition
{
  Vec3 xyz{0.0, 0.0, 0.0};
  Vec3 rpy{0.0, 0.0, 0.0};
};

struct PointDefinition
{
  std::string name;
  std::optional<PoseDefinition> pose;
  std::vector<double> joints;
  std::string ik_seed;
  std::string description;
  std::vector<std::string> tags;
};

enum class SegmentType
{
  JOINT,
  LINEAR
};

struct SegmentConstraints
{
  std::string vertical_axis;
  bool keep_orientation{true};
};

struct SegmentDefinition
{
  std::string name;
  SegmentType type{SegmentType::JOINT};
  std::string to;
  std::optional<double> velocity_scale;
  std::optional<double> acceleration_scale;
  bool stop_at_end{false};
  double joint_step_rad{0.05};
  double cartesian_step_m{0.005};
  double max_joint_jump_rad{0.35};
  SegmentConstraints constraints;
};

struct RouteDefinition
{
  std::string name;
  std::string start;
  std::vector<SegmentDefinition> segments;
  double velocity_scale{0.10};
  double acceleration_scale{0.10};
  bool enabled{true};
};

struct CatalogDefaults
{
  double velocity_scale{0.10};
  double acceleration_scale{0.10};
  double max_jerk_rad_sec3{100.0};
  double cartesian_max_jerk_rad_sec3{40.0};
  double joint_step_rad{0.05};
  double cartesian_step_m{0.005};
  double max_joint_jump_rad{0.35};
  double ik_timeout_sec{0.10};
  int ik_attempts{10};
};

struct ValidationResult
{
  bool success{false};
  std::string message;

  static ValidationResult ok(const std::string & message = "ok");
  static ValidationResult fail(const std::string & message);
};

class MotionCatalog
{
public:
  static MotionCatalog loadFromFile(const std::string & path);

  ValidationResult validate() const;
  const PointDefinition * findPoint(const std::string & name) const;
  const RouteDefinition * findRoute(const std::string & name) const;
  std::vector<std::string> enabledRouteNames() const;

  int schema_version{1};
  std::string source_path;
  std::string group_name{"arm"};
  std::string base_frame{"base_link"};
  std::string tool_frame{"gripper_center"};
  std::vector<std::string> joint_names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  std::vector<double> default_ik_seed{0.0, 0.18, 0.18, 0.0, 0.0, 0.0};
  CatalogDefaults defaults;
  std::map<std::string, PointDefinition> points;
  std::map<std::string, RouteDefinition> routes;
};

}  // namespace panthera_motion
