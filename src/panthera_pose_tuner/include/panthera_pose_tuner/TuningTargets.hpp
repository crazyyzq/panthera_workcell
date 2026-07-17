#pragma once

#include <array>
#include <string>
#include <vector>

namespace panthera_pose_tuner
{

enum class TargetKind
{
  Pose,
  Joint
};

struct RpyDeg
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct PoseMm
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TuningTarget
{
  TargetKind kind{TargetKind::Pose};
  std::string name;
  std::string description;

  // Pose target: gripper_center pose in base_link. Use millimeters and degrees for friendly tuning.
  PoseMm xyz_mm;
  RpyDeg rpy_deg;
  std::string base_frame{"base_link"};
  std::string hand_frame{"gripper_center"};
  double position_tolerance_mm{3.0};
  double orientation_tolerance_deg{3.0};

  // Joint target: 6 arm joints in radians. Useful as a known-safe transition before pose tuning.
  std::array<double, 6> joints_rad{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double velocity_scale{0.08};
  double acceleration_scale{0.08};
  double planning_time_sec{5.0};
  int planning_attempts{5};
  bool allow_execute{true};
};

std::vector<TuningTarget> buildTuningTargets();

}  // namespace panthera_pose_tuner
