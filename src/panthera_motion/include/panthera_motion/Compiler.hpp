#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "panthera_motion/Catalog.hpp"

namespace panthera_motion
{

struct CompiledRoute
{
  std::string name;
  std::vector<std::string> segment_names;
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<double> start_joints;
  std::vector<double> end_joints;
  double duration_sec{0.0};
  std::string content_hash;
};

class TrajectoryCompiler
{
public:
  explicit TrajectoryCompiler(const rclcpp::Node::SharedPtr & node);
  ~TrajectoryCompiler();

  TrajectoryCompiler(const TrajectoryCompiler &) = delete;
  TrajectoryCompiler & operator=(const TrajectoryCompiler &) = delete;

  ValidationResult compileAll(
    const MotionCatalog & catalog,
    std::map<std::string, CompiledRoute> & output);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

trajectory_msgs::msg::JointTrajectory scaleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  double speed_scale);

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory & trajectory);

ValidationResult enforceTrajectoryJerkLimit(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  double max_jerk_rad_sec3);

}  // namespace panthera_motion
