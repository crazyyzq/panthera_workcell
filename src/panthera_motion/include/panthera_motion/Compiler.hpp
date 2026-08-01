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

  ValidationResult compileRoute(
    const MotionCatalog & catalog,
    const RouteDefinition & route,
    CompiledRoute & output);

  ValidationResult forwardKinematics(
    const MotionCatalog & catalog,
    const std::vector<double> & joints,
    PoseDefinition & output);

  ValidationResult normalizeMeasuredJoints(
    const MotionCatalog & catalog,
    std::vector<double> & joints,
    double tolerance_rad);

  ValidationResult validateTrajectoryStates(
    const MotionCatalog & catalog,
    const trajectory_msgs::msg::JointTrajectory & trajectory);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

trajectory_msgs::msg::JointTrajectory scaleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  double speed_scale);

double alignTrajectoryStart(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & current_positions);

ValidationResult selectHoldingCommandStart(
  const std::vector<double> & measured_positions,
  const std::vector<double> & commanded_positions,
  double maximum_error_rad,
  std::vector<double> & output);

trajectory_msgs::msg::JointTrajectory makeResumeTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  const std::vector<double> & current_positions,
  double maximum_deviation_rad);

double maxAbsPositionSlope(
  const std::vector<double> & sample_times_sec,
  const std::vector<std::vector<double>> & position_samples);

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory & trajectory);

ValidationResult enforceTrajectoryDynamicsLimits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & max_velocities_rad_sec,
  const std::vector<double> & max_accelerations_rad_sec2,
  double max_jerk_rad_sec3);

}  // namespace panthera_motion
