#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "panthera_rs485/serial_port.hpp"
#include "panthera_spectrometer_cell/Config.h"
#include "panthera_spectrometer_cell/Types.h"

namespace panthera_spectrometer_cell
{

class RobotActions
{
public:
  RobotActions(rclcpp::Node::SharedPtr node, WorkcellConfig config);

  ActionResult initialize();
  ActionResult stop();
  ActionResult reset();
  void updateConfig(const WorkcellConfig & config);

  ActionResult pickFromOutlet(OutletId outlet);
  ActionResult returnCupToOutlet(OutletId outlet);
  ActionResult placeToSpectrometer(double axis_position_mm);
  ActionResult pickFromSpectrometer(double axis_position_mm);
  ActionResult cleanCup();
  ActionResult moveToOutletWait();
  ActionResult setSpeedScale(double scale);
  double speedScale() const;

private:
  ActionResult moveToNamedPose(const std::string & pose_name, const std::string & label);
  ActionResult moveToPose(const PoseConfig & pose, const std::string & label);
  ActionResult moveToPoseJoint(const PoseConfig & pose, const std::string & label);
  ActionResult moveToPoseCartesian(const PoseConfig & pose, const std::string & label);
  ActionResult moveToPoseCartesianWithJointFallback(
    const PoseConfig & pose,
    const std::string & label);
  ActionResult moveToSafePose(const std::string & label);
  ActionResult initializeRealInterfaces();
  ActionResult executePoseJoint(const PoseConfig & pose, const std::string & label);
  ActionResult executePoseCartesian(const PoseConfig & pose, const std::string & label);
  ActionResult executeJointTarget(const std::vector<double> & positions, const std::string & label);
  void waitForArmStateSettled(const std::string & label) const;
  bool setStartStateFromLatestArmJointState(const std::string & label);
  ActionResult executeWristPourSequence();
  ActionResult brushCleanCup();
  bool isJointWithinLimit(std::size_t joint_index, double position) const;
  bool getLatestArmJointValues(std::vector<double> & positions) const;
  ActionResult sendGripperTo(
    double position,
    double duration_sec,
    const std::string & label);
  ActionResult setCleaningMotor(bool enabled, const std::string & label);
  ActionResult openGripper();
  ActionResult closeGripper();
  ActionResult simulateDelay(const std::string & label);
  ActionResult maybeSimulatedFailure(const std::string & action_name);
  ActionResult computeSpectrometerTarget(
    double axis_position_mm,
    const Vec3 & offset_xyz,
    const std::string & target_name,
    PoseConfig & target) const;
  ActionResult applyCollisionObjects();
  ActionResult attachCup();
  ActionResult detachCup();
  double effectiveVelocityScale(double base_scale) const;
  double effectiveDuration(double duration_sec) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  WorkcellConfig config_;
  std::mutex motion_mutex_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_joint_state_{false};
  std::mutex cleaning_motor_mutex_;
  std::unique_ptr<panthera_rs485::SerialPort> cleaning_motor_serial_;
  std::atomic<double> speed_scale_{1.0};
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_dist_{0.0, 1.0};
};

}  // namespace panthera_spectrometer_cell
