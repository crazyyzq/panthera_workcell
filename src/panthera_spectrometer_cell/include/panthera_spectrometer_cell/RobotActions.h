#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "panthera_rs485/modbus_rtu.hpp"
#include "panthera_interfaces/action/execute_motion.hpp"
#include "panthera_interfaces/srv/stage_jog.hpp"
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
  ActionResult recoverHomeAfterError();
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
  ActionResult setBrush(bool enabled, double speed_percent);
  ActionResult restartCleaningMotor();
  ActionResult openGripper();
  ActionResult closeGripper();

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
  ActionResult initializeFixedMotionInterface();
  ActionResult initializeGripperAndJointStateInterfaces();
  ActionResult executeFixedRoute(
    const std::string & route_name,
    const std::string & expected_end_point,
    const std::string & label);
  ActionResult stageAndExecuteProcessPoint(
    const std::string & point_name,
    const Vec3 & offset_xyz,
    const std::string & expected_end_point,
    const std::string & label);
  ActionResult stageAndExecuteProcessRoute(
    const std::string & route_name,
    const std::vector<std::string> & offset_point_names,
    const Vec3 & offset_xyz,
    const std::string & expected_end_point,
    const std::string & label);
  ActionResult computeSpectrometerOffset(double axis_position_mm, Vec3 & offset_xyz) const;
  ActionResult executeFixedCleaning();
  bool usesFixedMotion() const;
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
    const std::string & label,
    bool require_grasp_contact = false);
  ActionResult closeGripperForGrasp();
  bool waitForGripperTarget(
    double target_position,
    std::chrono::duration<double> timeout,
    bool allow_grasp_contact,
    bool require_grasp_contact,
    bool & grasp_contact,
    std::string & error) const;
  ActionResult setCleaningMotor(bool enabled, const std::string & label);
  ActionResult setCleaningMotorDuty(
    bool enabled, int duty_permille, const std::string & label);
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
  rclcpp_action::Client<panthera_interfaces::action::ExecuteMotion>::SharedPtr motion_client_;
  rclcpp::Client<panthera_interfaces::srv::StageJog>::SharedPtr process_stage_client_;
  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
  arm_recovery_client_;
  rclcpp::CallbackGroup::SharedPtr arm_recovery_callback_group_;
  mutable std::mutex fixed_state_mutex_;
  std::string fixed_point_;
  OutletId active_outlet_{OutletId::NONE};
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::CallbackGroup::SharedPtr gripper_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  rclcpp::Time latest_joint_state_received_;
  bool has_joint_state_{false};
  std::mutex cleaning_motor_mutex_;
  std::unique_ptr<panthera_rs485::ModbusRtuMaster> cleaning_motor_modbus_;
  std::atomic<double> speed_scale_{1.0};
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_dist_{0.0, 1.0};
};

}  // namespace panthera_spectrometer_cell
