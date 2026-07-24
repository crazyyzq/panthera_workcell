#ifndef PANTHERA_HARDWARE__PANTHERA_HARDWARE_INTERFACE_HPP_
#define PANTHERA_HARDWARE__PANTHERA_HARDWARE_INTERFACE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Forward declaration
namespace panthera {
class Panthera;
}

namespace panthera_hardware
{
class GravityModel;

class PantheraHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantheraHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Panthera robot instance
  std::unique_ptr<panthera::Panthera> robot_;
  std::shared_ptr<GravityModel> gravity_model_;

  // Configuration
  std::string config_file_;

  // Joint states
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> raw_position_candidates_;
  std::vector<int> raw_position_candidate_counts_;
  std::vector<std::deque<double>> raw_velocity_windows_;
  bool arm_state_available_{false};

  // Joint commands
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;

  // Control parameters
  std::vector<double> max_torques_;
  std::vector<double> max_velocities_;
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;
  std::vector<double> gravity_scales_;

  // Throttled logging requires a clock whose lifetime outlives the logging call.
  // A temporary shared clock here previously caused a use-after-free on error paths.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // Control mode
  std::string control_mode_;

  // Gripper conversion: radians to meters
  double gripper_rad_to_m_;  // Conversion factor for gripper position (rad to m)

  // Use velocity and effort commands
  bool use_velocity_commands_;
  bool use_effort_commands_;

  // Reject one-frame encoder/status glitches before they reach /joint_states and TF.
  bool state_filter_enabled_;
  double state_filter_max_arm_jump_rad_;
  double state_filter_max_gripper_jump_m_;
  int state_filter_accept_after_count_;
  bool state_velocity_filter_enabled_;
  int state_velocity_median_window_;
  double state_velocity_arm_deadband_rad_sec_;
  double state_velocity_gripper_deadband_m_sec_;

  double filterPositionSample(
    size_t joint_index,
    double raw_position,
    double max_jump);

  double filterVelocitySample(
    size_t joint_index,
    double raw_velocity,
    double deadband);
};

}  // namespace panthera_hardware

#endif  // PANTHERA_HARDWARE__PANTHERA_HARDWARE_INTERFACE_HPP_
