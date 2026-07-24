#include "panthera_hardware/panthera_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "panthera/Panthera.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace panthera_hardware
{
namespace
{

bool readBoolParameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  bool default_value)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second == "true" || it->second == "1" || it->second == "True";
}

double readDoubleParameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? default_value : std::stod(it->second);
}

int readIntParameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  int default_value)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? default_value : std::stoi(it->second);
}

bool isPlausibleArmPosition(double position)
{
  // The vendor SDK uses 999 as a communication/status sentinel. No Panthera
  // arm joint can physically approach this range.
  return std::isfinite(position) && std::abs(position) <= 10.0;
}

bool parseGainVector(const char * value, std::vector<double> & gains)
{
  if (value == nullptr) {
    return false;
  }
  const std::string raw_value(value);
  if (raw_value.empty() || raw_value.back() == ',') {
    return false;
  }

  try {
    std::stringstream stream(raw_value);
    std::string item;
    gains.clear();
    while (std::getline(stream, item, ',')) {
      std::size_t parsed = 0;
      const double gain = std::stod(item, &parsed);
      if (item.find_first_not_of(" \t", parsed) != std::string::npos) {
        throw std::invalid_argument("gain contains trailing characters");
      }
      if (!std::isfinite(gain) || gain < 0.0) {
        gains.clear();
        return false;
      }
      gains.push_back(gain);
    }
  } catch (const std::exception &) {
    gains.clear();
    return false;
  }

  return gains.size() == 6;
}

bool parseFiniteVector(const char * value, std::vector<double> & values)
{
  if (value == nullptr) {
    return false;
  }
  const std::string raw_value(value);
  if (raw_value.empty() || raw_value.back() == ',') {
    return false;
  }

  try {
    std::stringstream stream(raw_value);
    std::string item;
    values.clear();
    while (std::getline(stream, item, ',')) {
      std::size_t parsed = 0;
      const double parsed_value = std::stod(item, &parsed);
      if (item.find_first_not_of(" \t", parsed) != std::string::npos ||
        !std::isfinite(parsed_value))
      {
        values.clear();
        return false;
      }
      values.push_back(parsed_value);
    }
  } catch (const std::exception &) {
    values.clear();
    return false;
  }

  return values.size() == 6;
}

}  // namespace

class GravityModel
{
public:
  bool load(const std::string & config_file)
  {
    try {
      const YAML::Node config = YAML::LoadFile(config_file);
      if (!config["urdf"] || !config["urdf"]["file_path"]) {
        return false;
      }

      const std::size_t separator = config_file.find_last_of("/\\");
      const std::string config_dir =
        separator == std::string::npos ? "." : config_file.substr(0, separator);
      std::string urdf_path =
        config_dir + "/" + config["urdf"]["file_path"].as<std::string>();

      if (urdf_path.size() >= 6 &&
        urdf_path.substr(urdf_path.size() - 6) == ".xacro")
      {
        const std::string urdf_candidate =
          urdf_path.substr(0, urdf_path.size() - 6) + ".urdf";
        std::ifstream urdf_file(urdf_candidate);
        if (!urdf_file.good()) {
          return false;
        }
        urdf_path = urdf_candidate;
      }

      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);

      if (!config["kinematics"] || !config["kinematics"]["joint_names"]) {
        return false;
      }

      joint_ids_.clear();
      const auto joint_names =
        config["kinematics"]["joint_names"].as<std::vector<std::string>>();
      for (const auto & name : joint_names) {
        if (!model_.existJointName(name)) {
          return false;
        }
        joint_ids_.push_back(model_.getJointId(name));
      }
      return joint_ids_.size() == 6;
    } catch (const std::exception &) {
      return false;
    }
  }

  std::vector<double> gravity(const std::vector<double> & positions) const
  {
    if (joint_ids_.size() != 6 || positions.size() != 6) {
      return {};
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
      const auto joint_id = joint_ids_[i];
      q[model_.joints[joint_id].idx_q()] = positions[i];
    }

    const Eigen::VectorXd generalized_gravity =
      pinocchio::computeGeneralizedGravity(model_, data_, q);

    std::vector<double> result(6, 0.0);
    for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
      result[i] = generalized_gravity[model_.joints[joint_ids_[i]].idx_v()];
    }
    return result;
  }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
};

hardware_interface::CallbackReturn PantheraHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get configuration file path
  if (info_.hardware_parameters.find("config_file") != info_.hardware_parameters.end())
  {
    config_file_ = info_.hardware_parameters["config_file"];
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Config file: %s", config_file_.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Parameter 'config_file' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get control mode (default: position_velocity)
  if (info_.hardware_parameters.find("control_mode") != info_.hardware_parameters.end())
  {
    control_mode_ = info_.hardware_parameters["control_mode"];
  }
  else
  {
    control_mode_ = "position_velocity";
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Control mode: %s", control_mode_.c_str());
  if (control_mode_ != "position_velocity" &&
    control_mode_ != "pd_control" && control_mode_ != "full_control" &&
    control_mode_ != "mit_gravity_compensation")
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "Unsupported control_mode '%s'", control_mode_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get gripper rad to meter conversion factor
  if (info_.hardware_parameters.find("gripper_rad_to_m") != info_.hardware_parameters.end())
  {
    gripper_rad_to_m_ = std::stod(info_.hardware_parameters["gripper_rad_to_m"]);
  }
  else
  {
    gripper_rad_to_m_ = 0.01;  // Default: 1 radian = 0.01 meter
  }
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
              "Gripper rad to meter conversion: %.4f", gripper_rad_to_m_);

  state_filter_enabled_ = readBoolParameter(info_, "state_filter_enabled", true);
  state_filter_max_arm_jump_rad_ =
    readDoubleParameter(info_, "state_filter_max_arm_jump_rad", 0.06);
  state_filter_max_gripper_jump_m_ =
    readDoubleParameter(info_, "state_filter_max_gripper_jump_m", 0.004);
  state_filter_accept_after_count_ =
    std::max(1, readIntParameter(info_, "state_filter_accept_after_count", 3));
  state_velocity_filter_enabled_ =
    readBoolParameter(info_, "state_velocity_filter_enabled", true);
  state_velocity_median_window_ =
    std::max(1, readIntParameter(info_, "state_velocity_median_window", 21));
  if (state_velocity_median_window_ % 2 == 0) {
    ++state_velocity_median_window_;
  }
  state_velocity_arm_deadband_rad_sec_ =
    std::max(
    0.0, readDoubleParameter(
      info_, "state_velocity_arm_deadband_rad_sec", 0.03));
  state_velocity_gripper_deadband_m_sec_ =
    std::max(
    0.0, readDoubleParameter(
      info_, "state_velocity_gripper_deadband_m_sec", 0.001));
  RCLCPP_INFO(
    rclcpp::get_logger("PantheraHardwareInterface"),
    "State filter: enabled=%s arm_jump=%.4frad gripper_jump=%.4fm accept_after=%d "
    "velocity_enabled=%s median_window=%d arm_deadband=%.4frad/s "
    "gripper_deadband=%.4fm/s",
    state_filter_enabled_ ? "true" : "false",
    state_filter_max_arm_jump_rad_,
    state_filter_max_gripper_jump_m_,
    state_filter_accept_after_count_,
    state_velocity_filter_enabled_ ? "true" : "false",
    state_velocity_median_window_,
    state_velocity_arm_deadband_rad_sec_,
    state_velocity_gripper_deadband_m_sec_);

  // Check if velocity and effort commands are enabled
  use_velocity_commands_ =
    control_mode_ == "full_control" ||
    control_mode_ == "position_velocity" ||
    control_mode_ == "mit_gravity_compensation";
  use_effort_commands_ = (control_mode_ == "full_control");

  // Initialize state and command storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  raw_position_candidates_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  raw_position_candidate_counts_.resize(info_.joints.size(), 0);
  raw_velocity_windows_.resize(info_.joints.size());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);

  // Initialize control parameters
  max_torques_.resize(info_.joints.size(), 0.0);
  max_velocities_.resize(info_.joints.size(), 0.5);
  kp_gains_.resize(info_.joints.size(), 0.0);
  kd_gains_.resize(info_.joints.size(), 0.0);
  gravity_scales_.assign(6, 1.0);

  // Load joint-specific parameters
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Max torque
    if (info_.joints[i].parameters.find("max_torque") != info_.joints[i].parameters.end())
    {
      max_torques_[i] = std::stod(info_.joints[i].parameters.at("max_torque"));
    }
    else
    {
      max_torques_[i] = 10.0;  // Default value
    }

    // Max velocity
    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end())
    {
      max_velocities_[i] = std::stod(info_.joints[i].parameters.at("max_velocity"));
    }
    else
    {
      max_velocities_[i] = 0.5;  // Default value
    }

    // PD gains
    if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end())
    {
      kp_gains_[i] = std::stod(info_.joints[i].parameters.at("kp"));
    }
    else
    {
      kp_gains_[i] = 4.0;  // Default value
    }

    if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end())
    {
      kd_gains_[i] = std::stod(info_.joints[i].parameters.at("kd"));
    }
    else
    {
      kd_gains_[i] = 0.5;  // Default value
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Joint %s: max_torque=%.2f, max_velocity=%.2f, kp=%.2f, kd=%.2f",
                info_.joints[i].name.c_str(), max_torques_[i], max_velocities_[i],
                kp_gains_[i], kd_gains_[i]);
  }

  if (control_mode_ == "mit_gravity_compensation") {
    std::vector<double> mit_kp(6, 60.0);
    std::vector<double> mit_kd(6, 5.5);
    const char * kp_value = std::getenv("PANTHERA_MIT_KP");
    const char * kd_value = std::getenv("PANTHERA_MIT_KD");
    const char * gravity_scale_value = std::getenv("PANTHERA_MIT_GRAVITY_SCALE");
    if ((kp_value != nullptr && !parseGainVector(kp_value, mit_kp)) ||
      (kd_value != nullptr && !parseGainVector(kd_value, mit_kd)) ||
      (gravity_scale_value != nullptr &&
      !parseFiniteVector(gravity_scale_value, gravity_scales_)))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "MIT gains must contain six finite non-negative values and gravity scale "
        "must contain six finite values");
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::copy(mit_kp.begin(), mit_kp.end(), kp_gains_.begin());
    std::copy(mit_kd.begin(), mit_kd.end(), kd_gains_.begin());
    RCLCPP_INFO(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "MIT gains: Kp=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
      "Kd=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      kp_gains_[0], kp_gains_[1], kp_gains_[2],
      kp_gains_[3], kp_gains_[4], kp_gains_[5],
      kd_gains_[0], kd_gains_[1], kd_gains_[2],
      kd_gains_[3], kd_gains_[4], kd_gains_[5]);
    RCLCPP_INFO(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "MIT gravity scale=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      gravity_scales_[0], gravity_scales_[1], gravity_scales_[2],
      gravity_scales_[3], gravity_scales_[4], gravity_scales_[5]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Configuring...");

  // Initialize Panthera robot
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initializing Panthera robot with config: %s", config_file_.c_str());

    robot_ = std::make_unique<panthera::Panthera>(config_file_);

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Panthera robot initialized successfully");
    if (control_mode_ == "mit_gravity_compensation") {
      gravity_model_ = std::make_shared<GravityModel>();
      if (!gravity_model_->load(config_file_)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("PantheraHardwareInterface"),
          "Failed to load the URDF/dynamics model for MIT gravity compensation");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "MIT gravity compensation model loaded successfully");
    }
  }
  catch (const std::bad_alloc & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Memory allocation failed during Panthera initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to initialize Panthera robot: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Unknown exception during Panthera initialization");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint states
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();
    if (positions.size() < 6 || velocities.size() < 6 || torques.size() < 6)
    {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("PantheraHardwareInterface"),
        throttle_clock_,
        1000,
        "Incomplete arm state from SDK: positions=%zu velocities=%zu torques=%zu",
        positions.size(),
        velocities.size(),
        torques.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < 6; i++)
    {
      if (!isPlausibleArmPosition(positions[i])) {
        RCLCPP_ERROR(
          rclcpp::get_logger("PantheraHardwareInterface"),
          "Reject invalid initial arm state on joint %zu: %.6f", i + 1, positions[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
      hw_positions_[i] = positions[i];
      hw_velocities_[i] = velocities[i];
      hw_efforts_[i] = torques[i];
      hw_commands_positions_[i] = positions[i];  // Initialize commands to current position
    }
    arm_state_available_ = true;

    // Read gripper state (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[6] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[6] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
      hw_commands_positions_[6] = hw_positions_[6];  // Initialize commands to current position
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = -hw_positions_[6];  // Mimic L_finger_joint position (negated, opposite direction)
      hw_velocities_[7] = -hw_velocities_[6];  // Mimic L_finger_joint velocity (negated)
      hw_efforts_[7] = 0.0;  // Passive joint, no actuator
      hw_commands_positions_[7] = hw_positions_[7];
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Initial joint states read successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to read initial joint states: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PantheraHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PantheraHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Skip command interfaces for mimic joints (R_finger_joint)
    if (info_.joints[i].name == "R_finger_joint")
    {
      continue;
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));

    if (use_velocity_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    if (use_effort_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Activating...");

  // Read current state and set as command
  try
  {
    // The SDK can expose one cached/stale frame while the serial devices are
    // coming online. Never latch a single frame as the first MIT target.
    std::array<std::vector<double>, 6> position_samples;
    for (int attempt = 0; attempt < 10 && position_samples[0].size() < 5; ++attempt) {
      robot_->send_get_motor_state_cmd();
      robot_->motor_send_cmd();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      const auto positions = robot_->getCurrentPos();
      if (positions.size() < 6 ||
        !std::all_of(
          positions.begin(), positions.begin() + 6,
          [](double position) {return isPlausibleArmPosition(position);}))
      {
        continue;
      }
      for (size_t i = 0; i < 6; ++i) {
        position_samples[i].push_back(positions[i]);
      }
    }
    if (position_samples[0].size() < 5) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "Failed to collect five valid arm-state samples during activation");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read 6 arm joint positions
    for (size_t i = 0; i < 6; i++)
    {
      auto & samples = position_samples[i];
      std::nth_element(samples.begin(), samples.begin() + 2, samples.end());
      hw_positions_[i] = samples[2];
      hw_commands_positions_[i] = samples[2];
    }

    // Read gripper position (7th joint, index 6 = L_finger_joint) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_commands_positions_[6] = gripper_rad * gripper_rad_to_m_;
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_commands_positions_[7] = -hw_commands_positions_[6];  // Mimic L_finger_joint position (negated)
    }

    RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"),
                "Hardware activated successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PantheraHardwareInterface"),
                 "Failed to activate hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PantheraHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Deactivating...");

  try
  {
    if (robot_)
    {
      // Keep the current pose when controllers are deactivated. A plain stop
      // releases the joint before the SDK destructor applies the brake.
      robot_->set_brake();
      robot_->set_brake();
      robot_->set_brake();
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "Failed to stop hardware while deactivating: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PantheraHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

double PantheraHardwareInterface::filterPositionSample(
  size_t joint_index,
  double raw_position,
  double max_jump)
{
  if (
    !state_filter_enabled_ ||
    joint_index >= hw_positions_.size() ||
    !std::isfinite(raw_position) ||
    !std::isfinite(hw_positions_[joint_index]))
  {
    if (joint_index < raw_position_candidates_.size()) {
      raw_position_candidates_[joint_index] = raw_position;
      raw_position_candidate_counts_[joint_index] = 0;
    }
    return raw_position;
  }

  const double last_position = hw_positions_[joint_index];
  const double jump = std::abs(raw_position - last_position);
  if (jump <= max_jump) {
    raw_position_candidates_[joint_index] = raw_position;
    raw_position_candidate_counts_[joint_index] = 0;
    return raw_position;
  }

  const double candidate = raw_position_candidates_[joint_index];
  if (std::isfinite(candidate) && std::abs(raw_position - candidate) <= max_jump) {
    raw_position_candidate_counts_[joint_index] += 1;
  } else {
    raw_position_candidates_[joint_index] = raw_position;
    raw_position_candidate_counts_[joint_index] = 1;
  }

  if (raw_position_candidate_counts_[joint_index] >= state_filter_accept_after_count_) {
    RCLCPP_WARN(
      rclcpp::get_logger("PantheraHardwareInterface"),
      "Accept persistent state jump on %s: %.4f -> %.4f rad/m after %d samples",
      info_.joints[joint_index].name.c_str(),
      last_position,
      raw_position,
      raw_position_candidate_counts_[joint_index]);
    raw_position_candidate_counts_[joint_index] = 0;
    return raw_position;
  }

  RCLCPP_WARN_THROTTLE(
    rclcpp::get_logger("PantheraHardwareInterface"),
    throttle_clock_,
    1000,
    "Reject transient state jump on %s: %.4f -> %.4f rad/m, holding previous value",
    info_.joints[joint_index].name.c_str(),
    last_position,
    raw_position);
  return last_position;
}

double PantheraHardwareInterface::filterVelocitySample(
  size_t joint_index,
  double raw_velocity,
  double deadband)
{
  if (
    !state_velocity_filter_enabled_ ||
    joint_index >= raw_velocity_windows_.size() ||
    !std::isfinite(raw_velocity))
  {
    return raw_velocity;
  }

  auto & window = raw_velocity_windows_[joint_index];
  window.push_back(raw_velocity);
  while (window.size() > static_cast<size_t>(state_velocity_median_window_)) {
    window.pop_front();
  }

  std::vector<double> sorted(window.begin(), window.end());
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
  std::nth_element(sorted.begin(), middle, sorted.end());
  const double filtered = *middle;
  return std::abs(filtered) < deadband ? 0.0 : filtered;
}

hardware_interface::return_type PantheraHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read joint states from hardware
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    // Read 6 arm joint states
    auto positions = robot_->getCurrentPos();
    auto torques = robot_->getCurrentTorque();
    if (positions.size() < 6 || torques.size() < 6)
    {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("PantheraHardwareInterface"),
        throttle_clock_,
        1000,
        "Incomplete arm state from SDK: positions=%zu torques=%zu",
        positions.size(),
        torques.size());
      return hardware_interface::return_type::ERROR;
    }

    const bool arm_state_valid = std::all_of(
      positions.begin(), positions.begin() + 6,
      [](double position) {return isPlausibleArmPosition(position);});
    if (!arm_state_valid) {
      arm_state_available_ = false;
      for (size_t i = 0; i < 6; ++i) {
        // Publish the SDK's explicit unavailable sentinel. Consumers reject
        // this sample and their last trusted state naturally becomes stale.
        hw_positions_[i] = positions[i];
        hw_velocities_[i] = 0.0;
        raw_position_candidate_counts_[i] = 0;
      }
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("PantheraHardwareInterface"),
        throttle_clock_, 1000,
        "Arm motors are not ready (SDK position sentinel); suppressing hardware commands");
      return hardware_interface::return_type::OK;
    }

    const bool arm_state_recovered = !arm_state_available_;
    const double period_sec = period.seconds();
    arm_state_available_ = true;
    for (size_t i = 0; i < 6; i++)
    {
      const double previous_position = hw_positions_[i];
      if (arm_state_recovered) {
        hw_positions_[i] = positions[i];
        raw_position_candidate_counts_[i] = 0;
        raw_velocity_windows_[i].clear();
      } else {
        hw_positions_[i] = filterPositionSample(i, positions[i], state_filter_max_arm_jump_rad_);
      }
      const double position_velocity =
        !arm_state_recovered && std::isfinite(previous_position) &&
        std::isfinite(period_sec) && period_sec > 0.0 ?
        (hw_positions_[i] - previous_position) / period_sec : 0.0;
      hw_velocities_[i] = filterVelocitySample(
        i, position_velocity, state_velocity_arm_deadband_rad_sec_);
      hw_efforts_[i] = torques[i];
    }
    if (arm_state_recovered) {
      RCLCPP_WARN(
        rclcpp::get_logger("PantheraHardwareInterface"),
        "Arm motor feedback recovered; hardware commands are enabled again");
    }

    // Read gripper state (7th joint, index 6 = L_finger_joint)
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 6)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      const double raw_gripper_position = gripper_rad * gripper_rad_to_m_;
      const double previous_gripper_position = hw_positions_[6];
      hw_positions_[6] =
        filterPositionSample(6, raw_gripper_position, state_filter_max_gripper_jump_m_);
      const double gripper_velocity =
        std::isfinite(previous_gripper_position) &&
        std::isfinite(period_sec) && period_sec > 0.0 ?
        (hw_positions_[6] - previous_gripper_position) / period_sec : 0.0;
      hw_velocities_[6] = filterVelocitySample(
        6,
        gripper_velocity,
        state_velocity_gripper_deadband_m_sec_);
      hw_efforts_[6] = robot_->getCurrentTorqueGripper();
    }

    // R_finger_joint (8th joint, index 7) is a mimic joint that follows L_finger_joint
    if (info_.joints.size() > 7)
    {
      hw_positions_[7] = -hw_positions_[6];  // Mimic L_finger_joint position (negated, opposite direction)
      hw_velocities_[7] = -hw_velocities_[6];  // Mimic L_finger_joint velocity (negated)
      hw_efforts_[7] = 0.0;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          throttle_clock_, 1000,
                          "Failed to read joint states: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PantheraHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to hardware
  try
  {
    if (!arm_state_available_) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("PantheraHardwareInterface"),
        throttle_clock_, 1000,
        "Suppress arm write while motors are not ready");
      return hardware_interface::return_type::OK;
    }
    // Extract first 6 joints for arm control
    std::vector<double> arm_positions(hw_commands_positions_.begin(),
                                       hw_commands_positions_.begin() + 6);
    std::vector<double> arm_velocities(hw_commands_velocities_.begin(),
                                        hw_commands_velocities_.begin() + 6);
    std::vector<double> arm_efforts(hw_commands_efforts_.begin(),
                                     hw_commands_efforts_.begin() + 6);
    std::vector<double> arm_max_torques(max_torques_.begin(), max_torques_.begin() + 6);
    std::vector<double> arm_max_velocities(max_velocities_.begin(), max_velocities_.begin() + 6);
    std::vector<double> arm_kp(kp_gains_.begin(), kp_gains_.begin() + 6);
    std::vector<double> arm_kd(kd_gains_.begin(), kd_gains_.begin() + 6);
    constexpr std::array<double, 6> kMinimumPosition{
      -2.4, -0.01, -0.01, -1.6, -1.7, -2.5};
    constexpr std::array<double, 6> kMaximumPosition{
      2.4, 3.2, 4.0, 1.6, 1.7, 2.5};
    for (std::size_t index = 0; index < arm_positions.size(); ++index)
    {
      if (!std::isfinite(arm_positions[index]) || !std::isfinite(arm_velocities[index]) ||
        !std::isfinite(arm_efforts[index]))
      {
        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("PantheraHardwareInterface"),
          throttle_clock_, 1000,
          "Rejected invalid command for arm joint %zu: position=%.6f; holding trusted state",
          index + 1, arm_positions[index]);
        return hardware_interface::return_type::ERROR;
      }
      arm_positions[index] = std::clamp(
        arm_positions[index], kMinimumPosition[index], kMaximumPosition[index]);
      arm_velocities[index] = std::clamp(
        arm_velocities[index], -arm_max_velocities[index], arm_max_velocities[index]);
      arm_efforts[index] = std::clamp(
        arm_efforts[index], -arm_max_torques[index], arm_max_torques[index]);
    }
    // Control 6 arm joints. Never report a successful hardware cycle when the
    // vendor SDK rejected the command.
    bool arm_command_ok = false;
    if (control_mode_ == "full_control")
    {
      // Full control mode: use position, velocity, and effort commands
      arm_command_ok = robot_->posVelTorqueKpKd(
        arm_positions, arm_velocities, arm_efforts, arm_kp, arm_kd);
    }
    else if (control_mode_ == "mit_gravity_compensation")
    {
      std::vector<double> current_positions(hw_positions_.begin(), hw_positions_.begin() + 6);
      std::vector<double> gravity_torque =
        gravity_model_ ? gravity_model_->gravity(current_positions) : std::vector<double>();
      if (gravity_torque.size() != 6 ||
        !std::all_of(
          gravity_torque.begin(), gravity_torque.end(),
          [](double torque) {return std::isfinite(torque);}))
      {
        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("PantheraHardwareInterface"),
          throttle_clock_, 1000,
          "Gravity compensation torque calculation failed");
        return hardware_interface::return_type::ERROR;
      }
      for (size_t i = 0; i < gravity_torque.size(); ++i) {
        gravity_torque[i] *= gravity_scales_[i];
        gravity_torque[i] = std::clamp(
          gravity_torque[i], -arm_max_torques[i], arm_max_torques[i]);
      }
      arm_command_ok = robot_->posVelTorqueKpKd(
        arm_positions, arm_velocities, gravity_torque, arm_kp, arm_kd);
    }
    else if (control_mode_ == "position_velocity")
    {
      // The vendor cooperative mode interprets velocity as a positive speed
      // ceiling, not a signed trajectory setpoint. Trajectory timing already
      // defines the desired motion; feeding its instantaneous velocity caused
      // repeated slow/stop commands near spline knots and visible stutter.
      const std::vector<double> & velocities = arm_max_velocities;
      arm_command_ok = robot_->posVelMaxTorque(
        arm_positions, velocities, arm_max_torques, false);
    }
    else if (control_mode_ == "pd_control")
    {
      // PD control mode (MIT mode with zero velocity and torque)
      std::vector<double> zero_vel(6, 0.0);
      std::vector<double> zero_torque(6, 0.0);
      arm_command_ok = robot_->posVelTorqueKpKd(
        arm_positions, zero_vel, zero_torque, arm_kp, arm_kd);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                            throttle_clock_, 1000,
                            "Unknown control mode: %s", control_mode_.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (!arm_command_ok)
    {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("PantheraHardwareInterface"),
        throttle_clock_, 1000,
        "Vendor SDK rejected an arm command in control mode '%s'", control_mode_.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Control gripper (7th joint, index 6) if present
    // Convert from meters to radians for hardware
    if (info_.joints.size() > 6)
    {
      double gripper_pos_m = hw_commands_positions_[6];
      if (!std::isfinite(gripper_pos_m) || gripper_rad_to_m_ <= 0.0) {
        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("PantheraHardwareInterface"),
          throttle_clock_, 1000,
          "Rejected invalid gripper command or conversion factor");
        return hardware_interface::return_type::ERROR;
      }
      double gripper_pos_rad = gripper_pos_m / gripper_rad_to_m_;
      double gripper_vel_m =
        control_mode_ == "full_control" ? hw_commands_velocities_[6] : max_velocities_[6];
      gripper_vel_m = std::clamp(
        gripper_vel_m, -max_velocities_[6], max_velocities_[6]);
      double gripper_vel_rad = gripper_vel_m / gripper_rad_to_m_;
      constexpr double kProtocolVelocityLimitRad = 50.0;
      gripper_vel_rad = std::clamp(
        gripper_vel_rad, -kProtocolVelocityLimitRad, kProtocolVelocityLimitRad);
      double gripper_max_torque = max_torques_[6];

      if (control_mode_ == "full_control")
      {
        // Use MIT mode for gripper
        double gripper_torque = hw_commands_efforts_[6];
        double gripper_kp = kp_gains_[6];
        double gripper_kd = kd_gains_[6];
        if (!robot_->gripperControlMIT(
            gripper_pos_rad, gripper_vel_rad, gripper_torque,
            gripper_kp, gripper_kd))
        {
          return hardware_interface::return_type::ERROR;
        }
      }
      else
      {
        // Use posVelMaxTorque mode for gripper
        if (!robot_->gripperControl(
            gripper_pos_rad, std::abs(gripper_vel_rad), gripper_max_torque))
        {
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("PantheraHardwareInterface"),
                          throttle_clock_, 1000,
                          "Failed to write commands: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace panthera_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panthera_hardware::PantheraHardwareInterface, hardware_interface::SystemInterface)
