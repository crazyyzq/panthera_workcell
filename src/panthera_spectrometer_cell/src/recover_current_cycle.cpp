#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "panthera_spectrometer_cell/Config.h"
#include "panthera_spectrometer_cell/RobotActions.h"

using namespace std::chrono_literals;

namespace
{

std::string defaultConfigPath()
{
  return ament_index_cpp::get_package_share_directory("panthera_spectrometer_cell") +
         "/config/spectrometer_cell.yaml";
}

template<typename T>
T getOrDeclareParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    rclcpp::Parameter parameter;
    if (node->get_parameter(name, parameter)) {
      return parameter.get_value<T>();
    }
  }
  return node->declare_parameter<T>(name, default_value);
}

panthera_spectrometer_cell::OutletId outletFromString(const std::string & value)
{
  if (value == "OUTLET_1" || value == "outlet_1" || value == "1") {
    return panthera_spectrometer_cell::OutletId::OUTLET_1;
  }
  if (value == "OUTLET_2" || value == "outlet_2" || value == "2") {
    return panthera_spectrometer_cell::OutletId::OUTLET_2;
  }
  return panthera_spectrometer_cell::OutletId::NONE;
}

bool runStep(
  const rclcpp::Logger & logger,
  const std::string & label,
  const panthera_spectrometer_cell::ActionResult & result)
{
  if (result.success) {
    RCLCPP_INFO(logger, "RECOVERY_STEP_OK %s: %s", label.c_str(), result.message.c_str());
    return true;
  }
  RCLCPP_ERROR(logger, "RECOVERY_STEP_FAILED %s: %s", label.c_str(), result.message.c_str());
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "panthera_spectrometer_cell_recover_current_cycle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();
  const std::string config_file =
    getOrDeclareParameter<std::string>(node, "config_file", defaultConfigPath());
  const bool simulation_enabled =
    getOrDeclareParameter<bool>(node, "simulation_enabled", false);
  const bool gripper_command_enabled =
    getOrDeclareParameter<bool>(node, "gripper_command_enabled", true);
  const double spectrometer_laser_mm =
    getOrDeclareParameter<double>(node, "spectrometer_laser_mm", 150.3);
  const std::string outlet_name =
    getOrDeclareParameter<std::string>(node, "outlet", "OUTLET_1");
  const double detection_hold_sec =
    getOrDeclareParameter<double>(node, "detection_hold_sec", 1.0);
  const bool clean_return_only =
    getOrDeclareParameter<bool>(node, "clean_return_only", false);
  const bool start_from_spectrometer =
    getOrDeclareParameter<bool>(node, "start_from_spectrometer", false);

  const auto outlet = outletFromString(outlet_name);
  if (outlet == panthera_spectrometer_cell::OutletId::NONE) {
    RCLCPP_FATAL(logger, "unknown outlet '%s'", outlet_name.c_str());
    rclcpp::shutdown();
    return 2;
  }

  panthera_spectrometer_cell::WorkcellConfig config;
  try {
    config = panthera_spectrometer_cell::WorkcellConfig::loadFromFile(config_file);
    config.simulation.enabled = simulation_enabled;
    config.gripper.commandEnabled = gripper_command_enabled;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "failed to load config '%s': %s", config_file.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 0;
  auto robot = std::make_shared<panthera_spectrometer_cell::RobotActions>(node, config);
  try {
    RCLCPP_WARN(
      logger,
      "RECOVERY_START raw_laser=%.3fmm outlet=%s clean_return_only=%s "
      "start_from_spectrometer=%s gripper_command_enabled=%s",
      spectrometer_laser_mm,
      outlet_name.c_str(),
      clean_return_only ? "true" : "false",
      start_from_spectrometer ? "true" : "false",
      gripper_command_enabled ? "true" : "false");

    if (!runStep(logger, "initialize", robot->initialize())) {
      exit_code = 3;
    } else {
      std::this_thread::sleep_for(500ms);
    }

    if (exit_code == 0 && !clean_return_only && !start_from_spectrometer &&
      !runStep(logger, "place_to_spectrometer", robot->placeToSpectrometer(spectrometer_laser_mm)))
    {
      exit_code = 4;
    }

    if (exit_code == 0 && !clean_return_only && !start_from_spectrometer &&
      detection_hold_sec > 0.0)
    {
      RCLCPP_INFO(logger, "RECOVERY_HOLD spectrometer %.2fs", detection_hold_sec);
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(detection_hold_sec * 1000.0)));
    }

    if (exit_code == 0 && start_from_spectrometer &&
      !runStep(logger, "open_gripper", robot->openGripper()))
    {
      exit_code = 4;
    }

    if (exit_code == 0 && start_from_spectrometer &&
      !runStep(
        logger, "move_to_spectrometer_wait",
        robot->moveHomeToSpectrometerWaitForRecovery()))
    {
      exit_code = 4;
    }

    if (exit_code == 0 && !clean_return_only &&
      !runStep(logger, "pick_from_spectrometer", robot->pickFromSpectrometer(spectrometer_laser_mm)))
    {
      exit_code = 5;
    }

    if (exit_code == 0 && !runStep(logger, "clean_cup", robot->cleanCup())) {
      exit_code = 6;
    }

    if (exit_code == 0 && !runStep(logger, "return_cup", robot->returnCupToOutlet(outlet))) {
      exit_code = 7;
    }

    const auto stop_result = robot->stop();
    if (!stop_result.success) {
      RCLCPP_ERROR(logger, "RECOVERY_STOP_FAILED %s", stop_result.message.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "RECOVERY_EXCEPTION %s", e.what());
    try {
      robot->stop();
    } catch (const std::exception &) {
    }
    exit_code = 99;
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return exit_code;
}
