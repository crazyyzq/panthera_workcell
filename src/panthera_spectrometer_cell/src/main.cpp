#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "panthera_interfaces/srv/set_speed_scale.hpp"
#include "panthera_spectrometer_cell/Config.h"
#include "panthera_spectrometer_cell/RobotActions.h"
#include "panthera_spectrometer_cell/Sensors.h"
#include "panthera_spectrometer_cell/Spectrometer.h"
#include "panthera_spectrometer_cell/StateMachine.h"

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

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "panthera_spectrometer_cell",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const std::string config_file =
    getOrDeclareParameter<std::string>(node, "config_file", defaultConfigPath());
  const bool simulation_enabled =
    getOrDeclareParameter<bool>(node, "simulation_enabled", true);

  panthera_spectrometer_cell::WorkcellConfig config;
  try {
    config = panthera_spectrometer_cell::WorkcellConfig::loadFromFile(config_file);
    config.simulation.enabled = simulation_enabled;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "failed to load config '%s': %s", config_file.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "starting panthera spectrometer cell: config=%s simulation=%s tick_rate=%.2fHz",
    config_file.c_str(),
    config.simulation.enabled ? "true" : "false",
    config.loop.tickRateHz);

  auto robot = std::make_shared<panthera_spectrometer_cell::RobotActions>(node, config);
  auto sensors = std::make_shared<panthera_spectrometer_cell::Sensors>(node, config);
  auto spectrometer = std::make_shared<panthera_spectrometer_cell::Spectrometer>(node, config);
  auto state_machine = std::make_shared<panthera_spectrometer_cell::StateMachine>(
    node,
    config,
    robot,
    sensors,
    spectrometer);

  auto speed_scale_service = node->create_service<panthera_interfaces::srv::SetSpeedScale>(
    "/spectrometer_cell/set_speed_scale",
    [robot](
      const std::shared_ptr<panthera_interfaces::srv::SetSpeedScale::Request> request,
      std::shared_ptr<panthera_interfaces::srv::SetSpeedScale::Response> response) {
      const auto result = robot->setSpeedScale(request->scale);
      response->success = result.success;
      response->message = result.message;
      response->applied_scale = robot->speedScale();
    });

  auto stop_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto stop_motion_service = node->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/stop_motion",
    [robot](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto result = robot->stop();
      response->success = result.success;
      response->message = result.message;
    },
    rmw_qos_profile_services_default,
    stop_callback_group);

  auto reload_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto reload_config_service = node->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/reload_config",
    [node, config_file, simulation_enabled, state_machine](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      try {
        auto reloaded_config =
          panthera_spectrometer_cell::WorkcellConfig::loadFromFile(config_file);
        reloaded_config.simulation.enabled = simulation_enabled;
        const auto result = state_machine->reloadConfig(reloaded_config);
        response->success = result.success;
        response->message = result.message;
        if (result.success) {
          RCLCPP_WARN(
            node->get_logger(),
            "reloaded spectrometer cell config from %s",
            config_file.c_str());
        } else {
          RCLCPP_WARN(
            node->get_logger(),
            "rejected config reload from %s: %s",
            config_file.c_str(),
            result.message.c_str());
        }
      } catch (const std::exception & e) {
        response->success = false;
        response->message = std::string("reload config failed: ") + e.what();
        RCLCPP_ERROR(node->get_logger(), "%s", response->message.c_str());
      }
    },
    rmw_qos_profile_services_default,
    reload_callback_group);

  const auto tick_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / config.loop.tickRateHz));

  auto timer = node->create_wall_timer(
    tick_period > 0ms ? tick_period : 100ms,
    [state_machine]() {
      state_machine->tick();
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);

  robot->stop();
  rclcpp::shutdown();
  return 0;
}
