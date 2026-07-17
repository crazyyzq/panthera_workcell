#pragma once

#include <chrono>
#include <memory>
#include <random>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "panthera_spectrometer_cell/Config.h"
#include "panthera_spectrometer_cell/Types.h"

namespace panthera_spectrometer_cell
{

class Spectrometer
{
public:
  Spectrometer(rclcpp::Node::SharedPtr node, WorkcellConfig config);

  ActionResult initialize();
  void updateConfig(const WorkcellConfig & config);
  ActionResult startDetection();
  ActionResult pollDetectionDone(bool & done);
  ActionResult waitDetectionDone(std::chrono::milliseconds timeout);

private:
  void setupManualServices();
  bool shouldSimulateFailure();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  WorkcellConfig config_;

  bool detection_active_{false};
  std::atomic_bool manual_detection_done_{false};
  std::chrono::steady_clock::time_point detection_start_time_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_detection_done_srv_;

  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_dist_{0.0, 1.0};
};

}  // namespace panthera_spectrometer_cell
