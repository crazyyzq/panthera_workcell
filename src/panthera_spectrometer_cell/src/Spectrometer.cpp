#include "panthera_spectrometer_cell/Spectrometer.h"

#include <chrono>
#include <thread>
#include <utility>

namespace panthera_spectrometer_cell
{

Spectrometer::Spectrometer(rclcpp::Node::SharedPtr node, WorkcellConfig config)
: node_(std::move(node)),
  logger_(node_->get_logger().get_child("spectrometer")),
  config_(std::move(config)),
  rng_(config_.simulation.randomSeed + 37)
{
}

ActionResult Spectrometer::initialize()
{
  RCLCPP_INFO(
    logger_,
    "initialize spectrometer adapter, simulation=%s",
    config_.simulation.enabled ? "true" : "false");
  setupManualServices();
  // Real hardware integration point: replace the manual service flag with IO/RS485/TCP done status.
  return ActionResult::ok("spectrometer initialized");
}

void Spectrometer::updateConfig(const WorkcellConfig & config)
{
  config_ = config;
  RCLCPP_WARN(logger_, "spectrometer config reloaded");
}

ActionResult Spectrometer::startDetection()
{
  if (shouldSimulateFailure()) {
    return ActionResult::fail("startDetection failed: simulated spectrometer start failure");
  }

  detection_active_ = true;
  manual_detection_done_.store(false);
  detection_start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "spectrometer detection started");
  // TODO(real hardware): optionally send start-detection command here.
  return ActionResult::ok("spectrometer detection started");
}

ActionResult Spectrometer::pollDetectionDone(bool & done)
{
  done = false;

  if (!detection_active_) {
    return ActionResult::fail("pollDetectionDone failed: detection was not started");
  }

  if (shouldSimulateFailure()) {
    return ActionResult::fail("pollDetectionDone failed: simulated spectrometer failure");
  }

  if (config_.simulation.enabled) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - detection_start_time_).count();
    if (elapsed >= config_.simulation.fakeDetectionTimeSec) {
      detection_active_ = false;
      done = true;
      return ActionResult::ok("simulated spectrometer detection done");
    }
    return ActionResult::ok("simulated spectrometer still detecting");
  }

  if (manual_detection_done_.exchange(false)) {
    detection_active_ = false;
    done = true;
    return ActionResult::ok("manual spectrometer detection done");
  }

  return ActionResult::ok("real spectrometer waiting for manual/IO done signal");
}

ActionResult Spectrometer::waitDetectionDone(std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    bool done = false;
    const auto result = pollDetectionDone(done);
    if (!result.success) {
      return result;
    }
    if (done) {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return ActionResult::fail("waitDetectionDone failed: timeout");
}

void Spectrometer::setupManualServices()
{
  manual_detection_done_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/simulate_detection_done",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      manual_detection_done_.store(true);
      response->success = true;
      response->message = "manual spectrometer detection done queued";
      RCLCPP_WARN(logger_, "%s", response->message.c_str());
    });
}

bool Spectrometer::shouldSimulateFailure()
{
  if (!config_.simulation.enabled || config_.simulation.randomFailureRate <= 0.0) {
    return false;
  }
  return unit_dist_(rng_) < config_.simulation.randomFailureRate;
}

}  // namespace panthera_spectrometer_cell
