#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <panthera_interfaces/msg/external_signal.hpp>
#include <panthera_interfaces/srv/run_workflow.hpp>
#include <rclcpp/rclcpp.hpp>

#include "panthera_rs485/modbus_rtu.hpp"

namespace
{

int declareIntInRange(
  rclcpp::Node & node,
  const std::string & name,
  int default_value,
  int min_value,
  int max_value)
{
  const int value = node.declare_parameter<int>(name, default_value);
  if (value < min_value || value > max_value) {
    throw std::runtime_error(
            name + " must be in range " + std::to_string(min_value) +
            ".." + std::to_string(max_value));
  }
  return value;
}

struct StateMapping
{
  std::string workflow_name;
  std::string detail;
};

std::vector<std::string> splitMappingEntry(const std::string & entry)
{
  std::vector<std::string> parts;
  std::string current;
  for (const char c : entry) {
    if (c == ':') {
      parts.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  parts.push_back(current);
  return parts;
}

std::map<int32_t, StateMapping> parseStateMap(const std::vector<std::string> & entries)
{
  std::map<int32_t, StateMapping> parsed;
  for (const auto & entry : entries) {
    if (entry.empty()) {
      continue;
    }

    const auto parts = splitMappingEntry(entry);
    if (parts.size() < 2 || parts[0].empty() || parts[1].empty()) {
      throw std::runtime_error(
              "state_map entries must use 'code:workflow_name[:detail]', got: " + entry);
    }

    StateMapping mapping;
    mapping.workflow_name = parts[1];
    if (parts.size() >= 3) {
      mapping.detail = parts[2];
      for (size_t i = 3; i < parts.size(); ++i) {
        mapping.detail += ":" + parts[i];
      }
    }
    parsed[static_cast<int32_t>(std::stol(parts[0]))] = mapping;
  }
  return parsed;
}

}  // namespace

class Rs485StateSignalNode : public rclcpp::Node
{
public:
  Rs485StateSignalNode()
  : Node("rs485_state_signal_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyS3");
    baudrate_ = declare_parameter<int>("baudrate", 9600);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 200);
    slave_id_ = static_cast<uint8_t>(declareIntInRange(*this, "slave_id", 1, 1, 247));
    register_address_ = static_cast<uint16_t>(
      declareIntInRange(*this, "register_address", 0, 0, 65535));
    register_type_ = declare_parameter<std::string>("register_type", "holding");
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 10.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 2.0);
    reopen_after_failures_ = declareIntInRange(*this, "reopen_after_failures", 5, 1, 1000000);
    source_ = declare_parameter<std::string>("source", "rs485_process_state");
    signal_name_ = declare_parameter<std::string>("signal_name", "station_state");
    active_nonzero_ = declare_parameter<bool>("active_nonzero", true);
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/workflow/external_signal");
    auto_run_workflow_ = declare_parameter<bool>("auto_run_workflow", false);
    trigger_on_change_ = declare_parameter<bool>("trigger_on_change", true);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    workflow_service_ = declare_parameter<std::string>("workflow_service", "/run_workflow");
    min_trigger_interval_sec_ = declare_parameter<double>("min_trigger_interval_sec", 1.0);

    state_map_ = parseStateMap(
      declare_parameter<std::vector<std::string>>(
        "state_map",
        std::vector<std::string>{"1:cup_pick_place:object_ready"}));

    if (register_type_ != "holding" && register_type_ != "input") {
      throw std::runtime_error("register_type must be 'holding' or 'input'");
    }

    signal_pub_ = create_publisher<panthera_interfaces::msg::ExternalSignal>(
      publish_topic_,
      10);
    workflow_client_ = create_client<panthera_interfaces::srv::RunWorkflow>(workflow_service_);

    if (poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("poll_rate_hz must be greater than zero");
    }

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Rs485StateSignalNode::pollOnce, this));

    ensureConnected();

    RCLCPP_INFO(
      get_logger(),
      "RS485 state signal node started: port=%s baud=%d slave=%u register=0x%04X type=%s auto_run=%s",
      port_.c_str(),
      baudrate_,
      static_cast<unsigned>(slave_id_),
      static_cast<unsigned>(register_address_),
      register_type_.c_str(),
      auto_run_workflow_ ? "true" : "false");
  }

private:
  bool reconnectIntervalElapsed() const
  {
    if (!has_last_connect_attempt_) {
      return true;
    }
    return (now() - last_connect_attempt_time_).seconds() >= reconnect_interval_sec_;
  }

  bool connectModbus()
  {
    last_connect_attempt_time_ = now();
    has_last_connect_attempt_ = true;

    try {
      modbus_ = std::make_shared<panthera_rs485::ModbusRtuMaster>(
        port_,
        baudrate_,
        std::chrono::milliseconds(timeout_ms_));
      consecutive_failures_ = 0;
      RCLCPP_INFO(get_logger(), "connected RS485 state port: %s", port_.c_str());
      return true;
    } catch (const std::exception & e) {
      modbus_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "RS485 state connect failed on %s: %s",
        port_.c_str(),
        e.what());
      return false;
    }
  }

  bool ensureConnected()
  {
    if (modbus_) {
      return true;
    }
    if (!reconnectIntervalElapsed()) {
      return false;
    }
    return connectModbus();
  }

  void resetConnection(const std::string & reason)
  {
    if (modbus_) {
      RCLCPP_WARN(get_logger(), "reset RS485 state connection: %s", reason.c_str());
    }
    modbus_.reset();
  }

  void updateConnectionHealth(bool success, const std::string & reason)
  {
    if (success) {
      consecutive_failures_ = 0;
      return;
    }

    ++consecutive_failures_;
    if (consecutive_failures_ >= reopen_after_failures_) {
      resetConnection(reason);
      consecutive_failures_ = 0;
    }
  }

  int32_t readStateCode()
  {
    if (!ensureConnected()) {
      throw std::runtime_error("RS485 state port is not connected");
    }

    std::vector<uint16_t> values;
    if (register_type_ == "holding") {
      values = modbus_->readHoldingRegisters(slave_id_, register_address_, 1);
    } else {
      values = modbus_->readInputRegisters(slave_id_, register_address_, 1);
    }

    if (values.empty()) {
      throw std::runtime_error("empty Modbus state response");
    }
    return static_cast<int32_t>(values[0]);
  }

  panthera_interfaces::msg::ExternalSignal buildSignal(
    int32_t code,
    bool active,
    const std::string & workflow_name,
    const std::string & detail)
  {
    panthera_interfaces::msg::ExternalSignal msg;
    msg.header.stamp = now();
    msg.header.frame_id = source_;
    msg.source = source_;
    msg.name = signal_name_;
    msg.code = code;
    msg.active = active;
    msg.workflow_name = workflow_name;
    msg.detail = detail;
    return msg;
  }

  void maybeTriggerWorkflow(const panthera_interfaces::msg::ExternalSignal & signal)
  {
    if (!auto_run_workflow_ || !signal.active || signal.workflow_name.empty()) {
      return;
    }

    const bool code_changed = !has_last_code_ || signal.code != last_code_;
    if (trigger_on_change_ && !code_changed) {
      return;
    }

    const auto now_time = now();
    if (has_last_trigger_time_ &&
      (now_time - last_trigger_time_).seconds() < min_trigger_interval_sec_)
    {
      return;
    }

    if (!workflow_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "workflow service is not ready: %s",
        workflow_service_.c_str());
      return;
    }

    auto request = std::make_shared<panthera_interfaces::srv::RunWorkflow::Request>();
    request->workflow_name = signal.workflow_name;
    request->dry_run = dry_run_;
    workflow_client_->async_send_request(request);
    last_trigger_time_ = now_time;
    has_last_trigger_time_ = true;

    RCLCPP_INFO(
      get_logger(),
      "triggered workflow '%s' from RS485 state code %d",
      signal.workflow_name.c_str(),
      signal.code);
  }

  void pollOnce()
  {
    try {
      const int32_t code = readStateCode();
      const auto mapping_it = state_map_.find(code);
      const bool active = active_nonzero_ ? code != 0 : true;
      const std::string workflow_name =
        mapping_it == state_map_.end() ? "" : mapping_it->second.workflow_name;
      const std::string detail =
        mapping_it == state_map_.end() ? "unmapped" : mapping_it->second.detail;

      const auto signal = buildSignal(code, active, workflow_name, detail);
      signal_pub_->publish(signal);
      maybeTriggerWorkflow(signal);
      updateConnectionHealth(true, "");

      last_code_ = code;
      has_last_code_ = true;
    } catch (const std::exception & e) {
      const auto signal = buildSignal(-1, false, "", e.what());
      signal_pub_->publish(signal);
      updateConnectionHealth(false, e.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "RS485 state read failed: %s",
        e.what());
    }
  }

  std::string port_;
  int baudrate_{9600};
  int timeout_ms_{200};
  uint8_t slave_id_{1};
  uint16_t register_address_{0};
  std::string register_type_;
  double poll_rate_hz_{10.0};
  double reconnect_interval_sec_{2.0};
  int reopen_after_failures_{5};
  std::string source_;
  std::string signal_name_;
  bool active_nonzero_{true};
  std::string publish_topic_;
  bool auto_run_workflow_{false};
  bool trigger_on_change_{true};
  bool dry_run_{false};
  std::string workflow_service_;
  double min_trigger_interval_sec_{1.0};

  std::map<int32_t, StateMapping> state_map_;
  std::shared_ptr<panthera_rs485::ModbusRtuMaster> modbus_;
  int consecutive_failures_{0};
  bool has_last_connect_attempt_{false};
  rclcpp::Time last_connect_attempt_time_;
  rclcpp::Publisher<panthera_interfaces::msg::ExternalSignal>::SharedPtr signal_pub_;
  rclcpp::Client<panthera_interfaces::srv::RunWorkflow>::SharedPtr workflow_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool has_last_code_{false};
  int32_t last_code_{0};
  bool has_last_trigger_time_{false};
  rclcpp::Time last_trigger_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rs485StateSignalNode>());
  rclcpp::shutdown();
  return 0;
}
