#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <panthera_interfaces/msg/laser_distance.hpp>
#include <panthera_interfaces/srv/get_laser_distance.hpp>
#include <rclcpp/rclcpp.hpp>

#include "panthera_rs485/laser_displacement_sensor.hpp"

using namespace std::chrono_literals;

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

std::vector<double> normalizedOrigin(const std::vector<double> & value)
{
  if (value.size() != 3) {
    throw std::runtime_error("pose_origin_xyz must contain exactly 3 numbers");
  }
  return value;
}

void applyDistanceAxis(
  geometry_msgs::msg::PoseStamped & pose,
  const std::string & axis,
  double distance_m)
{
  if (axis == "x" || axis == "+x") {
    pose.pose.position.x += distance_m;
  } else if (axis == "-x") {
    pose.pose.position.x -= distance_m;
  } else if (axis == "y" || axis == "+y") {
    pose.pose.position.y += distance_m;
  } else if (axis == "-y") {
    pose.pose.position.y -= distance_m;
  } else if (axis == "z" || axis == "+z") {
    pose.pose.position.z += distance_m;
  } else if (axis == "-z") {
    pose.pose.position.z -= distance_m;
  } else {
    throw std::runtime_error("pose_axis must be one of x, -x, y, -y, z, -z");
  }
}

}  // namespace

class LaserDistanceNode : public rclcpp::Node
{
public:
  LaserDistanceNode()
  : Node("laser_distance_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyS4");
    baudrate_ = declare_parameter<int>("baudrate", 9600);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 200);
    slave_id_ = static_cast<uint8_t>(declareIntInRange(*this, "slave_id", 1, 1, 247));
    distance_register_ = static_cast<uint16_t>(
      declareIntInRange(*this, "distance_register", 0, 0, 65535));
    decode_mode_ = declare_parameter<std::string>("decode_mode", "uint32_abcd");
    scale_ = declare_parameter<double>("scale", 0.1);
    offset_mm_ = declare_parameter<double>("offset_mm", 0.0);
    unit_ = declare_parameter<std::string>("unit", "mm");
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 5.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 2.0);
    reopen_after_failures_ = declareIntInRange(*this, "reopen_after_failures", 5, 1, 1000000);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    source_ = declare_parameter<std::string>("source", "laser_displacement");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/sensors/laser/distance");

    publish_pose_ = declare_parameter<bool>("publish_pose", false);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/laser/object_pose");
    pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "base_link");
    pose_axis_ = declare_parameter<std::string>("pose_axis", "z");
    pose_origin_xyz_ = normalizedOrigin(
      declare_parameter<std::vector<double>>("pose_origin_xyz", {0.0, 0.0, 0.0}));

    distance_pub_ = create_publisher<panthera_interfaces::msg::LaserDistance>(
      distance_topic_,
      rclcpp::SensorDataQoS());

    if (publish_pose_) {
      pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    }

    read_once_srv_ = create_service<panthera_interfaces::srv::GetLaserDistance>(
      "~/read_once",
      std::bind(
        &LaserDistanceNode::readOnceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    if (poll_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&LaserDistanceNode::pollOnce, this));
    }

    ensureConnected();

    RCLCPP_INFO(
      get_logger(),
      "laser RS485 node started: port=%s baud=%d slave=%u register=0x%04X decode=%s poll=%.2fHz",
      port_.c_str(),
      baudrate_,
      static_cast<unsigned>(slave_id_),
      static_cast<unsigned>(distance_register_),
      decode_mode_.c_str(),
      poll_rate_hz_);
    if (std::abs(offset_mm_) > 1e-9) {
      RCLCPP_WARN(
        get_logger(),
        "laser distance calibration offset is active: %.3f mm", offset_mm_);
    }
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
      auto modbus = std::make_shared<panthera_rs485::ModbusRtuMaster>(
        port_,
        baudrate_,
        std::chrono::milliseconds(timeout_ms_));
      auto sensor = std::make_unique<panthera_rs485::LaserDisplacementSensor>(
        modbus,
        slave_id_,
        distance_register_,
        decode_mode_,
        scale_,
        unit_);

      modbus_ = modbus;
      sensor_ = std::move(sensor);
      consecutive_failures_ = 0;
      RCLCPP_INFO(get_logger(), "connected laser RS485 port: %s", port_.c_str());
      return true;
    } catch (const std::exception & e) {
      modbus_.reset();
      sensor_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "laser RS485 connect failed on %s: %s",
        port_.c_str(),
        e.what());
      return false;
    }
  }

  bool ensureConnected()
  {
    if (modbus_ && sensor_) {
      return true;
    }
    if (!reconnectIntervalElapsed()) {
      return false;
    }
    return connectModbus();
  }

  void resetConnection(const std::string & reason)
  {
    if (modbus_ || sensor_) {
      RCLCPP_WARN(get_logger(), "reset laser RS485 connection: %s", reason.c_str());
    }
    sensor_.reset();
    modbus_.reset();
  }

  void updateConnectionHealth(const panthera_rs485::LaserDistanceResult & result)
  {
    if (result.valid) {
      consecutive_failures_ = 0;
      return;
    }

    ++consecutive_failures_;
    if (consecutive_failures_ >= reopen_after_failures_) {
      resetConnection(result.status);
      consecutive_failures_ = 0;
    }
  }

  panthera_rs485::LaserDistanceResult readDistance(uint8_t slave_id)
  {
    if (!ensureConnected()) {
      panthera_rs485::LaserDistanceResult result;
      result.valid = false;
      result.status = "laser RS485 port is not connected";
      return result;
    }

    if (slave_id == slave_id_) {
      return sensor_->readDistance();
    }

    panthera_rs485::LaserDisplacementSensor temporary_sensor(
      modbus_,
      slave_id,
      distance_register_,
      decode_mode_,
      scale_,
      unit_);
    return temporary_sensor.readDistance();
  }

  panthera_interfaces::msg::LaserDistance makeMessage(
    uint8_t slave_id,
    const panthera_rs485::LaserDistanceResult & result)
  {
    panthera_interfaces::msg::LaserDistance msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.source = source_;
    msg.slave_id = slave_id;
    msg.valid = result.valid;
    const double corrected_distance_mm = result.distance_mm + offset_mm_;
    msg.distance_mm = corrected_distance_mm;
    msg.distance_m = corrected_distance_mm / 1000.0;
    msg.raw_registers = result.raw_registers;
    msg.decode_mode = decode_mode_;
    if (result.valid && std::abs(offset_mm_) > 1e-9) {
      msg.status = result.status + " calibrated_offset_mm=" + std::to_string(offset_mm_);
    } else {
      msg.status = result.status;
    }
    return msg;
  }

  void publishPoseFromDistance(const panthera_interfaces::msg::LaserDistance & distance)
  {
    if (!publish_pose_ || !pose_pub_ || !distance.valid) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = distance.header.stamp;
    pose.header.frame_id = pose_frame_id_;
    pose.pose.position.x = pose_origin_xyz_[0];
    pose.pose.position.y = pose_origin_xyz_[1];
    pose.pose.position.z = pose_origin_xyz_[2];
    pose.pose.orientation.w = 1.0;

    try {
      applyDistanceAxis(pose, pose_axis_, distance.distance_m);
      pose_pub_->publish(pose);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "failed to publish laser pose: %s",
        e.what());
    }
  }

  void pollOnce()
  {
    const auto result = readDistance(slave_id_);
    updateConnectionHealth(result);
    auto msg = makeMessage(slave_id_, result);
    distance_pub_->publish(msg);
    publishPoseFromDistance(msg);

    if (!result.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "laser read failed: %s",
        result.status.c_str());
    }
  }

  void readOnceCallback(
    const std::shared_ptr<panthera_interfaces::srv::GetLaserDistance::Request> request,
    std::shared_ptr<panthera_interfaces::srv::GetLaserDistance::Response> response)
  {
    const uint8_t request_slave = request->slave_id == 0 ? slave_id_ : request->slave_id;
    if (request_slave == 0 || request_slave > 247) {
      response->success = false;
      response->message = "slave_id must be 0(default) or 1..247";
      return;
    }

    const auto result = readDistance(request_slave);
    updateConnectionHealth(result);
    const auto msg = makeMessage(request_slave, result);

    response->success = result.valid;
    response->distance_mm = msg.distance_mm;
    response->distance_m = msg.distance_m;
    response->raw_registers = msg.raw_registers;
    response->message = msg.status;

    distance_pub_->publish(msg);
    publishPoseFromDistance(msg);
  }

  std::string port_;
  int baudrate_{9600};
  int timeout_ms_{200};
  uint8_t slave_id_{1};
  uint16_t distance_register_{0};
  std::string decode_mode_;
  double scale_{0.1};
  double offset_mm_{0.0};
  std::string unit_;
  double poll_rate_hz_{5.0};
  double reconnect_interval_sec_{2.0};
  int reopen_after_failures_{5};
  std::string frame_id_;
  std::string source_;
  std::string distance_topic_;

  bool publish_pose_{false};
  std::string pose_topic_;
  std::string pose_frame_id_;
  std::string pose_axis_;
  std::vector<double> pose_origin_xyz_;

  std::shared_ptr<panthera_rs485::ModbusRtuMaster> modbus_;
  std::unique_ptr<panthera_rs485::LaserDisplacementSensor> sensor_;
  int consecutive_failures_{0};
  bool has_last_connect_attempt_{false};
  rclcpp::Time last_connect_attempt_time_;
  rclcpp::Publisher<panthera_interfaces::msg::LaserDistance>::SharedPtr distance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Service<panthera_interfaces::srv::GetLaserDistance>::SharedPtr read_once_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserDistanceNode>());
  rclcpp::shutdown();
  return 0;
}
