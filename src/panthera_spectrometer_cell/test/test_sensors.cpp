#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "panthera_interfaces/msg/laser_distance.hpp"
#include "panthera_spectrometer_cell/Sensors.h"

namespace
{

using panthera_spectrometer_cell::Sensors;
using panthera_spectrometer_cell::WorkcellConfig;

WorkcellConfig optionalSensorConfig()
{
  WorkcellConfig config;
  config.simulation.enabled = false;
  config.positioning.mode = "sensor_optional";
  config.positioning.fixedAxisPositionMm = 150.0;
  config.spectrometerAxis.laserMinMm = 120.0;
  config.spectrometerAxis.laserMaxMm = 280.0;
  config.loop.sensorTimeoutSec = 0.2;
  return config;
}

void publishAndDeliver(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<panthera_interfaces::msg::LaserDistance>::SharedPtr & publisher,
  const panthera_interfaces::msg::LaserDistance & message)
{
  for (int attempt = 0; attempt < 20; ++attempt) {
    publisher->publish(message);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

class SensorsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(SensorsTest, OptionalModeFallsBackWhenNoLaserIsConnected)
{
  auto node = std::make_shared<rclcpp::Node>("sensor_optional_missing_test");
  Sensors sensors(node, optionalSensorConfig());

  const auto result = sensors.readSpectrometerPosition();

  EXPECT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.value, 150.0);
}

TEST_F(SensorsTest, OptionalModeUsesFreshValidLaser)
{
  auto node = std::make_shared<rclcpp::Node>("sensor_optional_valid_test");
  Sensors sensors(node, optionalSensorConfig());
  ASSERT_TRUE(sensors.initialize().success);
  auto publisher =
    node->create_publisher<panthera_interfaces::msg::LaserDistance>(
    "/sensors/laser/distance", rclcpp::SensorDataQoS());
  panthera_interfaces::msg::LaserDistance message;
  message.valid = true;
  message.distance_mm = 162.5;
  message.status = "test";
  publishAndDeliver(node, publisher, message);

  const auto result = sensors.readSpectrometerPosition();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.ready);
  EXPECT_DOUBLE_EQ(result.value, 162.5);

  sensors.beginSpectrometerMeasurement();
  EXPECT_FALSE(sensors.readSpectrometerPosition().ready);
  publishAndDeliver(node, publisher, message);
  EXPECT_TRUE(sensors.readSpectrometerPosition().ready);
}

TEST_F(SensorsTest, OptionalModeRejectsNonFiniteAndOutOfRangeLaser)
{
  auto node = std::make_shared<rclcpp::Node>("sensor_optional_invalid_test");
  Sensors sensors(node, optionalSensorConfig());
  ASSERT_TRUE(sensors.initialize().success);
  auto publisher =
    node->create_publisher<panthera_interfaces::msg::LaserDistance>(
    "/sensors/laser/distance", rclcpp::SensorDataQoS());
  panthera_interfaces::msg::LaserDistance message;
  message.valid = true;
  message.distance_mm = std::numeric_limits<double>::quiet_NaN();
  message.status = "test_nan";
  publishAndDeliver(node, publisher, message);
  EXPECT_DOUBLE_EQ(sensors.readSpectrometerPosition().value, 150.0);

  message.distance_mm = 999.0;
  message.status = "test_out_of_range";
  publishAndDeliver(node, publisher, message);
  const auto result = sensors.readSpectrometerPosition();

  EXPECT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.value, 150.0);
}

}  // namespace
