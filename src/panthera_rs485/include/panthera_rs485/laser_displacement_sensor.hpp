#ifndef PANTHERA_RS485__LASER_DISPLACEMENT_SENSOR_HPP_
#define PANTHERA_RS485__LASER_DISPLACEMENT_SENSOR_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "panthera_rs485/modbus_rtu.hpp"

namespace panthera_rs485
{

struct LaserDistanceResult
{
  bool valid{false};
  double distance_mm{0.0};
  std::vector<uint16_t> raw_registers;
  std::string status;
};

class LaserDisplacementSensor
{
public:
  LaserDisplacementSensor(
    std::shared_ptr<ModbusRtuMaster> modbus,
    uint8_t slave_id,
    uint16_t distance_register,
    const std::string & decode_mode,
    double scale,
    const std::string & unit);

  LaserDistanceResult readDistance();

  uint8_t slaveId() const { return slave_id_; }
  const std::string & decodeMode() const { return decode_mode_; }

private:
  double decodeDistanceMm(const std::vector<uint16_t> & registers) const;

  std::shared_ptr<ModbusRtuMaster> modbus_;
  uint8_t slave_id_;
  uint16_t distance_register_;
  std::string decode_mode_;
  double scale_;
  std::string unit_;
};

}  // namespace panthera_rs485

#endif  // PANTHERA_RS485__LASER_DISPLACEMENT_SENSOR_HPP_
