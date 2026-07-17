#include "panthera_rs485/laser_displacement_sensor.hpp"

#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace panthera_rs485
{

namespace
{

std::array<uint8_t, 4> registerBytesAbcd(const std::vector<uint16_t> & registers)
{
  return {
    static_cast<uint8_t>((registers[0] >> 8) & 0xFF),
    static_cast<uint8_t>(registers[0] & 0xFF),
    static_cast<uint8_t>((registers[1] >> 8) & 0xFF),
    static_cast<uint8_t>(registers[1] & 0xFF)};
}

std::array<uint8_t, 4> registerBytesCdab(const std::vector<uint16_t> & registers)
{
  return {
    static_cast<uint8_t>((registers[1] >> 8) & 0xFF),
    static_cast<uint8_t>(registers[1] & 0xFF),
    static_cast<uint8_t>((registers[0] >> 8) & 0xFF),
    static_cast<uint8_t>(registers[0] & 0xFF)};
}

uint32_t bytesToU32(const std::array<uint8_t, 4> & bytes)
{
  return
    (static_cast<uint32_t>(bytes[0]) << 24) |
    (static_cast<uint32_t>(bytes[1]) << 16) |
    (static_cast<uint32_t>(bytes[2]) << 8) |
    static_cast<uint32_t>(bytes[3]);
}

float bytesToFloat(const std::array<uint8_t, 4> & bytes)
{
  const uint32_t bits = bytesToU32(bytes);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

LaserDisplacementSensor::LaserDisplacementSensor(
  std::shared_ptr<ModbusRtuMaster> modbus,
  uint8_t slave_id,
  uint16_t distance_register,
  const std::string & decode_mode,
  double scale,
  const std::string & unit)
: modbus_(std::move(modbus)),
  slave_id_(slave_id),
  distance_register_(distance_register),
  decode_mode_(decode_mode),
  scale_(scale),
  unit_(unit)
{
}

LaserDistanceResult LaserDisplacementSensor::readDistance()
{
  LaserDistanceResult result;
  try {
    result.raw_registers = modbus_->readInputRegisters(slave_id_, distance_register_, 2);
    result.distance_mm = decodeDistanceMm(result.raw_registers);
    result.valid = true;
    result.status = "ok";
  } catch (const std::exception & e) {
    result.valid = false;
    result.status = e.what();
  }
  return result;
}

double LaserDisplacementSensor::decodeDistanceMm(
  const std::vector<uint16_t> & registers) const
{
  if (registers.size() != 2) {
    throw std::runtime_error("Laser distance requires exactly 2 registers");
  }

  double value = 0.0;
  if (decode_mode_ == "float32_abcd") {
    value = bytesToFloat(registerBytesAbcd(registers));
  } else if (decode_mode_ == "float32_cdab") {
    value = bytesToFloat(registerBytesCdab(registers));
  } else if (decode_mode_ == "uint32_abcd") {
    value = static_cast<double>(bytesToU32(registerBytesAbcd(registers))) * scale_;
  } else if (decode_mode_ == "uint32_cdab") {
    value = static_cast<double>(bytesToU32(registerBytesCdab(registers))) * scale_;
  } else if (decode_mode_ == "int32_abcd") {
    value = static_cast<double>(static_cast<int32_t>(bytesToU32(registerBytesAbcd(registers)))) * scale_;
  } else if (decode_mode_ == "int32_cdab") {
    value = static_cast<double>(static_cast<int32_t>(bytesToU32(registerBytesCdab(registers)))) * scale_;
  } else {
    throw std::runtime_error("Unsupported laser decode_mode: " + decode_mode_);
  }

  if (unit_ == "m") {
    return value * 1000.0;
  }
  if (unit_ == "cm") {
    return value * 10.0;
  }
  return value;
}

}  // namespace panthera_rs485
