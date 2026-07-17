#ifndef PANTHERA_RS485__MODBUS_RTU_HPP_
#define PANTHERA_RS485__MODBUS_RTU_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "panthera_rs485/serial_port.hpp"

namespace panthera_rs485
{

class ModbusException : public std::runtime_error
{
public:
  explicit ModbusException(const std::string & message);
};

class ModbusRtuMaster
{
public:
  ModbusRtuMaster(
    const std::string & port,
    int baudrate,
    std::chrono::milliseconds timeout);

  std::vector<uint16_t> readInputRegisters(
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t register_count);

  std::vector<uint16_t> readHoldingRegisters(
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t register_count);

  void writeMultipleRegisters(
    uint8_t slave_id,
    uint16_t start_address,
    const std::vector<uint16_t> & values);

  static uint16_t crc16(const std::vector<uint8_t> & data);
  static std::string toHex(const std::vector<uint8_t> & data);

private:
  std::vector<uint8_t> transact(
    const std::vector<uint8_t> & request_without_crc,
    size_t expected_response_size);

  static void appendU16(std::vector<uint8_t> & data, uint16_t value);
  static uint16_t readU16(const std::vector<uint8_t> & data, size_t offset);

  SerialPort serial_;
  std::chrono::milliseconds timeout_;
  std::mutex mutex_;
};

}  // namespace panthera_rs485

#endif  // PANTHERA_RS485__MODBUS_RTU_HPP_
