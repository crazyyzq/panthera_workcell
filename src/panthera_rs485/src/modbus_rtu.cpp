#include "panthera_rs485/modbus_rtu.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace panthera_rs485
{

ModbusException::ModbusException(const std::string & message)
: std::runtime_error(message)
{
}

ModbusRtuMaster::ModbusRtuMaster(
  const std::string & port,
  int baudrate,
  std::chrono::milliseconds timeout,
  SerialParity parity,
  int stop_bits)
: timeout_(timeout)
{
  serial_.open(port, baudrate, timeout, parity, stop_bits);
}

uint16_t ModbusRtuMaster::crc16(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (const uint8_t byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::string ModbusRtuMaster::toHex(const std::vector<uint8_t> & data)
{
  std::ostringstream out;
  out << std::hex << std::uppercase << std::setfill('0');
  for (size_t i = 0; i < data.size(); ++i) {
    if (i > 0) {
      out << ' ';
    }
    out << "0x" << std::setw(2) << static_cast<int>(data[i]);
  }
  return out.str();
}

void ModbusRtuMaster::appendU16(std::vector<uint8_t> & data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  data.push_back(static_cast<uint8_t>(value & 0xFF));
}

uint16_t ModbusRtuMaster::readU16(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1]);
}

std::vector<uint8_t> ModbusRtuMaster::transact(
  const std::vector<uint8_t> & request_without_crc,
  size_t expected_response_size)
{
  if (request_without_crc.size() < 2) {
    throw ModbusException("Modbus request must contain at least slave id and function code");
  }

  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<uint8_t> request = request_without_crc;
  const uint16_t request_crc = crc16(request);
  request.push_back(static_cast<uint8_t>(request_crc & 0xFF));
  request.push_back(static_cast<uint8_t>((request_crc >> 8) & 0xFF));

  serial_.flushInput();
  serial_.writeAll(request);

  auto response = serial_.readExact(5, timeout_);
  if (response.size() >= 2 &&
    response[1] == static_cast<uint8_t>(request_without_crc[1] | 0x80))
  {
    const uint16_t received_crc =
      static_cast<uint16_t>(response[response.size() - 2]) |
      static_cast<uint16_t>(response[response.size() - 1] << 8);
    auto response_without_crc = response;
    response_without_crc.resize(response_without_crc.size() - 2);
    const uint16_t computed_crc = crc16(response_without_crc);
    if (received_crc != computed_crc) {
      throw ModbusException("CRC mismatch in Modbus exception response: " + toHex(response));
    }
    if (response_without_crc[0] != request_without_crc[0]) {
      throw ModbusException("Unexpected slave id in Modbus exception response: " + toHex(response));
    }
    const uint8_t code = response.size() > 2 ? response[2] : 0;
    throw ModbusException("Modbus exception code: " + std::to_string(code));
  }

  if (expected_response_size > response.size()) {
    auto rest = serial_.readExact(expected_response_size - response.size(), timeout_);
    response.insert(response.end(), rest.begin(), rest.end());
  }

  if (response.size() < 5) {
    throw ModbusException("Short Modbus response: " + toHex(response));
  }

  const uint16_t received_crc =
    static_cast<uint16_t>(response[response.size() - 2]) |
    static_cast<uint16_t>(response[response.size() - 1] << 8);
  response.resize(response.size() - 2);
  const uint16_t computed_crc = crc16(response);
  if (received_crc != computed_crc) {
    throw ModbusException(
            "CRC mismatch, response=" + toHex(response) +
            " received=0x" + std::to_string(received_crc) +
            " computed=0x" + std::to_string(computed_crc));
  }

  const uint8_t request_slave = request_without_crc[0];
  const uint8_t request_function = request_without_crc[1];
  if (response[0] != request_slave) {
    throw ModbusException("Unexpected slave id in response: " + toHex(response));
  }

  if (response[1] == static_cast<uint8_t>(request_function | 0x80)) {
    const uint8_t code = response.size() > 2 ? response[2] : 0;
    throw ModbusException("Modbus exception code: " + std::to_string(code));
  }

  if (response[1] != request_function) {
    throw ModbusException("Unexpected function code in response: " + toHex(response));
  }

  return response;
}

std::vector<uint16_t> ModbusRtuMaster::readInputRegisters(
  uint8_t slave_id,
  uint16_t start_address,
  uint16_t register_count)
{
  if (slave_id == 0) {
    throw ModbusException("readInputRegisters requires slave_id 1..247");
  }
  if (register_count == 0 || register_count > 125) {
    throw ModbusException("readInputRegisters register_count must be 1..125");
  }

  std::vector<uint8_t> request{slave_id, 0x04};
  appendU16(request, start_address);
  appendU16(request, register_count);

  const size_t expected_size = 5 + static_cast<size_t>(2 * register_count);
  const auto response = transact(request, expected_size);

  const uint8_t byte_count = response[2];
  if (byte_count != register_count * 2) {
    throw ModbusException("Unexpected byte count in read input response: " + toHex(response));
  }

  std::vector<uint16_t> values;
  values.reserve(register_count);
  for (uint16_t i = 0; i < register_count; ++i) {
    values.push_back(readU16(response, 3 + static_cast<size_t>(i * 2)));
  }
  return values;
}

std::vector<uint16_t> ModbusRtuMaster::readHoldingRegisters(
  uint8_t slave_id,
  uint16_t start_address,
  uint16_t register_count)
{
  if (slave_id == 0) {
    throw ModbusException("readHoldingRegisters requires slave_id 1..247");
  }
  if (register_count == 0 || register_count > 125) {
    throw ModbusException("readHoldingRegisters register_count must be 1..125");
  }

  std::vector<uint8_t> request{slave_id, 0x03};
  appendU16(request, start_address);
  appendU16(request, register_count);

  const size_t expected_size = 5 + static_cast<size_t>(2 * register_count);
  const auto response = transact(request, expected_size);

  const uint8_t byte_count = response[2];
  if (byte_count != register_count * 2) {
    throw ModbusException("Unexpected byte count in read holding response: " + toHex(response));
  }

  std::vector<uint16_t> values;
  values.reserve(register_count);
  for (uint16_t i = 0; i < register_count; ++i) {
    values.push_back(readU16(response, 3 + static_cast<size_t>(i * 2)));
  }
  return values;
}

void ModbusRtuMaster::writeSingleRegister(
  uint8_t slave_id,
  uint16_t address,
  uint16_t value)
{
  if (slave_id == 0) {
    throw ModbusException("writeSingleRegister requires slave_id 1..247");
  }

  std::vector<uint8_t> request{slave_id, 0x06};
  appendU16(request, address);
  appendU16(request, value);

  const auto response = transact(request, 8);
  if (readU16(response, 2) != address || readU16(response, 4) != value) {
    throw ModbusException("Unexpected write response: " + toHex(response));
  }
}

void ModbusRtuMaster::writeMultipleRegisters(
  uint8_t slave_id,
  uint16_t start_address,
  const std::vector<uint16_t> & values)
{
  if (values.empty()) {
    throw ModbusException("writeMultipleRegisters requires at least one register");
  }
  if (values.size() > 123) {
    throw ModbusException("writeMultipleRegisters supports at most 123 registers");
  }

  std::vector<uint8_t> request{slave_id, 0x10};
  appendU16(request, start_address);
  appendU16(request, static_cast<uint16_t>(values.size()));
  request.push_back(static_cast<uint8_t>(values.size() * 2));
  for (const auto value : values) {
    appendU16(request, value);
  }

  const auto response = transact(request, 8);
  if (readU16(response, 2) != start_address ||
    readU16(response, 4) != values.size())
  {
    throw ModbusException("Unexpected write response: " + toHex(response));
  }
}

}  // namespace panthera_rs485
