#ifndef PANTHERA_RS485__SERIAL_PORT_HPP_
#define PANTHERA_RS485__SERIAL_PORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace panthera_rs485
{

enum class SerialParity
{
  NONE,
  EVEN,
  ODD,
};

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void open(
    const std::string & device,
    int baudrate,
    std::chrono::milliseconds read_timeout,
    SerialParity parity = SerialParity::NONE,
    int stop_bits = 1);

  void close();
  bool isOpen() const;

  void flushInput();
  void writeAll(const std::vector<uint8_t> & data);
  std::vector<uint8_t> readExact(
    size_t length,
    std::chrono::milliseconds timeout);

private:
  int fd_{-1};
  std::chrono::milliseconds read_timeout_{100};
};

}  // namespace panthera_rs485

#endif  // PANTHERA_RS485__SERIAL_PORT_HPP_
