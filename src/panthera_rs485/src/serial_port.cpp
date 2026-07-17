#include "panthera_rs485/serial_port.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace panthera_rs485
{

namespace
{

speed_t baudToConstant(int baudrate)
{
  switch (baudrate) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
    default:
      throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

std::string errnoMessage(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(
  const std::string & device,
  int baudrate,
  std::chrono::milliseconds read_timeout)
{
  close();
  read_timeout_ = read_timeout;

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error(errnoMessage("Failed to open serial port " + device));
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const auto msg = errnoMessage("tcgetattr failed for " + device);
    close();
    throw std::runtime_error(msg);
  }

  cfmakeraw(&tty);

  const speed_t speed = baudToConstant(baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const auto msg = errnoMessage("tcsetattr failed for " + device);
    close();
    throw std::runtime_error(msg);
  }

  tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}

void SerialPort::flushInput()
{
  if (!isOpen()) {
    throw std::runtime_error("Serial port is not open");
  }
  tcflush(fd_, TCIFLUSH);
}

void SerialPort::writeAll(const std::vector<uint8_t> & data)
{
  if (!isOpen()) {
    throw std::runtime_error("Serial port is not open");
  }

  size_t written = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(1000);
        continue;
      }
      throw std::runtime_error(errnoMessage("Serial write failed"));
    }
    written += static_cast<size_t>(n);
  }

  tcdrain(fd_);
}

std::vector<uint8_t> SerialPort::readExact(
  size_t length,
  std::chrono::milliseconds timeout)
{
  if (!isOpen()) {
    throw std::runtime_error("Serial port is not open");
  }

  std::vector<uint8_t> data;
  data.reserve(length);

  const auto start = std::chrono::steady_clock::now();
  while (data.size() < length) {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= timeout) {
      throw std::runtime_error(
        "Serial read timeout, expected " + std::to_string(length) +
        " bytes, got " + std::to_string(data.size()));
    }

    const auto remain = timeout - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    timeval tv{};
    tv.tv_sec = static_cast<long>(remain.count() / 1000);
    tv.tv_usec = static_cast<long>((remain.count() % 1000) * 1000);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    const int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(errnoMessage("Serial select failed"));
    }
    if (ret == 0) {
      continue;
    }

    uint8_t buffer[256];
    const size_t want = std::min(sizeof(buffer), length - data.size());
    const ssize_t n = ::read(fd_, buffer, want);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      throw std::runtime_error(errnoMessage("Serial read failed"));
    }
    if (n > 0) {
      data.insert(data.end(), buffer, buffer + n);
    }
  }

  return data;
}

}  // namespace panthera_rs485
