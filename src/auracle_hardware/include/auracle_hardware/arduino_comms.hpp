#ifndef AURACLE_HARDWARE_ARDUINO_COMMS_HPP
#define AURACLE_HARDWARE_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace auracle_hardware
{

// One "e ..." line from the Nano. See serial_protocol.h in the firmware.
struct Telemetry
{
  uint32_t t_ms = 0;       // Nano millis() when the sample was taken
  long l_enc = 0;          // rear-left ticks (1x decoding)
  long r_enc = 0;          // rear-right ticks
  bool has_imu = false;
  int16_t imu[6] = {0, 0, 0, 0, 0, 0};  // ax ay az gx gy gz, raw MPU6050 counts
};

class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string & serial_device, int32_t baud_rate)
  {
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convertBaudRate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_conn_.FlushIOBuffers();
  }

  void disconnect()
  {
    if (serial_conn_.IsOpen()) {
      serial_conn_.Close();
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  // Rear-wheel targets in encoder ticks/sec.
  void sendVelocity(double left_ticks_s, double right_ticks_s)
  {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "v %.1f %.1f\n", left_ticks_s, right_ticks_s);
    writeLine(buf);
  }

  void sendStop()
  {
    writeLine("s\n");
  }

  // Drains everything waiting on the port in one read() and keeps the
  // newest complete telemetry line. Returns true if at least one was parsed.
  // Sets saw_ready if the Nano printed READY (i.e. it rebooted).
  bool readLatest(Telemetry & out, bool & saw_ready)
  {
    saw_ready = false;
    if (!fillBuffer()) {
      return false;
    }

    bool got = false;
    size_t start = 0;
    size_t nl;
    while ((nl = rx_.find('\n', start)) != std::string::npos) {
      size_t len = nl - start;
      if (len > 0 && rx_[start + len - 1] == '\r') {
        --len;
      }
      const char * line = rx_.c_str() + start;
      if (len > 0 && line[0] == 'e') {
        got |= parseTelemetry(std::string(line, len), out);
      } else if (len >= 5 && std::string(line, 5) == "READY") {
        saw_ready = true;
      }
      start = nl + 1;
    }
    rx_.erase(0, start);
    if (rx_.size() > 256) {
      rx_.clear();  // no newline in 256 bytes = garbage
    }
    return got;
  }

  bool waitForReady(int timeout_ms, bool & imu_ok)
  {
    imu_ok = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (fillBuffer()) {
        size_t pos = rx_.find("READY");
        if (pos != std::string::npos && rx_.find('\n', pos) != std::string::npos) {
          imu_ok = rx_.find("imu=1", pos) != std::string::npos;
          rx_.erase(0, rx_.find('\n', pos) + 1);
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

private:
  bool fillBuffer()
  {
    int n = 0;
    try {
      n = serial_conn_.GetNumberOfBytesAvailable();
      if (n <= 0) {
        return false;
      }
      std::string chunk;
      serial_conn_.Read(chunk, static_cast<size_t>(n), 5);
      rx_ += chunk;
    } catch (const std::exception &) {
      return false;
    }
    return true;
  }

  void writeLine(const std::string & line)
  {
    try {
      serial_conn_.Write(line);
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("ArduinoComms"), "Serial write failed: %s", e.what());
    }
  }

  static bool parseTelemetry(const std::string & line, Telemetry & out)
  {
    Telemetry t;
    unsigned long t_ms = 0;
    int matched = std::sscanf(
      line.c_str(), "e %lu %ld %ld i %hd %hd %hd %hd %hd %hd",
      &t_ms, &t.l_enc, &t.r_enc,
      &t.imu[0], &t.imu[1], &t.imu[2], &t.imu[3], &t.imu[4], &t.imu[5]);
    if (matched != 3 && matched != 9) {
      return false;
    }
    t.t_ms = static_cast<uint32_t>(t_ms);
    t.has_imu = (matched == 9);
    out = t;
    return true;
  }

  static LibSerial::BaudRate convertBaudRate(int32_t baud_rate)
  {
    switch (baud_rate) {
      case 9600: return LibSerial::BaudRate::BAUD_9600;
      case 19200: return LibSerial::BaudRate::BAUD_19200;
      case 38400: return LibSerial::BaudRate::BAUD_38400;
      case 57600: return LibSerial::BaudRate::BAUD_57600;
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("ArduinoComms"),
          "Unsupported baud rate %d, defaulting to 115200", baud_rate);
        return LibSerial::BaudRate::BAUD_115200;
    }
  }

  LibSerial::SerialPort serial_conn_;
  std::string rx_;
};

}  // namespace auracle_hardware

#endif
