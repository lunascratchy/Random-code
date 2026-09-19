#ifndef AURACLE_HARDWARE_ARDUINO_COMMS_HPP
#define AURACLE_HARDWARE_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace auracle_hardware
{
class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string & serial_device, int32_t baud_rate)
  {
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convertBaudRate(baud_rate));
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

  void sendVelocity(double left_rad_s, double right_rad_s)
  {
    std::ostringstream ss;
    ss << "v " << left_rad_s << " " << right_rad_s << "\n";
    writeLine(ss.str());
  }
  void sendCommand(const std::string & line)
  {
    writeLine(line);
  }
  bool readTelemetry(long & l_enc, long & r_enc, float * acc, float * gyro)
  {
    bool got_new_data = false;
    while (serial_conn_.IsDataAvailable()) {
      uint8_t byte;
      try {
        serial_conn_.ReadByte(byte, 1);
      } catch (const std::exception &) {
        break;
      }
      char c = static_cast<char>(byte);

      if (c == '\r') continue;

      if (c == '\n') {
        if (parseLine(line_buf_, l_enc, r_enc, acc, gyro)) {
          got_new_data = true; // Mark that we got data, but DO NOT RETURN YET
        }
        line_buf_.clear();
        continue;
      }

      if (line_buf_.size() < 96) {
        line_buf_.push_back(c);
      } else {
        line_buf_.clear();
      }
    }
    return got_new_data; // Returns true only after draining the entire buffer
  }
  bool waitForReady(int timeout_ms)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
             .count() < timeout_ms)
    {
      while (serial_conn_.IsDataAvailable()) {
        uint8_t byte;
        try {
          serial_conn_.ReadByte(byte, 1);
        } catch (const std::exception &) {
          break;
        }
        char c = static_cast<char>(byte);
        if (c == '\n') {
          bool is_ready = line_buf_.rfind("READY", 0) == 0;
          line_buf_.clear();
          if (is_ready) {
            return true;
          }
        } else if (c != '\r') {
          line_buf_.push_back(c);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

private:
  void writeLine(const std::string & line)
  {
    try {
      serial_conn_.Write(line);
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("ArduinoComms"), "Serial write failed: %s", e.what());
    }
  }

  static bool parseLine(
    const std::string & line, long & l_enc, long & r_enc, float * acc, float * gyro)
  {
    if (line.empty() || line[0] != 'e') {
      return false;
    }
    int matched = std::sscanf(
      line.c_str(), "e %ld %ld i %f %f %f %f %f %f",
      &l_enc, &r_enc, &acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    return matched == 8;
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
  std::string line_buf_;
};

}  

#endif 
