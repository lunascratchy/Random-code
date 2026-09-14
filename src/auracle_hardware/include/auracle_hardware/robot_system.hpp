#ifndef AURACLE_HARDWARE_ROBOT_SYSTEM_HPP
#define AURACLE_HARDWARE_ROBOT_SYSTEM_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "auracle_hardware/arduino_comms.hpp"
#include "auracle_hardware/wheel.hpp"

namespace auracle_hardware
{

// Drives 4 wheel joints (front-left, front-right, rear-left,
// rear-right) over a 2-channel (left/right) serial link.
//
// Physical reality this maps to: the front and rear motor on each side
// share one L298N output and always move together, and only the rear
// motors have encoders. So:
//   - write(): averages the two joint commands on a side (front+rear)
//     into one target before sending it - if a controller only ever
//     commands one of the pair, it still isn't silently dropped.
//   - read(): the single rear-side encoder reading is applied to BOTH
//     joints on that side (front mirrors rear). There is no independent
//     sensing of the front wheels - if they ever slip differently than
//     the rear ones, this interface has no way to know.
//
// This class does NOT run a background thread. It doesn't need one:
// the push-telemetry protocol means read() only ever peeks at bytes
// already sitting in the OS receive buffer (see ArduinoComms), so it's
// already non-blocking without one. A request/response protocol would
// need a thread to stay RT-safe - this one doesn't pay that complexity
// cost because it doesn't have that problem.
class AuracleHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AuracleHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ArduinoComms comms_;

  // Fixed order: 0=front-left 1=front-right 2=rear-left 3=rear-right.
  // Only indices 2 and 3 ever receive a real encoder reading; 0 and 1
  // are copies of 2 and 3 respectively after every read().
  std::array<Wheel, 4> wheels_;

  // IMU state, indexed to match the <sensor name="imu_sensor"> block in
  // your ros2_control xacro: [ori.x/y/z/w, ang_vel.x/y/z, lin_acc.x/y/z]
  std::array<double, 10> imu_states_{};
  std::string imu_sensor_name_ = "imu_sensor";
  bool imu_enabled_ = true;

  std::string device_ = "/dev/ttyUSB0";
  int32_t baud_rate_ = 115200;
  double enc_counts_per_rev_ = 730.0;

  // Set true once a full telemetry line has been parsed at least once,
  // so we don't report bogus zero velocity forever if the Arduino is
  // simply slow to send its first line.
  bool comms_up_ = false;
};

}  // namespace auracle_hardware

#endif  // AURACLE_HARDWARE_ROBOT_SYSTEM_HPP
