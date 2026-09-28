#include "auracle_hardware/robot_system.hpp"

#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace auracle_hardware
{

namespace
{
constexpr size_t FL = 0;
constexpr size_t FR = 1;
constexpr size_t RL = 2;
constexpr size_t RR = 3;

// MPU6050 at the ranges the firmware configures (+/-2g, +/-250 deg/s).
constexpr double ACCEL_SCALE = 9.80665 / 16384.0;          // raw -> m/s^2
constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0;      // raw -> rad/s

// Firmware sends telemetry every 50 ms; warn if nothing for this long.
constexpr auto STALE_TIMEOUT = std::chrono::milliseconds(500);

rclcpp::Logger logger()
{
  return rclcpp::get_logger("AuracleHardwareInterface");
}

std::string getParam(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & default_value)
{
  auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;
  device_ = getParam(p, "device", device_);
  baud_rate_ = std::stoi(getParam(p, "baud_rate", "115200"));
  handshake_timeout_ms_ = std::stoi(getParam(p, "handshake_timeout_ms", "5000"));
  left_enc_counts_per_rev_ = std::stod(getParam(p, "left_enc_counts_per_rev", "730.0"));
  right_enc_counts_per_rev_ = std::stod(getParam(p, "right_enc_counts_per_rev", "655.0"));
  imu_sensor_name_ = getParam(p, "imu_sensor_name", "imu_sensor");
  imu_enabled_ = getParam(p, "imu_enabled", "true") == "true";

  double left_sign = std::stod(getParam(p, "left_direction_sign", "1.0"));
  double right_sign = std::stod(getParam(p, "right_direction_sign", "1.0"));

  wheels_[FL].setup(getParam(p, "front_left_wheel_name", "front_left_wheel_joint"),
    left_enc_counts_per_rev_, left_sign);
  wheels_[FR].setup(getParam(p, "front_right_wheel_name", "front_right_wheel_joint"),
    right_enc_counts_per_rev_, right_sign);
  wheels_[RL].setup(getParam(p, "rear_left_wheel_name", "rear_left_wheel_joint"),
    left_enc_counts_per_rev_, left_sign);
  wheels_[RR].setup(getParam(p, "rear_right_wheel_name", "rear_right_wheel_joint"),
    right_enc_counts_per_rev_, right_sign);

  for (const auto & joint : info_.joints) {
    bool matched = false;
    for (const auto & w : wheels_) {
      if (joint.name == w.name) {
        matched = true;
        break;
      }
    }
    if (!matched) {
      RCLCPP_WARN(
        logger(),
        "Joint '%s' declared in the xacro is not one of the 4 configured wheel names - "
        "it will not be driven.",
        joint.name.c_str());
    }
  }

  RCLCPP_INFO(
    logger(),
    "Configured for device=%s baud=%d left_enc=%.1f right_enc=%.1f imu_enabled=%s "
    "wheels=[%s, %s, %s, %s]",
    device_.c_str(), baud_rate_, left_enc_counts_per_rev_, right_enc_counts_per_rev_,
    imu_enabled_ ? "true" : "false",
    wheels_[FL].name.c_str(), wheels_[FR].name.c_str(),
    wheels_[RL].name.c_str(), wheels_[RR].name.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Opening serial port %s ...", device_.c_str());
  try {
    comms_.connect(device_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      logger(), "Failed to open serial port %s: %s", device_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Opening the port toggles DTR, which resets the Nano: bootloader +
  // IMU gyro calibration (~0.5 s, robot must be still) before READY.
  bool imu_ok = false;
  if (comms_.waitForReady(handshake_timeout_ms_, imu_ok)) {
    RCLCPP_INFO(logger(), "Arduino READY (imu=%s).", imu_ok ? "ok" : "absent");
    if (imu_enabled_ && !imu_ok) {
      RCLCPP_ERROR(
        logger(),
        "imu_enabled is true but the Nano found no MPU6050. The EKF will get a "
        "zero yaw rate and odom won't rotate - check SDA/SCL (A4/A5) or relaunch "
        "with use_imu:=false.");
    }
  } else {
    RCLCPP_WARN(
      logger(),
      "No READY from the Arduino within %d ms - continuing anyway, but check "
      "the port, baud rate and firmware if reads keep failing.",
      handshake_timeout_ms_);
  }

  have_prev_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (comms_.connected()) {
    comms_.sendStop();
    comms_.disconnect();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & w : wheels_) {
    w.command = 0.0;
  }
  comms_.sendVelocity(0.0, 0.0);
  last_rx_ = std::chrono::steady_clock::now();
  stale_warned_ = false;
  RCLCPP_INFO(logger(), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.sendStop();
  RCLCPP_INFO(logger(), "Deactivated - motors stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AuracleHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & w : wheels_) {
    state_interfaces.emplace_back(w.name, hardware_interface::HW_IF_POSITION, &w.position);
    state_interfaces.emplace_back(w.name, hardware_interface::HW_IF_VELOCITY, &w.velocity);
  }

  if (imu_enabled_) {
    static const char * imu_interfaces[10] = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
    imu_states_[3] = 1.0;  // identity orientation - the MPU6050 gives none
    for (size_t i = 0; i < imu_states_.size(); ++i) {
      state_interfaces.emplace_back(imu_sensor_name_, imu_interfaces[i], &imu_states_[i]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AuracleHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & w : wheels_) {
    command_interfaces.emplace_back(w.name, hardware_interface::HW_IF_VELOCITY, &w.command);
  }
  return command_interfaces;
}

hardware_interface::return_type AuracleHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  Telemetry tel;
  bool saw_ready = false;
  bool got = comms_.readLatest(tel, saw_ready);
  auto now = std::chrono::steady_clock::now();

  if (saw_ready) {
    // Nano rebooted (brownout or watchdog) - its counters restarted at 0.
    RCLCPP_WARN(logger(), "Arduino sent READY mid-run - it reset. Re-syncing encoders.");
    have_prev_ = false;
  }

  if (!got) {
    if (!stale_warned_ && now - last_rx_ > STALE_TIMEOUT) {
      RCLCPP_WARN(logger(), "No telemetry from the Arduino for >500 ms.");
      stale_warned_ = true;
    }
    return hardware_interface::return_type::OK;
  }
  last_rx_ = now;
  stale_warned_ = false;

  // Positions are integrated from tick deltas rather than taken absolute,
  // so a Nano reset doesn't make odometry jump back to zero.
  if (have_prev_ && tel.t_ms > prev_.t_ms) {
    double dt = (tel.t_ms - prev_.t_ms) / 1000.0;
    double dl = wheels_[RL].ticksToRadians(static_cast<double>(tel.l_enc - prev_.l_enc));
    double dr = wheels_[RR].ticksToRadians(static_cast<double>(tel.r_enc - prev_.r_enc));
    wheels_[RL].position += dl;
    wheels_[RR].position += dr;
    wheels_[RL].velocity = dl / dt;
    wheels_[RR].velocity = dr / dt;
  } else {
    wheels_[RL].velocity = 0.0;
    wheels_[RR].velocity = 0.0;
  }
  prev_ = tel;
  have_prev_ = true;

  // Front wheels have no encoders: report the same-side rear wheel.
  wheels_[FL].position = wheels_[RL].position;
  wheels_[FL].velocity = wheels_[RL].velocity;
  wheels_[FR].position = wheels_[RR].position;
  wheels_[FR].velocity = wheels_[RR].velocity;

  if (imu_enabled_ && tel.has_imu) {
    imu_states_[4] = tel.imu[3] * GYRO_SCALE;
    imu_states_[5] = tel.imu[4] * GYRO_SCALE;
    imu_states_[6] = tel.imu[5] * GYRO_SCALE;
    imu_states_[7] = tel.imu[0] * ACCEL_SCALE;
    imu_states_[8] = tel.imu[1] * ACCEL_SCALE;
    imu_states_[9] = tel.imu[2] * ACCEL_SCALE;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuracleHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // diff_drive_controller commands both wheels on a side identically; the
  // Nano only has one closed loop per side (rear encoder), front follows.
  double left_rad_s = (wheels_[FL].command + wheels_[RL].command) / 2.0;
  double right_rad_s = (wheels_[FR].command + wheels_[RR].command) / 2.0;

  // Sent every cycle even when unchanged - it doubles as the heartbeat
  // for the Nano's 500 ms comms watchdog.
  comms_.sendVelocity(
    wheels_[RL].radiansToTicks(left_rad_s),
    wheels_[RR].radiansToTicks(right_rad_s));
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(auracle_hardware::AuracleHardwareInterface, hardware_interface::SystemInterface)
