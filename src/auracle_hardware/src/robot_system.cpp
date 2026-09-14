#include "auracle_hardware/robot_system.hpp"

#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace auracle_hardware
{

namespace
{
// Joint index constants, for readability at the call sites below.
constexpr size_t FL = 0;
constexpr size_t FR = 1;
constexpr size_t RL = 2;
constexpr size_t RR = 3;

// Missing/blank param doesn't throw and take down the controller_manager
// - falls back to the default and lets on_init's own logging note it.
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
}  // namespace

hardware_interface::CallbackReturn AuracleHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_ = getParam(info_.hardware_parameters, "device", device_);
  baud_rate_ = std::stoi(getParam(info_.hardware_parameters, "baud_rate", "115200"));
  enc_counts_per_rev_ = std::stod(getParam(info_.hardware_parameters, "enc_counts_per_rev", "730"));
  imu_sensor_name_ = getParam(info_.hardware_parameters, "imu_sensor_name", "imu_sensor");
  imu_enabled_ = getParam(info_.hardware_parameters, "imu_enabled", "true") == "true";

  double left_sign = std::stod(getParam(info_.hardware_parameters, "left_direction_sign", "1.0"));
  double right_sign = std::stod(getParam(info_.hardware_parameters, "right_direction_sign", "1.0"));

  std::string fl_name = getParam(info_.hardware_parameters, "front_left_wheel_name", "front_left_wheel_joint");
  std::string fr_name = getParam(info_.hardware_parameters, "front_right_wheel_name", "front_right_wheel_joint");
  std::string rl_name = getParam(info_.hardware_parameters, "rear_left_wheel_name", "rear_left_wheel_joint");
  std::string rr_name = getParam(info_.hardware_parameters, "rear_right_wheel_name", "rear_right_wheel_joint");

  wheels_[FL].setup(fl_name, enc_counts_per_rev_, left_sign);
  wheels_[FR].setup(fr_name, enc_counts_per_rev_, right_sign);
  wheels_[RL].setup(rl_name, enc_counts_per_rev_, left_sign);
  wheels_[RR].setup(rr_name, enc_counts_per_rev_, right_sign);

  // Sanity-check that the joints ros2_control gave us actually match the
  // wheel names we were just configured with, so a typo in the xacro
  // fails loudly at startup instead of silently controlling nothing.
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
        rclcpp::get_logger("AuracleHardwareInterface"),
        "Joint '%s' declared in the xacro is not one of the 4 configured wheel names - "
        "it will not be driven.",
        joint.name.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AuracleHardwareInterface"),
    "Configured for device=%s baud=%d enc_counts_per_rev=%.1f imu_enabled=%s "
    "wheels=[%s, %s, %s, %s]",
    device_.c_str(), baud_rate_, enc_counts_per_rev_, imu_enabled_ ? "true" : "false",
    wheels_[FL].name.c_str(), wheels_[FR].name.c_str(),
    wheels_[RL].name.c_str(), wheels_[RR].name.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AuracleHardwareInterface"), "Opening serial port %s ...", device_.c_str());
  try {
    comms_.connect(device_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("AuracleHardwareInterface"), "Failed to open serial port %s: %s",
      device_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Firmware sends "READY\n" once out of setup(). Give it a couple of
  // seconds (the Nano resets on port-open) - if it never shows up we
  // still proceed, since older firmware builds may not send it, but log
  // loudly so a wiring/firmware problem is visible.
  if (!comms_.waitForReady(3000)) {
    RCLCPP_WARN(
      rclcpp::get_logger("AuracleHardwareInterface"),
      "Did not see a READY handshake from the Arduino within 3s - continuing anyway, "
      "but check wiring/firmware if reads keep failing.");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (comms_.connected()) {
    comms_.disconnect();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & w : wheels_) {
    w.command = 0;
  }
  comms_.sendVelocity(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("AuracleHardwareInterface"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuracleHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Always leave the robot stopped when the controller deactivates
  // (controller_manager shutdown, ctrl-C) rather than coasting at its
  // last commanded velocity.
  comms_.sendVelocity(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("AuracleHardwareInterface"), "Deactivated - motors stopped.");
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  long l_enc = wheels_[RL].enc_ticks;
  long r_enc = wheels_[RR].enc_ticks;
  float acc[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};

  if (!comms_.readTelemetry(l_enc, r_enc, acc, gyro)) {
    // No fresh line this cycle - just keep the last known state rather
    // than erroring the whole hardware component out. Expected often,
    // since the Arduino's own send rate and this read() period aren't
    // in lockstep.
    return hardware_interface::return_type::OK;
  }
  comms_up_ = true;

  double dt = period.seconds();

  double new_position_l = wheels_[RL].ticksToRadians(l_enc);
  double new_position_r = wheels_[RR].ticksToRadians(r_enc);

  if (dt > 0.0) {
    wheels_[RL].velocity = (new_position_l - wheels_[RL].position) / dt;
    wheels_[RR].velocity = (new_position_r - wheels_[RR].position) / dt;
  }
  wheels_[RL].position = new_position_l;
  wheels_[RR].position = new_position_r;
  wheels_[RL].enc_ticks = l_enc;
  wheels_[RR].enc_ticks = r_enc;

  // Front wheels have no encoders - mirror whatever the rear wheel on
  // the same side just reported. This is an honest approximation, not a
  // real measurement: if a front wheel is slipping independently, this
  // interface cannot see it.
  wheels_[FL].position = wheels_[RL].position;
  wheels_[FL].velocity = wheels_[RL].velocity;
  wheels_[FR].position = wheels_[RR].position;
  wheels_[FR].velocity = wheels_[RR].velocity;

  if (imu_enabled_) {
    // MPU6050 has no magnetometer -> no absolute orientation. Report an
    // identity quaternion (pair this with a static_covariance_orientation
    // of [-1, ...] in your controller config, which tells
    // imu_sensor_broadcaster/EKF this field isn't to be trusted) and
    // pass gyro/accel straight through.
    imu_states_[0] = 0.0;
    imu_states_[1] = 0.0;
    imu_states_[2] = 0.0;
    imu_states_[3] = 1.0;
    imu_states_[4] = gyro[0];
    imu_states_[5] = gyro[1];
    imu_states_[6] = gyro[2];
    imu_states_[7] = acc[0];
    imu_states_[8] = acc[1];
    imu_states_[9] = acc[2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuracleHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Front and rear motors on a side share one physical driver output -
  // average the two joint commands so neither one gets silently ignored
  // if a controller only ever writes to one of the pair.
  double left_cmd = (wheels_[FL].command + wheels_[RL].command) / 2.0;
  double right_cmd = (wheels_[FR].command + wheels_[RR].command) / 2.0;

  comms_.sendVelocity(left_cmd, right_cmd);
  return hardware_interface::return_type::OK;
}

}  // namespace auracle_hardware

PLUGINLIB_EXPORT_CLASS(auracle_hardware::AuracleHardwareInterface, hardware_interface::SystemInterface)
