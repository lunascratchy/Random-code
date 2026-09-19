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

  device_ = getParam(info_.hardware_parameters, "device", device_);
  baud_rate_ = std::stoi(getParam(info_.hardware_parameters, "baud_rate", "115200"));
  left_enc_counts_per_rev_ = std::stod(getParam(info_.hardware_parameters, "left_enc_counts_per_rev", "730.0"));
  right_enc_counts_per_rev_ = std::stod(getParam(info_.hardware_parameters, "right_enc_counts_per_rev", "655.0"));
  imu_sensor_name_ = getParam(info_.hardware_parameters, "imu_sensor_name", "imu_sensor");
  imu_enabled_ = getParam(info_.hardware_parameters, "imu_enabled", "true") == "true";

  double left_sign = std::stod(getParam(info_.hardware_parameters, "left_direction_sign", "1.0"));
  double right_sign = std::stod(getParam(info_.hardware_parameters, "right_direction_sign", "1.0"));

  std::string fl_name = getParam(info_.hardware_parameters, "front_left_wheel_name", "front_left_wheel_joint");
  std::string fr_name = getParam(info_.hardware_parameters, "front_right_wheel_name", "front_right_wheel_joint");
  std::string rl_name = getParam(info_.hardware_parameters, "rear_left_wheel_name", "rear_left_wheel_joint");
  std::string rr_name = getParam(info_.hardware_parameters, "rear_right_wheel_name", "rear_right_wheel_joint");

  wheels_[FL].setup(fl_name, left_enc_counts_per_rev_, left_sign);
  wheels_[FR].setup(fr_name, right_enc_counts_per_rev_, right_sign);
  wheels_[RL].setup(rl_name, left_enc_counts_per_rev_, left_sign);
  wheels_[RR].setup(rr_name, right_enc_counts_per_rev_, right_sign);

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
  RCLCPP_INFO(rclcpp::get_logger("AuracleHardwareInterface"), "Opening serial port %s ...", device_.c_str());
  try {
    comms_.connect(device_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("AuracleHardwareInterface"), "Failed to open serial port %s: %s",
      device_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  // 1. Accumulate time regardless of whether we get data this loop
  dt_accumulator_ += period.seconds();

  if (!comms_.readTelemetry(l_enc, r_enc, acc, gyro)) {
    return hardware_interface::return_type::OK;
  }
  comms_up_ = true;

  double new_position_l = wheels_[RL].ticksToRadians(l_enc);
  double new_position_r = wheels_[RR].ticksToRadians(r_enc);

  // 2. Use the accumulated time to calculate true average velocity
  if (dt_accumulator_ > 0.0) {
    wheels_[RL].velocity = (new_position_l - wheels_[RL].position) / dt_accumulator_;
    wheels_[RR].velocity = (new_position_r - wheels_[RR].position) / dt_accumulator_;
  }
  
  // 3. Reset the timer for the next batch of data
  dt_accumulator_ = 0.0; 

  wheels_[RL].position = new_position_l;
  wheels_[RR].position = new_position_r;
  wheels_[RL].enc_ticks = l_enc;
  wheels_[RR].enc_ticks = r_enc;

  wheels_[FL].position = wheels_[RL].position;
  wheels_[FL].velocity = wheels_[RL].velocity;
  wheels_[FR].position = wheels_[RR].position;
  wheels_[FR].velocity = wheels_[RR].velocity;

  if (imu_enabled_) {
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
  double left_cmd = (wheels_[FL].command + wheels_[RL].command) / 2.0;
  double right_cmd = (wheels_[FR].command + wheels_[RR].command) / 2.0;

  comms_.sendVelocity(left_cmd, right_cmd);
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(auracle_hardware::AuracleHardwareInterface, hardware_interface::SystemInterface)
