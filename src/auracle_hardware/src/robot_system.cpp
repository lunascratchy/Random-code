#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <vector>

namespace auracle_hardware {

class AuracleHardwareInterface : public hardware_interface::SystemInterface {
public:
    // This runs once when the controller manager starts
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Read parameters from ros2_control.xacro
        std::string device = info_.hardware_parameters["device"];
        int baud = std::stoi(info_.hardware_parameters["baud_rate"]);
        
        RCLCPP_INFO(rclcpp::get_logger("AuracleHW"), "Initializing Auracle HW on %s at %d", device.c_str(), baud);
        
        // TODO: Initialize your Serial connection here
        return CallbackReturn::SUCCESS;
    }

    // This reads data FROM Arduino -> ROS
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // 1. Read Serial string from Arduino (e.g. "e 100 200")
        // 2. Parse the string into numbers
        // 3. Store in the state interfaces
        // Example:
        // hw_states_[0] = left_wheel_pos;
        // hw_states_[1] = right_wheel_pos;
        
        return hardware_interface::return_type::OK;
    }

    // This sends data FROM ROS -> Arduino
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // 1. Get desired velocity from command interfaces
        double left_vel = hw_commands_[0];
        double right_vel = hw_commands_[1];
        
        // 2. Format into a string for Arduino (e.g. "v 1.5 -1.5\n")
        std::string cmd = "v " + std::to_string(left_vel) + " " + std::to_string(right_vel) + "\n";
        
        // 3. Send via Serial
        // serial_port.write(cmd);
        
        return hardware_interface::return_type::OK;
    }

    // Standard ROS 2 boilerplate for mapping joints to internal arrays
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        // Add position and velocity interfaces here
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        // Add velocity command interfaces here
        return command_interfaces;
    }

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
};

} // namespace auracle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(auracle_hardware::AuracleHardwareInterface, hardware_interface::SystemInterface)