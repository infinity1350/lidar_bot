#ifndef DIFFDRIVE_HPP
#define DIFFDRIVE_HPP

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <motor_test/arduino_comms.hpp>
#include <motor_test/wheel.hpp>

namespace motor_test
{

class DiffDriveHardware : public hardware_interface::SystemInterface
{
    struct config
    {
        std::string left_wheel_name = "";
        std::string right_wheel_name = "";
        int counts_per_rev = 0;
        float loop_rate = 0.0;
        std::string device = "";
        int baudrate = 0;
        int timeout = 0;
        int k_p = 0;
        int k_d = 0;
        int k_o = 0;
        int k_i = 0;
    };
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        std::vector <hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector <hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        ArduinoComms comms_;
        config cfg_;
        Wheel left_wheel_;
        Wheel right_wheel_;
};
}
#endif // DIFFDRIVE_HPP