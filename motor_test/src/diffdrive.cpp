#include <limits>
#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include <motor_test/diffdrive.hpp>


namespace motor_test
{
    hardware_interface::CallbackReturn DiffDriveHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if(hardware_interface::SystemInterface::on_init(info)!=
            hardware_interface::CallbackReturn::SUCCESS)
            {
                return hardware_interface::CallbackReturn::ERROR;
            }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
        cfg_.timeout = std::stoi(info_.hardware_parameters["timeout_ms"]);
        
        if(info_.hardware_parameters.count("k_p") > 0)
        {
            cfg_.k_p = std::stoi(info_.hardware_parameters["k_p"]);
            cfg_.k_i = std::stoi(info_.hardware_parameters["k_i"]);
            cfg_.k_o = std::stoi(info_.hardware_parameters["k_o"]); 
            cfg_.k_d = std::stoi(info_.hardware_parameters["k_d"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "No PID values specified, defaulting to 0");
        }

        left_wheel_.setup(cfg_.left_wheel_name, cfg_.counts_per_rev);
        right_wheel_.setup(cfg_.right_wheel_name, cfg_.counts_per_rev);
        
        for(hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if(joint.command_interfaces.size() != 1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Joint '%s' has '%s' as command interface. '%s' expected.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if(joint.state_interfaces.size() != 2)
            {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if(joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Joint '%s' has '%s' as state interface. '%s' expected.", joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
            if(joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Joint '%s' has '%s' as state interface. '%s' expected.", joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    }
    
   std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
   {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));
        
        return state_interfaces;
   }

   std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
   {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));
        
        return command_interfaces;
   }

   hardware_interface::CallbackReturn DiffDriveHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
   {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Configuring ...please wait");
        if(comms_.connected())
        {
            comms_.disconnect();
        }

        comms_.connect(cfg_.device, cfg_.baudrate, cfg_.timeout);
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Connected to Arduino");
        
        return hardware_interface::CallbackReturn::SUCCESS;
   }

   hardware_interface::CallbackReturn DiffDriveHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
   {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Activating ...please wait");
        if(!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Cannot activate, no connection to Arduino");
            return hardware_interface::CallbackReturn::ERROR;
        }
        

        return hardware_interface::CallbackReturn::SUCCESS;
   }

   hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
   {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Deactivating ...please wait");
        if(comms_.connected())
        {
            comms_.disconnect();
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Disconnected from Arduino");
        }
        return hardware_interface::CallbackReturn::SUCCESS;
   }

   hardware_interface::CallbackReturn DiffDriveHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
   {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Cleaning up ...please wait");
        if(comms_.connected())
        {
            comms_.disconnect();
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Disconnected from Arduino");
        }
        return hardware_interface::CallbackReturn::SUCCESS;
   }
   hardware_interface::return_type DiffDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
   {
        if(!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Cannot read, no connection to Arduino");
            return hardware_interface::return_type::ERROR;
        }

        comms_.read_encoders(left_wheel_.enc, right_wheel_.enc);

        double dt = period.seconds();

        double prev_pos = left_wheel_.pos;
        left_wheel_.pos = left_wheel_.calc_enc_angle();
        left_wheel_.vel = (left_wheel_.pos - prev_pos) / dt;

        prev_pos = right_wheel_.pos;
        right_wheel_.pos = right_wheel_.calc_enc_angle();
        right_wheel_.vel = (right_wheel_.pos - prev_pos) / dt;

        return hardware_interface::return_type::OK;
   }

   hardware_interface::return_type DiffDriveHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
   {
        if(!comms_.connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "Cannot write, no connection to Arduino");
            return hardware_interface::return_type::ERROR;
        }

        int motor_l_counts_per_loop = left_wheel_.cmd / left_wheel_.rads_per_sec  / cfg_.loop_rate;
        int motor_r_counts_per_loop = right_wheel_.cmd / right_wheel_.rads_per_sec  / cfg_.loop_rate;
        comms_.set_motor_value(motor_l_counts_per_loop, motor_r_counts_per_loop);

        return hardware_interface::return_type::OK;
   }
        
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motor_test::DiffDriveHardware, hardware_interface::SystemInterface)  