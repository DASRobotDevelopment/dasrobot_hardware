#pragma once

#include <array>
#include <string>
#include <vector>
#include <fcntl.h>      
#include <unistd.h>     
#include <termios.h>    
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dasrobot_drive_controller {
    class DasrobotDriveController : public hardware_interface::SystemInterface {
    public:

        DasrobotDriveController() = default;
        virtual ~DasrobotDriveController();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:

        int serial_fd_ = -1;
        int baudrate_ = 115200;
        std::string serial_port_name_ = "/dev/ttyUSB0";
        
        double encoder_ppr_ = 988.0;
        size_t wheel_count_ = 1;
        double wheel_radius_ = 0.1;
        double wheel_separation_x_ = 0.5;
        double wheel_separation_y_ = 0.4;

        static constexpr int MAX_WHEEL_COUNT_ = 6;

        double counts_to_rad_ = 2.0 * M_PI / encoder_ppr_;

        hardware_interface::HardwareInfo hardware_info_;
        std::array<long, MAX_WHEEL_COUNT_> previous_encoders_value_{};
        std::array<double, MAX_WHEEL_COUNT_> hw_positions_{};
        std::array<double, MAX_WHEEL_COUNT_> hw_velocities_{};
        std::array<double, MAX_WHEEL_COUNT_> cmd_velocities_{};

        bool open_serial();
        void close_serial();
        bool serial_write(const std::string& data);
        std::string serial_readline(int timeout_ms = 10);  
              
    };        
} // namespace dasrobot_drive_controller
