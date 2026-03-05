#include <cmath>
#include <algorithm>
#include <sstream>
#include "dasrobot_drive_controller/dasrobot_drive_controller.hpp"

namespace dasrobot_drive_controller {

hardware_interface::CallbackReturn DasrobotDriveController::on_init(const hardware_interface::HardwareInfo & info) {
    RCLCPP_INFO(rclcpp::get_logger("DasrobotDriveController"), "DasrobotDriveController: Initializing...");

    hardware_info_ = info;

    // Проверяем наличие обязательных параметров
    const auto& params = hardware_info_.hardware_parameters;
    if (params.find("serial_port_name") == params.end()) {
        RCLCPP_FATAL(rclcpp::get_logger("DasrobotDriveController"), "Missing required parameter 'serial_port_name'");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("DasrobotDriveController"), "ALL PARAMS:");
    // for (const auto& param : params) {
    //     RCLCPP_INFO(rclcpp::get_logger("DasrobotDriveController"), "  %s = '%s'", 
    //                 param.first.c_str(), param.second.c_str());
    // }

    // Читаем параметры из URDF с обработкой ошибок
    try {
        encoder_ppr_ = std::stod(params.at("encoder_ppr"));
        baudrate_ = std::stoi(params.at("baudrate"));
        serial_port_name_ = params.at("serial_port_name");
        
        wheel_count_ = std::stoi(params.at("wheel_count"));
        wheel_radius_ = std::stod(params.at("wheel_radius"));
        wheel_separation_x_ = std::stod(params.at("wheel_separation_x"));
        wheel_separation_y_ = std::stod(params.at("wheel_separation_y"));
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("DasrobotDriveController"), 
                    "Failed to parse hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    counts_to_rad_ = 2.0 * M_PI / encoder_ppr_;

    RCLCPP_INFO(rclcpp::get_logger("DasrobotDriveController"), 
        "  Plugin Config:\n"
        "  Port: %s\n"
        "  Baudrate: %d\n"
        "  Wheel radius: %.3f m\n"
        "  Separation X (F-B): %.3f m\n"
        "  Separation Y (L-R): %.3f m\n"
        "  Encoder PPR: %.0f",
        serial_port_name_.c_str(), baudrate_, wheel_radius_, 
        wheel_separation_x_, wheel_separation_y_, encoder_ppr_);

    if (!open_serial()) {
        RCLCPP_FATAL(rclcpp::get_logger("DasrobotDriveController"), "Failed to open serial port: %s", serial_port_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DasrobotDriveController"), "Serial port opened successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DasrobotDriveController::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < wheel_count_; ++i) {
        if (i >= hardware_info_.joints.size()) {
            RCLCPP_FATAL(rclcpp::get_logger("DasrobotDriveController"), 
                        "Joint %zu not found in URDF", i);
            break;
        }
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            hardware_info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            hardware_info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DasrobotDriveController::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < wheel_count_; ++i) {
        if (i >= hardware_info_.joints.size()) break;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hardware_info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocities_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type DasrobotDriveController::read(const rclcpp::Time &, const rclcpp::Duration &period) {
    double delta_time = period.seconds();
    if (delta_time <= 0.0) return hardware_interface::return_type::OK;

    this->serial_write("e\n");
    std::string response = this->serial_readline(10);

    if (response.empty() || response.find("e,") != 0) {
        if (!response.empty()) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DasrobotDriveController"), *rclcpp::Clock::SharedPtr(), 1000,
                               "Invalid encoder response: '%s'", response.c_str());
        }
        return hardware_interface::return_type::OK;
    }

    std::stringstream ss(response.substr(2));
    std::string value;
    size_t counter = 0;

    while (std::getline(ss, value, ',') && counter < wheel_count_) {
        try {
            int int_value = std::stoi(value);
            hw_positions_[counter] = int_value * counts_to_rad_;
            hw_velocities_[counter] = (int_value - previous_encoders_value_[counter]) * 
                                    counts_to_rad_ / delta_time;
            previous_encoders_value_[counter] = int_value;
            counter++;
        } catch (const std::exception&) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DasrobotDriveController"), *rclcpp::Clock::SharedPtr(), 5000,
                               "Failed to parse encoder value: %s", value.c_str());
            break;
        }
    }

    return hardware_interface::return_type::OK;    
}

hardware_interface::return_type DasrobotDriveController::write(const rclcpp::Time &, const rclcpp::Duration &) {
    std::string cmd = "v,";
    for (size_t i = 0; i < wheel_count_; ++i) {
        double rad_s = cmd_velocities_[i];
        double rpm_double = (rad_s * 60.0) / (2.0 * M_PI);
        int rpm_int = static_cast<int>(std::round(rpm_double));  
        cmd += std::to_string(rpm_int);
        if (i < wheel_count_ - 1) cmd += ",";
    }
    cmd += "\n";

    this->serial_write(cmd);
    return hardware_interface::return_type::OK;
}

// Деструктор
DasrobotDriveController::~DasrobotDriveController() {
    close_serial();
}

bool DasrobotDriveController::open_serial() {
    serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DasrobotDriveController"), 
                    "Failed to open serial port %s: %s", 
                    serial_port_name_.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DasrobotDriveController"), "Failed to get serial attributes");
        close_serial();
        return false;
    }
    
    // ✅ Поддержка разных baudrate
    speed_t baud_speed;
    switch (baudrate_) {
        case 115200: baud_speed = B115200; break;
        case 57600:  baud_speed = B57600;  break;
        case 9600:   baud_speed = B9600;   break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("DasrobotDriveController"), 
                       "Unsupported baudrate %d, using 115200", baudrate_);
            baud_speed = B115200;
            break;
    }
    
    cfsetispeed(&tty, baud_speed);
    cfsetospeed(&tty, baud_speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8n1
    tty.c_iflag &= ~IGNBRK;                         // No BREAK handling
    tty.c_lflag = 0;                                // No local echo
    tty.c_oflag = 0;                                // No output processing
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DasrobotDriveController"), "Failed to set serial attributes");
        close_serial();
        return false;
    }
    
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}

void DasrobotDriveController::close_serial() { 
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool DasrobotDriveController::serial_write(const std::string& data) {
    if (serial_fd_ < 0) return false;
    
    ssize_t bytes = ::write(serial_fd_, data.c_str(), data.length());
    if (bytes < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DasrobotDriveController"), "Serial write failed: %s", strerror(errno));
        return false;
    }
    
    tcdrain(serial_fd_);
    return bytes > 0;
}

std::string DasrobotDriveController::serial_readline(int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(serial_fd_, &fds);
    
    struct timeval tv = {0, timeout_ms * 1000};
    
    if (select(serial_fd_ + 1, &fds, NULL, NULL, &tv) > 0) {
        char buf[256];
        ssize_t n = ::read(serial_fd_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            // Убираем \r\n
            std::string result(buf);
            result.erase(std::remove(result.begin(), result.end(), '\r'), result.end());
            result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
            return result;
        }
    }
    return "";
}

} // namespace dasrobot_drive_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dasrobot_drive_controller::DasrobotDriveController, hardware_interface::SystemInterface)
