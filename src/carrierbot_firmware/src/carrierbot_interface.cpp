#include "carrierbot_firmware/carrierbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cstring>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace carrierbot_firmware
{
    CarrierbotInterface::CarrierbotInterface()
    {
    }

    CarrierbotInterface::~CarrierbotInterface()
    {
        if (can_interface_ != nullptr)
        {
            try
            {
                can_interface_->close();
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                    "Failed to close CAN port: " << e.what());
            }
            can_interface_.reset();
        }
    }

    //
    // 🔥 FOXy: configure thay cho on_init
    //
    hardware_interface::return_type CarrierbotInterface::configure(
        const hardware_interface::HardwareInfo &info)
    {
        info_ = info;

        try
        {
            port_ = info_.hardware_parameters.at("port");
            baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CarrierbotInterface"), "No CAN port specified!");
            return hardware_interface::return_type::ERROR;
        }

        can_interface_ = std::make_unique<WaveshareCAN>(port_, baudrate_, 2.0);
        ros_node_ = std::make_shared<rclcpp::Node>("carrierbot_interface");
        telemetry_pub_ = ros_node_->create_publisher<carrierbot_msgs::msg::CarrierbotTelemetry>("/carrierbot/telemetry", 10);
        velocity_command_.resize(info_.joints.size(), 0.0); 
        position_state_.resize(info_.joints.size(), 0.0);
        velocity_state_.resize(info_.joints.size(), 0.0);

        return hardware_interface::return_type::OK;
    }

    //
    // Export state
    //
    std::vector<hardware_interface::StateInterface>
    CarrierbotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_state_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_state_[i]);
        }

        return state_interfaces;
    }

    //
    // Export command
    //
    std::vector<hardware_interface::CommandInterface>
    CarrierbotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_command_[i]);
        }

        return command_interfaces;
    }

    //
    // 🔥 FOXy: start thay on_activate
    //
    hardware_interface::return_type CarrierbotInterface::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"), "Starting robot hardware");
        if (!can_interface_)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Failed to open CAN port: " << port_);
            return hardware_interface::return_type::ERROR;
        }
        
        try
        {
            can_interface_->open();
            RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"),
                                "CAN opened successfully on %s", port_.c_str());
            auto callback = [this](uint16_t can_id, const std::vector<uint8_t> &data)
            {
                if (can_id == 0x80)
                {
                    handleEncoderData(data);
                }
                else if (can_id == 0x85)
                {
                    handleVoltageData(data);
                }
            };
            can_interface_->start_receive_loop(callback);
        }

        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                        "Cannot open CAN: " << e.what());
            return hardware_interface::return_type::ERROR;
        }         
            
        return hardware_interface::return_type::OK;
    }

    //
    // 🔥 FOXy: stop thay on_deactivate
    //
    hardware_interface::return_type CarrierbotInterface::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("CarrierbotInterface"), "Stopping robot hardware");
        try
        {
            if (can_interface_)
            {
                can_interface_->close();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Failed to close CAN port: " << port_ << " error: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    //
    // Handle encoder data from CAN 0x80
    //
    void CarrierbotInterface::handleEncoderData(const std::vector<uint8_t> &data)
    {
        if (data.size() < 8)
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        std::memcpy(&raw_right_rps_,  &data[0], sizeof(float));
        std::memcpy(&raw_left_rps_, &data[4], sizeof(float));
    }
    //
    // Handle voltage data from CAN 0x85
    //
    void CarrierbotInterface::handleVoltageData(const std::vector<uint8_t> &data)
    {
        if (data.size() < 8)
            return;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            std::memcpy(&voltage,  &data[0], sizeof(float));
            telemetry_msg_.voltage = voltage;
        }
        publishTelemetry();
        std::cout << "Battery Voltage: " << voltage << " V\n";
    }

    //
    // 🔥 FOXy: read không có time, period
    //
    hardware_interface::return_type CarrierbotInterface::read()
    {
        rclcpp::Time now = rclcpp::Clock().now();

        // 👉 INIT lần đầu
        if (last_run_.nanoseconds() == 0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        double dt = (now - last_run_).seconds();
        if (dt <= 0.0 || dt > 1.0)
        {
            last_run_ = now;
            return hardware_interface::return_type::OK;
        }

        double left_raw, right_raw;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            left_raw = raw_left_rps_;
            right_raw = raw_right_rps_;
            telemetry_msg_.left_rps = static_cast<float>(left_raw) / 10.0f;
            telemetry_msg_.right_rps = static_cast<float>(right_raw) / 10.0f;
        }

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("CarrierbotInterface"),
        //     "CAN Receive Right: " << right_raw << " RPS | "
        //     " Left: "<< left_raw << " RPS");

        // Convert to rad/s
        double left_rad_s  = left_raw / 10.0 * 2 * M_PI;
        double right_rad_s = -right_raw / 10.0 * 2 * M_PI;

        // Update velocity
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            velocity_state_[0] = right_rad_s;
            velocity_state_[1] = left_rad_s;
        }
        publishTelemetry();

        // Integrate to position (rad)
        position_state_[0] += velocity_state_[0] * dt;
        position_state_[1] += velocity_state_[1] * dt;

        last_run_ = now;
        return hardware_interface::return_type::OK;
    }

    //
    // Send wheel velocities to CAN 0x60 and 0x70
    //
    void CarrierbotInterface::sendWheelVelocities()
    {
        if (!can_interface_ || velocity_command_.size() < 2)
                return;

            double left_cmd, right_cmd;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                right_cmd  = velocity_command_[0];
                left_cmd = velocity_command_[1];
            }

            float left_velocity  = static_cast<float>(left_cmd) * 10.0 / (2 * M_PI);
            float right_velocity = static_cast<float>(right_cmd) * 10.0 / (2 * M_PI) * (-1.0f); 
            std::cout << "Commanded left_vel: " << left_velocity << " RPS, right_vel: " << right_velocity << " RPS\n";
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                telemetry_msg_.left_velocity = left_velocity / 10.0f;
                telemetry_msg_.right_velocity = right_velocity / 10.0f;
            }
            publishTelemetry();

            std::vector<uint8_t> left_vel_data(8, 0);
            std::vector<uint8_t> right_vel_data(8, 0);

            std::memcpy(&left_vel_data[0], &left_velocity, sizeof(float));
            std::memcpy(&right_vel_data[0], &right_velocity, sizeof(float));

            can_interface_->send(0x70, left_vel_data);
            can_interface_->send(0x60, right_vel_data);

        // // Log CAN send
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("CarrierbotInterface"),
        //     "CAN Send 0x60(R): " << right_velocity << " RPS | "
        //     "0x70(L): " << left_velocity << " RPS");
    }

    //
    // 🔥 FOXy: write không có time, period
    //
    hardware_interface::return_type CarrierbotInterface::write()
    {
        try
        {
            sendWheelVelocities();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CarrierbotInterface"),
                                "Failed to send CAN command: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    //
    // 🔥 BẮT BUỘC TRONG FOXY
    //
    std::string CarrierbotInterface::get_name() const
    {
        return info_.name;
    }

    hardware_interface::status CarrierbotInterface::get_status() const
    {
        return hardware_interface::status::STARTED;
    }

    void CarrierbotInterface::publishTelemetry()
    {
        if (!telemetry_pub_)
        {
            return;
        }

        telemetry_pub_->publish(telemetry_msg_);
    }

} // namespace carrierbot_firmware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(carrierbot_firmware::CarrierbotInterface,
                       hardware_interface::SystemInterface)