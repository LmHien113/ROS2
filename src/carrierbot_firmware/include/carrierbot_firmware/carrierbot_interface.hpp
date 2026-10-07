#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <carrierbot_msgs/msg/carrierbot_telemetry.hpp>
#include "carrierbot_firmware/can_node.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <fstream>
#include <chrono>

namespace carrierbot_firmware
{
    class CarrierbotInterface : public hardware_interface::SystemInterface
    {
    public:
        CarrierbotInterface();
        virtual ~CarrierbotInterface();

        // Foxy uses configure instead of on_init
        hardware_interface::return_type configure(
            const hardware_interface::HardwareInfo & info) override;

        // Export interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Start/Stop instead of activate/deactivate
        hardware_interface::return_type start() override;
        hardware_interface::return_type stop() override;

        // Read/Write (no time, period in Foxy)
        hardware_interface::return_type read() override;
        hardware_interface::return_type write() override;

        // REQUIRED in Foxy
        std::string get_name() const override;
        hardware_interface::status get_status() const override;

    private:
        // Callback for wheel velocity commands
        void cmdVelWheelsCallback(const std::shared_ptr<const std_msgs::msg::Float64MultiArray> msg);
        void subscriptionSpinnerThread();

        // Helper functions
        void handleEncoderData(const std::vector<uint8_t> &data);
        void handleVoltageData(const std::vector<uint8_t> &data);
        void publishTelemetry();

        void sendWheelVelocities();
        void initCSV();
        void logToCSV(float left_encoder, float right_encoder);

        // Save hardware info manually (Foxy không có sẵn info_)
        hardware_interface::HardwareInfo info_;
        std::unique_ptr<WaveshareCAN> can_interface_;
        std::string port_;
        int baudrate_;
        std::vector<double> velocity_command_;
        std::vector<double> position_state_;
        std::vector<double> velocity_state_;
        carrierbot_msgs::msg::CarrierbotTelemetry telemetry_msg_;
        std::mutex state_mutex_;
        float raw_left_rps_ = 0.0f;
        float raw_right_rps_ = 0.0f;
        float voltage = 0.0f;
        rclcpp::Node::SharedPtr ros_node_;
        rclcpp::Publisher<carrierbot_msgs::msg::CarrierbotTelemetry>::SharedPtr telemetry_pub_;
        rclcpp::Time last_run_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_wheels_sub_;

        // Background thread for subscription processing
        std::thread subscription_thread_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        bool stop_subscription_thread_ = false;
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;

        // CSV logging
        std::ofstream csv_file_;
        std::chrono::system_clock::time_point start_time_;
        double last_left_cmd_rad_s_ = 0.0;
        double last_right_cmd_rad_s_ = 0.0;
        double last_left_cmd_rps_ = 0.0;
        double last_right_cmd_rps_ = 0.0;
        double last_left_encoder_rps_ = 0.0;
        double last_right_encoder_rps_ = 0.0;
        double last_left_encoder_rad_s_ = 0.0;
        double last_right_encoder_rad_s_ = 0.0;
        bool csv_ready_ = false;
    };
}