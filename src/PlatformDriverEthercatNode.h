#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joint_command_array.hpp"
#include "rover_msgs/msg/temperature_array.hpp"
#include "rover_msgs/msg/wrench_stamped_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "PlatformDriverEthercatTypes.h"
#include "PlatformDriverEthercatNodeTypes.h"

using namespace std::chrono_literals;

namespace platform_driver_ethercat
{

    class PlatformDriverEthercat;

    class PlatformDriverEthercatNode : public rclcpp::Node
    {
        public:
            PlatformDriverEthercatNode();
            ~PlatformDriverEthercatNode();

        private:
            bool configureHook();
            bool startHook();
            void updateHook();
            bool validateConfig();
            void evalJointCommands(const rover_msgs::msg::JointCommandArray::SharedPtr joint_commands);
            void updateJointReadings();
            void updateFtsReadings();
            void updateTempReadings();

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_readings_publisher_;
            rclcpp::Publisher<rover_msgs::msg::WrenchStampedArray>::SharedPtr fts_readings_publisher_;
            rclcpp::Publisher<rover_msgs::msg::TemperatureArray>::SharedPtr temp_readings_publisher_;
            rclcpp::Subscription<rover_msgs::msg::JointCommandArray>::SharedPtr joint_commands_subscriber_;

            sensor_msgs::msg::JointState joint_readings_;
            rover_msgs::msg::WrenchStampedArray fts_readings_;
            rover_msgs::msg::TemperatureArray temp_readings_;

            std::string network_interface_;
            unsigned int num_slaves_;
            DriveSlaveMapping drive_mapping_;
            FtsSlaveMapping fts_mapping_;
            ActiveJointMapping active_joint_mapping_;
            PassiveJointMapping passive_joint_mapping_;

            std::unique_ptr<PlatformDriverEthercat> platform_driver_;

            // moving average filter
            static const int window_size = 1000;
            std::vector<std::array<double, window_size>> temp_values;
            std::vector<double> temp_sums;
            int temp_index = 0;
            bool first_window = true;

    };
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<platform_driver_ethercat::PlatformDriverEthercatNode>());
    rclcpp::shutdown();
    return 0;
}
