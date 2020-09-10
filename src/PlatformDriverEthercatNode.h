#pragma once

#include <chrono>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rover_msgs/msg/joint_command_array.hpp"
#include "rover_msgs/msg/temperature_array.hpp"
#include "rover_msgs/msg/wrench_stamped_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "PlatformDriverEthercatTypes.h"
#include "PlatformDriverEthercatNodeTypes.h"

using namespace std::chrono_literals;
using namespace rclcpp_lifecycle;

namespace platform_driver_ethercat
{
    class PlatformDriverEthercat;

    class PlatformDriverEthercatNode : public LifecycleNode
    {
        public:
            PlatformDriverEthercatNode();
            ~PlatformDriverEthercatNode();

        private:
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State &);
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const State &);
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State &);
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State &);
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State &);
            node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State &);

            bool configureHook();
            bool startHook();
            void updateHook();
            void errorHook();
            void stopHook();
            void cleanupHook();

            bool validateConfig();
            void evalJointCommands(const rover_msgs::msg::JointCommandArray::SharedPtr joint_commands);
            void updateJointReadings();
            void updateFtsReadings();
            void updateTempReadings();

            rclcpp::TimerBase::SharedPtr timer_;
            LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_readings_publisher_;
            LifecyclePublisher<rover_msgs::msg::WrenchStampedArray>::SharedPtr fts_readings_publisher_;
            LifecyclePublisher<rover_msgs::msg::TemperatureArray>::SharedPtr temp_readings_publisher_;
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
