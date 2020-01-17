#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <base/samples/Joints.hpp>
#include <base/samples/Wrenches.hpp>

#include "PlatformDriverEthercatTypes.h"
#include "platform_driver_ethercat_types.h"

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
    bool updateHook();
    void timer_callback();
    bool validateConfig();
    void evalJointCommands();
    void updateJointReadings();
    void updateFtsReadings();
    void updateTempReadings();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<base::samples::Joints>::SharedPtr joint_readings_publisher_;
    rclcpp::Publisher<base::samples::Wrenches>::SharedPtr fts_readings_publisher_;
    rclcpp::Publisher<Temperatures>::SharedPtr temp_readings_publisher_;

    base::samples::Joints joint_readings_;
    base::samples::Wrenches fts_readings_;
    Temperatures temp_readings_;

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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformDriverEthercatNode>());
    rclcpp::shutdown();
    return 0;
}
