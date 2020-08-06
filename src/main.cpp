#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "PlatformDriverEthercatNode.h"

std::unique_ptr<platform_driver_ethercat::PlatformDriverEthercatNode> platform_driver;

void signalHandler(int /*signum*/)
{
    platform_driver->shutdown();
}

int main(int argc, char * argv[])
{
    // set stdout to unbuffered as a workaround to https://github.com/ros2/rcutils/issues/168
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // install signal handler to cleanly shutdown lifecycle node
    signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    platform_driver = std::make_unique<platform_driver_ethercat::PlatformDriverEthercatNode>();
    rclcpp::spin(platform_driver->get_node_base_interface());

    platform_driver.reset();
    rclcpp::shutdown();

    return 0;
}
