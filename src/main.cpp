#include "rclcpp/rclcpp.hpp"
#include "PlatformDriverEthercatNode.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<platform_driver_ethercat::PlatformDriverEthercatNode>()->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
