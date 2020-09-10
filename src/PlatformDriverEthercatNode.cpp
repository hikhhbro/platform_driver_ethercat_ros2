#include "PlatformDriverEthercatNode.h"

#include <sys/stat.h>
#include <sstream>
#include <set>

#include "PlatformDriverEthercat.h"
#include "yaml-cpp/yaml.h"

static std::stringstream ss;

using namespace platform_driver_ethercat;

PlatformDriverEthercatNode::PlatformDriverEthercatNode()
    : LifecycleNode("platform_driver_ethercat")
{
    this->declare_parameter("config_file", "");
}

PlatformDriverEthercatNode::~PlatformDriverEthercatNode()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_configure(const State &)
{
    if (configureHook())
    {
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration of platform driver successful");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration of platform driver failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    joint_readings_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    fts_readings_publisher_ = this->create_publisher<rover_msgs::msg::WrenchStampedArray>("fts_readings", 10);
    temp_readings_publisher_ = this->create_publisher<rover_msgs::msg::TemperatureArray>("temp_readings", 10);

    joint_commands_subscriber_ = this->create_subscription<rover_msgs::msg::JointCommandArray>("joint_cmds", 10, std::bind(&PlatformDriverEthercatNode::evalJointCommands, this, std::placeholders::_1));

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_activate(const State &)
{
    if (!startHook())
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

    joint_readings_publisher_->on_activate();
    fts_readings_publisher_->on_activate();
    temp_readings_publisher_->on_activate();

    timer_ = this->create_wall_timer(
            10ms, std::bind(&PlatformDriverEthercatNode::updateHook, this));

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_deactivate(const State &)
{
    joint_readings_publisher_->on_deactivate();
    fts_readings_publisher_->on_deactivate();
    temp_readings_publisher_->on_deactivate();

    timer_.reset();

    stopHook();

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_cleanup(const State &)
{
    cleanupHook();

    joint_readings_publisher_.reset();
    fts_readings_publisher_.reset();
    temp_readings_publisher_.reset();

    joint_commands_subscriber_.reset();

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_shutdown(const State &)
{
    std::cout << "shutting down" << std::endl;

    if (platform_driver_ != NULL)
    {
        platform_driver_->shutdownPlatform();
    }

    joint_readings_publisher_.reset();
    fts_readings_publisher_.reset();
    temp_readings_publisher_.reset();

    joint_commands_subscriber_.reset();

    timer_.reset();

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn PlatformDriverEthercatNode::on_error(const State &)
{
    errorHook();

    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool PlatformDriverEthercatNode::configureHook()
{
    std::string config_file = this->get_parameter("config_file").as_string();

    YAML::Node config = YAML::LoadFile(config_file);

    if (!config["network_interface"]
            || !config["num_slaves"]
            || !config["drive_mapping"]
            || !config["fts_mapping"]
            || !config["active_joint_mapping"]
            || !config["passive_joint_mapping"])
        return false;

    // Read configuration
    network_interface_ = config["network_interface"].as<std::string>();
    num_slaves_ = config["num_slaves"].as<int>();
    drive_mapping_ = config["drive_mapping"].as<DriveSlaveMapping>();
    fts_mapping_ = config["fts_mapping"].as<FtsSlaveMapping>();
    active_joint_mapping_ = config["active_joint_mapping"].as<ActiveJointMapping>();
    passive_joint_mapping_ = config["passive_joint_mapping"].as<PassiveJointMapping>();

    if (!validateConfig())
    {
        return false;
    }

    fts_readings_.wrenches.resize(fts_mapping_.size());
    joint_readings_.name.resize(active_joint_mapping_.size() + passive_joint_mapping_.size());
    joint_readings_.position.resize(active_joint_mapping_.size() + passive_joint_mapping_.size());
    joint_readings_.velocity.resize(active_joint_mapping_.size() + passive_joint_mapping_.size());
    joint_readings_.effort.resize(active_joint_mapping_.size() + passive_joint_mapping_.size());
    temp_readings_.temperatures.resize(active_joint_mapping_.size());

    platform_driver_.reset(new PlatformDriverEthercat(network_interface_, num_slaves_));

    // Add the drives to the platform driver
    for (const auto& drive : drive_mapping_)
    {
        platform_driver_->addDriveTwitter(drive.slave_id, drive.name, drive.params);
    }

    // Fill the fts output names with the fts mapping names and add the fts to platform driver
    size_t i = 0;
    for (const auto& fts : fts_mapping_)
    {
        fts_readings_.wrenches[i].header.frame_id = fts.name;
        platform_driver_->addAtiFts(fts.slave_id, fts.name);
        ++i;
    }

    // Fill the joint and temp output names with the joint mapping names, prepare the moving average
    // filter and add the joints to platform driver
    i = 0;
    for (const auto& joint : active_joint_mapping_)
    {
        joint_readings_.name[i] = joint.name;
        temp_readings_.temperatures[i].header.frame_id = joint.name;
        platform_driver_->addActiveJoint(joint.name, joint.drive, joint.params, joint.enabled);
        ++i;
    }

    std::array<double, window_size> temp_array;
    temp_array.fill(0.0);
    temp_values.assign(active_joint_mapping_.size(), temp_array);
    temp_sums.assign(active_joint_mapping_.size(), 0.0);

    for (const auto& joint : passive_joint_mapping_)
    {
        joint_readings_.name[i] = joint.name;
        platform_driver_->addPassiveJoint(joint.name, joint.drive, joint.enabled);
        ++i;
    }

    if (platform_driver_->initPlatform())
    {
        return true;
    }
    else
    {
        platform_driver_.reset();
        return false;
    }
}

bool PlatformDriverEthercatNode::startHook()
{
    return platform_driver_->startupPlatform();
}

void PlatformDriverEthercatNode::updateHook()
{
    updateJointReadings();
    updateFtsReadings();
    updateTempReadings();

    joint_readings_.header.stamp = this->now();
    joint_readings_publisher_->publish(joint_readings_);

    fts_readings_.header.stamp = this->now();
    fts_readings_publisher_->publish(fts_readings_);

    temp_readings_.header.stamp = this->now();
    temp_readings_publisher_->publish(temp_readings_);
}

void PlatformDriverEthercatNode::errorHook()
{
    platform_driver_->shutdownPlatform();
}

void PlatformDriverEthercatNode::stopHook()
{
    platform_driver_->shutdownPlatform();
}

void PlatformDriverEthercatNode::cleanupHook()
{
    platform_driver_.reset();
}

bool PlatformDriverEthercatNode::validateConfig()
{
    // Check if interface exists
    struct stat buffer;
    if (stat(("/sys/class/net/" + network_interface_).c_str(), &buffer) != 0)
    {
        ss << ": Interface " << network_interface_
            << " does not exist";
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
        ss.str(""); ss.clear();
        return false;
    }

    // Check if num slaves is valid
    if (num_slaves_ <= 0)
    {
        ss << ": Invalid number of slaves " << num_slaves_;
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
        ss.str(""); ss.clear();
        return false;
    }

    std::set<unsigned int> id_set;
    std::set<std::string> name_set;

    auto validateDevice = [&id_set, &name_set](SlaveConfig config) {
        const auto& slave_id = config.slave_id;
        const auto& name = config.name;

        // Check if slave id is valid
        if (slave_id <= 0)
        {
            ss << ": Invalid slave id " << slave_id;
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }

        // Check if slave id already exists
        if (id_set.find(slave_id) != id_set.end())
        {
            ss << ": Slave id " << slave_id << " already exists";
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }

        id_set.insert(slave_id);

        // Check if device name already exists
        if (name_set.find(name) != name_set.end())
        {
            ss << ": Device name " << name << " already exists";
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }

        name_set.insert(name);

        return true;
    };

    std::set<std::string> drive_set;
    for (const auto& drive : drive_mapping_)
    {
        if (!validateDevice(drive)) return false;
        drive_set.insert(drive.name);
    }

    for (const auto& fts : fts_mapping_)
    {
        if (!validateDevice(fts)) return false;
    }

    std::set<std::string> joint_set, active_set, passive_set;

    auto validateJoint = [&drive_set, &joint_set](JointConfig config,
            std::set<std::string>& current_set) {
        const auto& name = config.name;
        const auto& drive = config.drive;

        // Check if joint name already exists
        if (joint_set.find(name) != joint_set.end())
        {
            ss << ": Joint name " << name << " already exists";
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }
        joint_set.insert(name);

        // Check if drive name does not exist
        if (drive_set.find(drive) == drive_set.end())
        {
            ss << ": Drive " << drive << " for joint " << name
                << " does not exist";
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }

        // Check if the same drive is already in use for another joint of the current set
        if (current_set.find(drive) != current_set.end())
        {
            ss << ": Drive " << drive
                << " already in use with another joint of the same type";
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "%s", ss.str().c_str());
            ss.str(""); ss.clear();
            return false;
        }

        current_set.insert(drive);

        return true;
    };

    for (const auto& joint : active_joint_mapping_)
    {
        if (!validateJoint(joint, active_set)) return false;
    }

    for (const auto& joint : passive_joint_mapping_)
    {
        if (!validateJoint(joint, passive_set)) return false;
    }

    return true;
}

void PlatformDriverEthercatNode::evalJointCommands(const rover_msgs::msg::JointCommandArray::SharedPtr joint_commands)
{
    for (size_t i = 0; i < joint_commands->joint_command_array.size(); ++i)
    {
        rover_msgs::msg::JointCommand& joint(joint_commands->joint_command_array[i]);

        if (joint.mode == "POSITION")
        {
            platform_driver_->commandJointPositionRad(joint.name, joint.value);
        }
        else if (joint.mode == "VELOCITY")
        {
            platform_driver_->commandJointVelocityRadSec(joint.name, joint.value);
        }
    }
}

void PlatformDriverEthercatNode::updateJointReadings()
{
    size_t i = 0;

    for (const auto& joint : active_joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        joint_readings_.position[i] = position;
        joint_readings_.velocity[i] = velocity;
        joint_readings_.effort[i] = torque;

        ++i;
    }

    for (const auto& joint : passive_joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        joint_readings_.position[i] = position;
        joint_readings_.velocity[i] = velocity;
        joint_readings_.effort[i] = torque;

        ++i;
    }
}

void PlatformDriverEthercatNode::updateFtsReadings()
{
    size_t i = 0;
    for (const auto& fts_params : fts_mapping_)
    {
        double fx, fy, fz;
        double tx, ty, tz;

        platform_driver_->readFtsForceN(fts_params.name, fx, fy, fz);
        platform_driver_->readFtsTorqueNm(fts_params.name, tx, ty, tz);

        geometry_msgs::msg::WrenchStamped& wrench_msg(fts_readings_.wrenches[i]);

        wrench_msg.header.stamp = this->now();

        wrench_msg.wrench.force.x = fx;
        wrench_msg.wrench.force.y = fy;
        wrench_msg.wrench.force.z = fz;

        wrench_msg.wrench.torque.x = tx;
        wrench_msg.wrench.torque.y = ty;
        wrench_msg.wrench.torque.z = tz;

        ++i;
    }
}

void PlatformDriverEthercatNode::updateTempReadings()
{
    size_t i = 0;
    for (const auto& joint : active_joint_mapping_)
    {
        double joint_temp;
        platform_driver_->readJointTempDegC(joint.name, joint_temp);

        sensor_msgs::msg::Temperature& temperature_msg(temp_readings_.temperatures[i]);

        temperature_msg.header.stamp = this->now();

        temp_sums[i] += joint_temp - temp_values[i][temp_index];
        temp_values[i][temp_index] = joint_temp;

        if (first_window)
        {
            temperature_msg.temperature = temp_sums[i] / (1.0 * (temp_index + 1));
        }
        else
        {
            temperature_msg.temperature = temp_sums[i] / (1.0 * window_size);
        }

        ++i;
    }

    if (first_window && temp_index == window_size - 1)
    {
        first_window = false;
    }

    ++temp_index %= window_size;
}
