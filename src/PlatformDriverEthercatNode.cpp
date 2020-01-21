#include <PlatformDriverEthercat.h>
#include <sys/stat.h>
#include <sstream>
#include <set>

#include "PlatformDriverEthercatNode.h"
#include "yaml-cpp/yaml.h"

static std::stringstream ss;

using namespace platform_driver_ethercat;


PlatformDriverEthercatNode::PlatformDriverEthercatNode()
  : Node("platform_driver_ethercat")
{
    joint_readings_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_readings", 10);
    fts_readings_publisher_ = this->create_publisher<std::vector<geometry_msgs::msg::WrenchStamped>>("fts_readings", 10);
    temp_readings_publisher_ = this->create_publisher<std::vector<sensor_msgs::msg::Temperature>>("temp_readings", 10);

    configureHook();
    startHook();

    timer_ = this->create_wall_timer(
      10ms, std::bind(&PlatformDriverEthercatNode::updateHook, this));

    joint_commands_subscriber_ = this->create_subscription<rover_msgs::msg::JointCommandArray>("joint_commands", 10, std::bind(&PlatformDriverEthercatNode::evalJointCommands, this, std::placeholders::_1));
}

PlatformDriverEthercatNode::~PlatformDriverEthercatNode()
{
    platform_driver_->shutdownPlatform();
}

bool PlatformDriverEthercatNode::configureHook()
{
    YAML::Node config = YAML::LoadFile("marta.yaml");

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

    fts_readings_.resize(fts_mapping_.size());
    joint_readings_.resize(active_joint_mapping_.size() + passive_joint_mapping_.size());
    temp_readings_.resize(active_joint_mapping_.size());

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
        fts_readings_.names[i] = fts.name;
        platform_driver_->addAtiFts(fts.slave_id, fts.name);
        ++i;
    }

    // Fill the joint and temp output names with the joint mapping names, prepare the moving average
    // filter and add the joints to platform driver
    i = 0;
    for (const auto& joint : active_joint_mapping_)
    {
        joint_readings_.names[i] = joint.name;
        temp_readings_.names[i] = joint.name;
        platform_driver_->addActiveJoint(joint.name, joint.drive, joint.params, joint.enabled);
        ++i;
    }

    std::array<double, window_size> temp_array;
    temp_array.fill(0.0);
    temp_values.assign(active_joint_mapping_.size(), temp_array);
    temp_sums.assign(active_joint_mapping_.size(), 0.0);

    for (const auto& joint : passive_joint_mapping_)
    {
        joint_readings_.names[i] = joint.name;
        platform_driver_->addPassiveJoint(joint.name, joint.drive, joint.enabled);
        ++i;
    }

    return true;
}

bool PlatformDriverEthercatNode::startHook()
{
    if (platform_driver_->initPlatform())
    {
        return true;
    }

    return false;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    updateJointReadings();
    updateFtsReadings();
    updateTempReadings();

    joint_readings_.time = base::Time::now();
    _joints_readings.write(joint_readings_);

    fts_readings_.time = base::Time::now();
    _fts_readings.write(fts_readings_);

    temp_readings_.time = base::Time::now();
    _temp_readings.write(temp_readings_);

    // LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": " << base::Time::now();
    //auto message = std_msgs::msg::String();
    //message.data = "Hello, world! " + std::to_string(count_++);
    //publisher_->publish(message);
}

bool Task::validateConfig()
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

void Task::evalJointCommands(const rover_msgs::msg::JointCommandArray::SharedPtr joint_commands)
{
    base::commands::Joints joint_commands;

    if (_joints_commands.readNewest(joint_commands, false) == RTT::NewData)
    {
        for (size_t i = 0; i < joint_commands.size(); ++i)
        {
            base::JointState& joint(joint_commands[i]);

            if (joint.isPosition())
            {
                platform_driver_->commandJointPositionRad(joint_commands.names[i], joint.position);
            }
            else if (joint.isSpeed())
            {
                platform_driver_->commandJointVelocityRadSec(joint_commands.names[i], joint.speed);
            }
        }
    }
}

void Task::updateJointReadings()
{
    size_t i = 0;

    for (const auto& joint : active_joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        base::JointState& joint_state(joint_readings_[i]);
        joint_state.position = position;
        joint_state.speed = velocity;
        joint_state.effort = torque;

        ++i;
    }

    for (const auto& joint : passive_joint_mapping_)
    {
        double position, velocity, torque;

        platform_driver_->readJointPositionRad(joint.name, position);
        platform_driver_->readJointVelocityRadSec(joint.name, velocity);
        platform_driver_->readJointTorqueNm(joint.name, torque);

        base::JointState& joint_state(joint_readings_[i]);
        joint_state.position = position;
        joint_state.speed = velocity;
        joint_state.effort = torque;

        ++i;
    }
}

void Task::updateFtsReadings()
{
    size_t i = 0;
    for (const auto& fts_params : fts_mapping_)
    {
        double fx, fy, fz;
        double tx, ty, tz;

        platform_driver_->readFtsForceN(fts_params.name, fx, fy, fz);
        platform_driver_->readFtsTorqueNm(fts_params.name, tx, ty, tz);

        base::Wrench& wrench(fts_readings_[i]);
        wrench.force = base::Vector3d(fx, fy, fz);
        wrench.torque = base::Vector3d(tx, ty, tz);

        ++i;
    }
}

void Task::updateTempReadings()
{
    size_t i = 0;
    for (const auto& joint : active_joint_mapping_)
    {
        double joint_temp;
        platform_driver_->readJointTempDegC(joint.name, joint_temp);

        temp_sums[i] += joint_temp - temp_values[i][temp_index];
        temp_values[i][temp_index] = joint_temp;

        if (first_window)
        {
            temp_readings_[i] = temp_sums[i] / (1.0 * (temp_index + 1));
        }
        else
        {
            temp_readings_[i] = temp_sums[i] / (1.0 * window_size);
        }

        ++i;
    }

    if (first_window && temp_index == window_size - 1)
    {
        first_window = false;
    }

    ++temp_index %= window_size;
}
