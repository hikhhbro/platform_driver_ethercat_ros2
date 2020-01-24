#pragma once

#include <PlatformDriverEthercatTypes.h>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace platform_driver_ethercat
{
    struct SlaveConfig
    {
        unsigned int slave_id;
        std::string name;
    };

    struct DriveSlaveConfig : public SlaveConfig
    {
        DriveParams params;
        bool enabled;
    };

    struct FtsSlaveConfig : public SlaveConfig
    {
    };

    struct JointConfig
    {
        std::string name;
        std::string drive;
        bool enabled;
    };

    struct ActiveJointConfig : public JointConfig
    {
        ActiveJointParams params;
    };

    struct PassiveJointConfig : public JointConfig
    {
    };

    typedef std::vector<DriveSlaveConfig> DriveSlaveMapping;
    typedef std::vector<FtsSlaveConfig> FtsSlaveMapping;
    typedef std::vector<ActiveJointConfig> ActiveJointMapping;
    typedef std::vector<PassiveJointConfig> PassiveJointMapping;
}

namespace YAML
{
    using namespace platform_driver_ethercat;

    template<>
        struct convert<DriveParams>
        {
            static bool decode(const Node& node, DriveParams& params)
            {
                if (!node.IsMap())
                    return false;

                if (!node["min_position_deg"]
                        || !node["max_position_deg"]
                        || !node["max_motor_speed_rpm"]
                        || !node["max_motor_current_amp"]
                        || !node["motor_rated_current_amp"]
                        || !node["motor_rated_torque_nm"]
                        || !node["gear_ratio"]
                        || !node["encoder_increments"]
                        || !node["encoder_on_output"]
                        || !node["profile_velocity_rad_sec"]
                        || !node["profile_acceleration_rad_sec_sec"])
                    return false;

                params.min_position_deg = node["min_position_deg"].as<double>();
                params.max_position_deg = node["max_position_deg"].as<double>();
                params.max_motor_speed_rpm = node["max_motor_speed_rpm"].as<int>();
                params.max_motor_current_amp = node["max_motor_current_amp"].as<double>();
                params.motor_rated_current_amp = node["motor_rated_current_amp"].as<double>();
                params.motor_rated_torque_nm = node["motor_rated_torque_nm"].as<double>();
                params.gear_ratio = node["gear_ratio"].as<double>();
                params.encoder_increments = node["encoder_increments"].as<int>();
                params.encoder_on_output = node["encoder_on_output"].as<bool>();
                params.profile_velocity_rad_sec = node["profile_velocity_rad_sec"].as<double>();
                params.profile_acceleration_rad_sec_sec = node["profile_acceleration_rad_sec_sec"].as<double>();

                return true;
            }
        };

    template<>
        struct convert<DriveSlaveMapping>
        {
            static bool decode(const Node& node, DriveSlaveMapping& mapping)
            {
                if (!node.IsSequence())
                    return false;

                for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                {
                    if (!it->IsMap())
                        return false;

                    if (!(*it)["slave_id"]
                            || !(*it)["name"]
                            || !(*it)["params"])
                        return false;

                    DriveSlaveConfig config;

                    config.slave_id = (*it)["slave_id"].as<int>();
                    config.name = (*it)["name"].as<std::string>();
                    config.params = (*it)["params"].as<DriveParams>();

                    mapping.push_back(config);
                }

                return true;
            }
        };

    template<>
        struct convert<FtsSlaveMapping>
        {
            static bool decode(const Node& node, FtsSlaveMapping& mapping)
            {
                if (!node.IsSequence())
                    return false;

                for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                {
                    if (!it->IsMap())
                        return false;

                    if (!(*it)["slave_id"] || !(*it)["name"])
                        return false;

                    FtsSlaveConfig config;

                    config.slave_id = (*it)["slave_id"].as<int>();
                    config.name = (*it)["name"].as<std::string>();

                    mapping.push_back(config);
                }

                return true;
            }
        };

    template<>
        struct convert<ActiveJointParams>
        {
            static bool decode(const Node& node, ActiveJointParams& params)
            {
                if (!node.IsMap())
                    return false;

                if (!node["flip_sign"]
                        || !node["min_position_command_rad"]
                        || !node["max_position_command_rad"]
                        || !node["max_velocity_command_rad_sec"]
                        || !node["max_torque_command_nm"])
                    return false;

                params.flip_sign = node["flip_sign"].as<bool>();
                params.min_position_command_rad = node["min_position_command_rad"].as<double>();
                params.max_position_command_rad = node["max_position_command_rad"].as<double>();
                params.max_velocity_command_rad_sec = node["max_velocity_command_rad_sec"].as<double>();
                params.max_torque_command_nm = node["max_torque_command_nm"].as<double>();

                return true;
            }
        };

    template<>
        struct convert<ActiveJointMapping>
        {
            static bool decode(const Node& node, ActiveJointMapping& mapping)
            {
                if (!node.IsSequence())
                    return false;

                for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                {
                    if (!it->IsMap())
                        return false;

                    if (!(*it)["name"]
                            || !(*it)["drive"]
                            || !(*it)["params"] 
                            || !(*it)["temp_offset_deg_c"]
                            || !(*it)["enabled"])
                        return false;

                    ActiveJointConfig config;

                    config.name = (*it)["name"].as<std::string>();
                    config.drive = (*it)["drive"].as<std::string>();

                    config.params = (*it)["params"].as<ActiveJointParams>();
                    config.enabled = (*it)["enabled"].as<bool>();
                    config.params.temp_offset_deg_c = (*it)["temp_offset_deg_c"].as<double>();

                    mapping.push_back(config);
                }

                return true;
            }
        };

    template<>
        struct convert<PassiveJointMapping>
        {
            static bool decode(const Node& node, PassiveJointMapping& mapping)
            {
                if (!node.IsSequence())
                    return false;

                for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                {
                    if (!it->IsMap())
                        return false;

                    if (!(*it)["name"] || !(*it)["drive"] || !(*it)["enabled"])
                        return false;

                    PassiveJointConfig config;

                    config.name = (*it)["name"].as<std::string>();
                    config.drive = (*it)["drive"].as<std::string>();
                    config.enabled = (*it)["enabled"].as<bool>();

                    mapping.push_back(config);
                }

                return true;
            }
        };
}
