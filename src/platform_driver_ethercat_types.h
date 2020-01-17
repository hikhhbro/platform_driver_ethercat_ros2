#pragma once

#include <PlatformDriverEthercatTypes.h>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>
#include <string>
#include <vector>

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

struct Temperatures : public base::NamedVector<double>
{
    base::Time time;
};
}
