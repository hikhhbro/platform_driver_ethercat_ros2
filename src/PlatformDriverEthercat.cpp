#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <future>
#include <iostream>
#include <tuple>
#include <vector>

#include "CanDeviceAtiFts.h"
#include "CanDriveTwitter.h"
#include "EthercatInterface.h"
#include "JointActive.h"
#include "JointPassive.h"
#include "PlatformDriverEthercat.h"

#include <base-logging/Logging.hpp>

using namespace platform_driver_ethercat;

PlatformDriverEthercat::PlatformDriverEthercat(std::string dev_address, unsigned int num_slaves)
    : ethercat_(dev_address, num_slaves)
{
}

PlatformDriverEthercat::~PlatformDriverEthercat() {}

void PlatformDriverEthercat::addDriveTwitter(unsigned int slave_id,
                                             std::string name,
                                             DriveParams params)
{
    auto drive = std::make_shared<CanDriveTwitter>(ethercat_, slave_id, name, params);
    can_drives_.insert(std::make_pair(drive->getDeviceName(), drive));
    ethercat_.addDevice(drive);
}

void PlatformDriverEthercat::addAtiFts(unsigned int slave_id, std::string name)
{
    auto fts = std::make_shared<CanDeviceAtiFts>(ethercat_, slave_id, name);
    can_fts_.insert(std::make_pair(fts->getDeviceName(), fts));
    ethercat_.addDevice(fts);
}

void PlatformDriverEthercat::addActiveJoint(std::string name,
                                            std::string drive,
                                            ActiveJointParams params,
                                            bool enabled)
{
    std::shared_ptr<JointActive> joint(
        new JointActive(name, can_drives_.at(drive), params, enabled));
    joints_.insert(std::make_pair(joint->getName(), joint));
    active_joints_.insert(std::make_pair(joint->getName(), joint));
}

void PlatformDriverEthercat::addPassiveJoint(std::string name, std::string drive, bool enabled)
{
    std::shared_ptr<JointPassive> joint(new JointPassive(name, can_drives_.at(drive), enabled));
    joints_.insert(std::make_pair(joint->getName(), joint));
    passive_joints_.insert(std::make_pair(joint->getName(), joint));
}

bool PlatformDriverEthercat::initPlatform()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Initializing EtherCAT interface";

    if (!ethercat_.init())
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not initialize EtherCAT interface";
        return false;
    }

    if (!startupPlatform())
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not start up drives";
        return false;
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Platform init success";
    return true;
}

bool PlatformDriverEthercat::startupPlatform()
{
    // Start all drives for enabled active joints in groups of 6
    auto joint_iterator = active_joints_.begin();
    while (joint_iterator != active_joints_.end())
    {
        std::vector<std::tuple<CanDriveTwitter&, std::future<bool>>> future_tuples;

        unsigned int j = 0;

        while (j < 6)
        {
            auto active_joint = joint_iterator->second;

            if (active_joint->isEnabled())
            {
                CanDriveTwitter& drive = *(active_joint->getDrive());

                auto future = std::async(std::launch::async, &CanDriveTwitter::startup, &drive);
                auto tuple =
                    std::tuple<CanDriveTwitter&, std::future<bool>>(drive, std::move(future));

                future_tuples.push_back(std::move(tuple));

                j++;
            }

            joint_iterator++;

            if (joint_iterator == active_joints_.end())
            {
                break;
            }
        }

        for (auto& future_tuple : future_tuples)
        {
            std::future<bool>& future = std::get<1>(future_tuple);

            if (!future.get())
            {
                return false;
            }
        }
    }

    return true;
}

bool PlatformDriverEthercat::shutdownPlatform()
{
    bool bRet = true;
    // shut down all motors
    for (auto& drive : can_drives_)
    {
        bRet &= drive.second->shutdown();
    }
    return bRet;
}

bool PlatformDriverEthercat::resetPlatform()
{
    bool bRetMotor = true;
    bool bRet = true;

    for (auto& drive : can_drives_)
    {
        bRetMotor = drive.second->reset();

        if (!bRetMotor)
        {
            LOG_ERROR_S << "Resetting of Motor " << drive.second->getDeviceName() << " failed";
        }

        bRet &= bRetMotor;
    }

    return bRet;
}

bool PlatformDriverEthercat::commandJointPositionRad(std::string joint_name, double position_rad)
{
    return joints_.at(joint_name)->commandPositionRad(position_rad);
}

bool PlatformDriverEthercat::commandJointVelocityRadSec(std::string joint_name,
                                                        double velocity_rad_sec)
{
    return joints_.at(joint_name)->commandVelocityRadSec(velocity_rad_sec);
}

bool PlatformDriverEthercat::commandJointTorqueNm(std::string joint_name, double torque_nm)
{
    return joints_.at(joint_name)->commandTorqueNm(torque_nm);
}

bool PlatformDriverEthercat::readJointPositionRad(std::string joint_name, double& position_rad)
{
    return joints_.at(joint_name)->readPositionRad(position_rad);
}

bool PlatformDriverEthercat::readJointVelocityRadSec(std::string joint_name,
                                                     double& velocity_rad_sec)
{
    return joints_.at(joint_name)->readVelocityRadSec(velocity_rad_sec);
}

bool PlatformDriverEthercat::readJointTorqueNm(std::string joint_name, double& torque_nm)
{
    return joints_.at(joint_name)->readTorqueNm(torque_nm);
}

bool PlatformDriverEthercat::readJointTempDegC(std::string joint_name, double& temp_deg_c)
{
    return joints_.at(joint_name)->readTempDegC(temp_deg_c);
}

void PlatformDriverEthercat::readFtsForceN(std::string fts_name, double& fx, double& fy, double& fz)
{
    Eigen::Vector3d force = can_fts_.at(fts_name)->readForceN();

    fx = force[0];
    fy = force[1];
    fz = force[2];
}

void PlatformDriverEthercat::readFtsTorqueNm(std::string fts_name,
                                             double& tx,
                                             double& ty,
                                             double& tz)
{
    Eigen::Vector3d torque = can_fts_.at(fts_name)->readTorqueNm();

    tx = torque[0];
    ty = torque[1];
    tz = torque[2];
}
