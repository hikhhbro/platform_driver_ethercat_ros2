#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "EthercatInterface.h"
#include "PlatformDriverEthercatTypes.h"

namespace platform_driver_ethercat
{

class CanDeviceAtiFts;
class CanDriveTwitter;
class Joint;
class JointActive;
class JointPassive;

/**
 * Represents and Controls all Drive components on an arbitrary platform.
 * Drives shall be connected in a CAN Bus network and comply with the CANopen protocol to control
 * different types of motors.
 */
class PlatformDriverEthercat
{
  public:
    /**
     * Default constructor.
     */
    PlatformDriverEthercat(std::string can_address, unsigned int num_slaves);

    /**
     * Default destructor.
     */
    ~PlatformDriverEthercat();

    void addDriveTwitter(unsigned int slave_id, std::string name, DriveParams params);

    void addAtiFts(unsigned int slave_id, std::string name);

    void addActiveJoint(std::string name,
                        std::string drive,
                        ActiveJointParams params,
                        bool enabled);

    void addPassiveJoint(std::string name, std::string drive, bool enabled);

    /**
     * Initializes the ethercat interface and starts up the drives.
     * @return True if initialization is successful, false otherwise.
     */
    bool initPlatform();

    /**
     * Starts up the platform.
     * Enables the motors
     * @return True if all drive are properly started.
     */
    bool startupPlatform();

    /**
     * Shuts down the platform.
     * Disables the motors and closes the interface.
     * @return True if all drives are properly shut down.
     */
    bool shutdownPlatform();

    /**
     * Reinitializes the nodes on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    bool resetPlatform();

    /**
     * Sends position command for specific joint.
     */
    bool commandJointPositionRad(std::string joint_name, double position_rad);

    /**
     * Sends velocity command for specific joint.
     */
    bool commandJointVelocityRadSec(std::string joint_name, double velocity_rad_sec);

    /**
     * Sends torque command for specific joint.
     */
    bool commandJointTorqueNm(std::string joint_name, double torque_nm);

    /**
     * Gets the position and velocity of a given joint.
     */
    bool readJointPositionRad(std::string joint_name, double& position_rad);

    /**
     * Gets the velocity of a given joint.
     */
    bool readJointVelocityRadSec(std::string joint_name, double& velocity_rad_sec);

    /**
     * Gets the motor torque (from active current) of a given joint.
     */
    bool readJointTorqueNm(std::string joint_name, double& torque_nm);

    bool readJointTempDegC(std::string joint_name, double& temp_deg_c);

    void readFtsForceN(std::string fts_name, double& fx, double& fy, double& fz);

    void readFtsTorqueNm(std::string fts_name, double& tx, double& ty, double& tz);

  private:
    std::map<std::string, std::shared_ptr<CanDriveTwitter>> can_drives_;
    std::map<std::string, std::shared_ptr<CanDeviceAtiFts>> can_fts_;
    std::map<std::string, std::shared_ptr<Joint>> joints_;
    std::map<std::string, std::shared_ptr<JointActive>> active_joints_;
    std::map<std::string, std::shared_ptr<JointPassive>> passive_joints_;

    EthercatInterface ethercat_;
};
}
