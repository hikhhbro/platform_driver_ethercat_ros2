#pragma once

#include "Joint.h"
#include "PlatformDriverEthercatTypes.h"

namespace platform_driver_ethercat
{
class JointActive : public Joint
{
  public:
    JointActive(std::string name,
                std::shared_ptr<CanDriveTwitter>& drive,
                ActiveJointParams params,
                bool enabled);

    bool commandPositionRad(double position_rad);
    bool commandVelocityRadSec(double velocity_rad_sec);
    bool commandTorqueNm(double torque_nm);

    bool readPositionRad(double& position_rad);
    bool readVelocityRadSec(double& velocity_rad_sec);
    bool readTorqueNm(double& torque_nm);
    bool readTempDegC(double& temp_deg_c);

  private:
    ActiveJointParams params_;
};
}
