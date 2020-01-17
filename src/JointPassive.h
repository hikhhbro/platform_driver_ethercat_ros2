#pragma once

#include "Joint.h"

namespace platform_driver_ethercat
{
class JointPassive : public Joint
{
  public:
    JointPassive(std::string name, std::shared_ptr<CanDriveTwitter>& drive, bool enabled);

    bool commandPositionRad(double position_rad);
    bool commandVelocityRadSec(double velocity_rad_sec);
    bool commandTorqueNm(double torque_nm);

    bool readPositionRad(double& position_rad);
    bool readVelocityRadSec(double& velocity_rad_sec);
    bool readTorqueNm(double& torque_nm);
    bool readTempDegC(double& temp_deg_c);
};
}
