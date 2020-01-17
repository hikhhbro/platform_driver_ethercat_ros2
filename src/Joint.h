#pragma once

#include <memory>
#include <string>

namespace platform_driver_ethercat
{
class CanDriveTwitter;

class Joint
{
  public:
    Joint(std::string name, std::shared_ptr<CanDriveTwitter>& drive, bool enabled)
        : name_(name), drive_(drive), enabled_(enabled){};

    virtual ~Joint(){};

    virtual bool commandPositionRad(double position_rad) = 0;
    virtual bool commandVelocityRadSec(double velocity_rad_sec) = 0;
    virtual bool commandTorqueNm(double torque_nm) = 0;

    virtual bool readPositionRad(double& position_rad) = 0;
    virtual bool readVelocityRadSec(double& velocity_rad_sec) = 0;
    virtual bool readTorqueNm(double& torque_nm) = 0;
    virtual bool readTempDegC(double& temp_deg_c) = 0;

    std::string getName() { return name_; };
    std::shared_ptr<CanDriveTwitter> getDrive() { return drive_; };
    bool isEnabled() { return enabled_; };

  protected:
    std::string name_;
    std::shared_ptr<CanDriveTwitter> drive_;
    bool enabled_;
};
}
