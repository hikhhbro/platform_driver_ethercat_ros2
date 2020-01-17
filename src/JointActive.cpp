#include <algorithm>
#include <limits>

#include "CanDriveTwitter.h"
#include "JointActive.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

JointActive::JointActive(std::string name,
                         std::shared_ptr<CanDriveTwitter>& drive,
                         ActiveJointParams params,
                         bool enabled)
    : Joint(name, drive, enabled), params_(params){};

bool JointActive::commandPositionRad(double position_rad)
{
    if (!enabled_) return false;

    double position_old = position_rad;

    double min_pos = params_.min_position_command_rad;
    double max_pos = params_.max_position_command_rad;

    if (!(min_pos == 0.0 && max_pos == 0.0))
    {
        position_rad = std::max(min_pos, std::min(max_pos, position_rad));
    }

    if (position_rad != position_old)
    {
        LOG_WARN_S << __PRETTY_FUNCTION__ << ": Command exceeds position limit for joint " << name_;
    }

    if (params_.flip_sign)
    {
        position_rad *= -1.0;
    }

    drive_->commandPositionRad(position_rad);

    return true;
}

bool JointActive::commandVelocityRadSec(double velocity_rad_sec)
{
    if (!enabled_) return false;

    double velocity_old = velocity_rad_sec;
    double max_vel = params_.max_velocity_command_rad_sec;

    if (max_vel != 0.0)
    {
        velocity_rad_sec = std::min(max_vel, std::max(-max_vel, velocity_rad_sec));
    }

    if (velocity_rad_sec != velocity_old)
    {
        LOG_WARN_S << __PRETTY_FUNCTION__ << ": Command exceeds velocity limit for joint " << name_;
    }

    double current_pos;
    readPositionRad(current_pos);

    velocity_old = velocity_rad_sec;

    double min_pos = params_.min_position_command_rad;
    double max_pos = params_.max_position_command_rad;

    if (!(min_pos == 0.0 && max_pos == 0.0))
    {
        if (current_pos >= max_pos)
            velocity_rad_sec = std::min(0.0, velocity_rad_sec);
        else if (current_pos <= min_pos)
            velocity_rad_sec = std::max(0.0, velocity_rad_sec);
    }

    if (velocity_rad_sec != velocity_old)
    {
        LOG_WARN_S << __PRETTY_FUNCTION__ << ": Position limit reached for joint " << name_;
    }

    if (params_.flip_sign)
    {
        velocity_rad_sec *= -1.0;
    }

    drive_->commandVelocityRadSec(velocity_rad_sec);

    return true;
}

bool JointActive::commandTorqueNm(double torque_nm)
{
    if (!enabled_) return false;

    double torque_old = torque_nm;
    double max_torque = params_.max_torque_command_nm;

    if (max_torque != 0.0)
    {
        torque_nm = std::min(max_torque, std::max(-max_torque, torque_nm));
    }

    if (torque_nm != torque_old)
    {
        LOG_WARN_S << __PRETTY_FUNCTION__ << ": Command exceeds torque limit for joint " << name_;
    }

    if (params_.flip_sign)
    {
        torque_nm *= -1.0;
    }

    drive_->commandTorqueNm(torque_nm);

    return true;
}

bool JointActive::readPositionRad(double& position_rad)
{
    if (enabled_)
    {
        position_rad = drive_->readPositionRad();
        if (params_.flip_sign) position_rad *= -1.0;
        return true;
    }
    else
    {
        position_rad = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readVelocityRadSec(double& velocity_rad_sec)
{
    if (enabled_)
    {
        velocity_rad_sec = drive_->readVelocityRadSec();
        if (params_.flip_sign) velocity_rad_sec *= -1.0;
        return true;
    }
    else
    {
        velocity_rad_sec = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readTorqueNm(double& torque_nm)
{
    if (enabled_)
    {
        torque_nm = drive_->readTorqueNm();
        if (params_.flip_sign) torque_nm *= -1.0;
        return true;
    }
    else
    {
        torque_nm = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readTempDegC(double& temp_deg_c)
{
    double Vout = drive_->readAnalogInputV();

    const double alpha = 0.00385;
    const double V0 = 3.3;
    const double R0 = 100;
    const double R1 = 3000;
    const double R2 = 2000;
    const double R3 = 200;

    temp_deg_c =
        1.0 / alpha * (V0 - (R0 + R1) / R0 * R3 / (R2 + R3) * Vout) / (-V0 + R3 / (R2 + R3) * Vout)
        - params_.temp_offset_deg_c;

    return true;
}
