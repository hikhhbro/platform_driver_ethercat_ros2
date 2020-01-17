#pragma once

#include <memory>
#include <string>

namespace platform_driver_ethercat
{
struct DriveParams
{
    double min_position_deg;
    double max_position_deg;
    unsigned int max_motor_speed_rpm;
    double max_motor_current_amp;
    double motor_rated_current_amp;
    double motor_rated_torque_nm;
    double gear_ratio;
    unsigned int encoder_increments;
    bool encoder_on_output;
    double profile_velocity_rad_sec;
    double profile_acceleration_rad_sec_sec;
};

struct ActiveJointParams
{
    bool flip_sign;
    double min_position_command_rad;
    double max_position_command_rad;
    double max_velocity_command_rad_sec;
    double max_torque_command_nm;
    double temp_offset_deg_c;
};
}
