#include <math.h>
#include <unistd.h>
#include <bitset>
#include <iostream>

#include "CanDriveTwitter.h"
#include "EthercatInterface.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDriveTwitter::CanDriveTwitter(EthercatInterface& ethercat,
                                 unsigned int slave_id,
                                 std::string name,
                                 DriveParams params)
    : CanDevice(ethercat, slave_id, name),
      params_(params),
      input_(NULL),
      output_(NULL),
      command_thread_(&CanDriveTwitter::commandSetPoint, this)
{
}

CanDriveTwitter::~CanDriveTwitter() {}

bool CanDriveTwitter::configure()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Configuring drive " << device_name_ << " ...";

    typedef struct SdoWrite
    {
        uint16_t index;
        uint8_t subindex;
        uint8_t fieldsize;
        int32_t data;
    } SdoWrite;

    std::vector<SdoWrite> sdo_writes;

    // set RxPDO map
    sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x00});    // disable
    sdo_writes.push_back(SdoWrite{0x1c12, 1, 2, 0x160a});  // control word
    sdo_writes.push_back(SdoWrite{0x1c12, 2, 2, 0x160b});  // mode of operation
    sdo_writes.push_back(SdoWrite{0x1c12, 3, 2, 0x160f});  // target position
    sdo_writes.push_back(SdoWrite{0x1c12, 4, 2, 0x161c});  // target velocity
    sdo_writes.push_back(SdoWrite{0x1c12, 5, 2, 0x160c});  // target torque
    sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x05});    // enable

    // set TxPDO map
    sdo_writes.push_back(SdoWrite{0x1c13, 0, 1, 0x00});    // disable
    sdo_writes.push_back(SdoWrite{0x1c13, 1, 2, 0x1a0a});  // status word
    sdo_writes.push_back(SdoWrite{0x1c13, 2, 2, 0x1a0b});  // mode of operation display
    sdo_writes.push_back(SdoWrite{0x1c13, 3, 2, 0x1a0e});  // actual position
    sdo_writes.push_back(SdoWrite{0x1c13, 4, 2, 0x1a11});  // actual velocity
    sdo_writes.push_back(SdoWrite{0x1c13, 5, 2, 0x1a13});  // actual torque
    sdo_writes.push_back(SdoWrite{0x1c13, 6, 2, 0x1a1d});  // analog input
    sdo_writes.push_back(SdoWrite{0x1c13, 0, 1, 0x06});    // enable

    // set commutation
    sdo_writes.push_back(SdoWrite{0x3034, 17, 4, 0x00000003});  // commutation method
    sdo_writes.push_back(
        SdoWrite{0x31d6, 1, 4, 0x41f00000});  // stepper commutation desired current

    double gear_ratio = params_.gear_ratio;
    unsigned int encoder_increments = params_.encoder_increments;
    bool encoder_on_output = params_.encoder_on_output;

    double max_current = (params_.max_motor_current_amp * 1000.0) / params_.motor_rated_current_amp;
    double rated_current = params_.motor_rated_current_amp * 1000.0;
    double rated_torque = params_.motor_rated_torque_nm * 1000.0;
    double min_pos = params_.min_position_deg * encoder_increments / 360.0;
    double max_pos = params_.max_position_deg * encoder_increments / 360.0;
    double max_vel = params_.max_motor_speed_rpm * encoder_increments / 60.0;
    double profile_vel = params_.profile_velocity_rad_sec * encoder_increments / (2.0 * M_PI);
    double profile_acc =
        params_.profile_acceleration_rad_sec_sec * encoder_increments / (2.0 * M_PI);

    if (encoder_on_output)
    {
        max_vel /= gear_ratio;
    }
    else
    {
        min_pos *= gear_ratio;
        max_pos *= gear_ratio;
        profile_vel *= gear_ratio;
        profile_acc *= gear_ratio;
    }

    sdo_writes.push_back(
        SdoWrite{0x6073, 0, 2, (int)max_current});  // max current (thousands of rated current)
    sdo_writes.push_back(SdoWrite{0x6075, 0, 4, (int)rated_current});  // motor rated current (mA)
    sdo_writes.push_back(SdoWrite{0x6076, 0, 4, (int)rated_torque});   // motor rated torque (mNm)
    sdo_writes.push_back(SdoWrite{0x607b, 1, 4, (int)min_pos});  // min position range limit (inc)
    sdo_writes.push_back(SdoWrite{0x607b, 2, 4, (int)max_pos});  // max position range limit (inc)
    sdo_writes.push_back(SdoWrite{0x607d, 1, 4, (int)min_pos});  // min position limit (inc)
    sdo_writes.push_back(SdoWrite{0x607d, 2, 4, (int)max_pos});  // max position limit (inc)
    sdo_writes.push_back(SdoWrite{0x607f, 0, 4, (int)max_vel});  // max profile velocity
    sdo_writes.push_back(SdoWrite{0x6080, 0, 4, (int)max_vel});  // max motor speed

    // set profile motion parameters
    sdo_writes.push_back(SdoWrite{0x6081, 0, 4, (int)profile_vel});  // profile velocity
    sdo_writes.push_back(SdoWrite{0x6083, 0, 4, (int)profile_acc});  // profile acceleration
    sdo_writes.push_back(SdoWrite{0x6084, 0, 4, (int)profile_acc});  // profile deceleration

    // factors (set to 2 to convert units on software side instead of Elmo conversion to avoid
    // decrease in resolution)
    sdo_writes.push_back(
        SdoWrite{0x608f, 1, 4, 0x00000001});  // position encoder resolution (encoder increments)
    sdo_writes.push_back(
        SdoWrite{0x608f, 2, 4, 0x00000001});  // position encoder resolution (motor increments)
    sdo_writes.push_back(
        SdoWrite{0x6090, 1, 4, 0x00000001});  // velocity encoder resolution (encoder increments)
    sdo_writes.push_back(
        SdoWrite{0x6090, 2, 4, 0x00000001});  // velocity encoder resolution (motor increments)
    sdo_writes.push_back(
        SdoWrite{0x6091, 1, 4, 0x00000001});  // gear ratio (motor shaft revolutions)
    sdo_writes.push_back(
        SdoWrite{0x6091, 2, 4, 0x00000001});  // gear ratio (driving shaft revolutions)
    sdo_writes.push_back(SdoWrite{0x6092, 1, 4, 0x00000001});  // feed constant (feed)
    sdo_writes.push_back(
        SdoWrite{0x6092, 2, 4, 0x00000001});  // feed constant (driving shaft revolutions)
    sdo_writes.push_back(SdoWrite{0x6096, 1, 4, 0x00000001});  // velocity factor (numerator)
    sdo_writes.push_back(SdoWrite{0x6096, 2, 4, 0x00000001});  // velocity factor (divisor)
    sdo_writes.push_back(SdoWrite{0x6097, 1, 4, 0x00000001});  // acceleration factor (numerator)
    sdo_writes.push_back(SdoWrite{0x6097, 2, 4, 0x00000001});  // acceleration factor (divisor)

    bool success = true;

    for (auto sdo_write : sdo_writes)
    {
        success &= ethercat_.sdoWrite(
            slave_id_, sdo_write.index, sdo_write.subindex, sdo_write.fieldsize, sdo_write.data);
    }

    if (success)
    {
        LOG_INFO_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_ << " configured";
        return true;
    }
    else
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Failed to configure drive " << device_name_;
        return false;
    }
}

void CanDriveTwitter::setInputPdo(unsigned char* input_pdo) { input_ = (TxPdo*)input_pdo; }

void CanDriveTwitter::setOutputPdo(unsigned char* output_pdo)
{
    output_ = (RxPdo*)output_pdo;

    output_->control_word = 0x0004;  // disable quick stop & disable voltage
    output_->operation_mode = 0;
    output_->target_position = 0;
    output_->target_velocity = 0;
    output_->target_torque = 0;
}

bool CanDriveTwitter::startup()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Starting up drive " << device_name_ << " ...";

    DriveState state = readDriveState();
    int cnt = 10000;

    while (state != ST_OPERATION_ENABLE)
    {
        switch (state)
        {
            case ST_FAULT:
                output_->control_word = 0x0080;  // fault reset
                break;
            case ST_QUICK_STOP_ACTIVE:
                output_->control_word = 0x0004;  // disable quick stop
                break;
            case ST_SWITCH_ON_DISABLED:
                output_->control_word = 0x0006;  // enable voltage
                break;
            case ST_READY_TO_SWITCH_ON:
                output_->control_word = 0x0007;  // switch on
                break;
            case ST_SWITCHED_ON:
                output_->control_word = 0x000f;  // enable operation
                break;
            default: break;
        }

        usleep(1000);  // sleep 0.001 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not start up drive " << device_name_
                        << ". Last state was " << state;
            return false;
        }
    }

    LOG_INFO_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_ << " started up";

    return true;
}

bool CanDriveTwitter::shutdown()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Shutting down drive " << device_name_ << " ...";

    DriveState state = readDriveState();
    int cnt = 1000;

    while (state != ST_SWITCH_ON_DISABLED)
    {
        switch (state)
        {
            case ST_OPERATION_ENABLE:
                output_->control_word = 0x0007;  // disable operation
                break;
            case ST_SWITCHED_ON:
                output_->control_word = 0x0006;  // switch off
                break;
            case ST_READY_TO_SWITCH_ON:
                output_->control_word = 0x0004;  // disable voltage
                break;
            case ST_FAULT:
                output_->control_word = 0x0080;  // fault reset
                break;
            case ST_QUICK_STOP_ACTIVE:
                output_->control_word = 0x0004;  // disable quick stop & disable voltage
                break;
            default: break;
        }

        usleep(1000);  // sleep 0.001 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not shut down drive " << device_name_
                        << ". Last state was " << state;
            return false;
        }
    }

    LOG_INFO_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_ << " shut down";

    return true;
}

bool CanDriveTwitter::reset() { return shutdown() && startup(); }

CanDriveTwitter::OperationMode CanDriveTwitter::readOperationMode()
{
    return (OperationMode)input_->operation_mode_display;
}

bool CanDriveTwitter::commandOperationMode(CanDriveTwitter::OperationMode mode)
{
    OperationMode current_mode = readOperationMode();

    if (current_mode == mode)
    {
        return true;
    }

    output_->operation_mode = mode;

    int cnt = 100;

    while (current_mode != mode)
    {
        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not set operation mode for drive "
                        << device_name_ << ". Current mode is " << current_mode
                        << ". Requested mode is " << mode << ".";
            return false;
        }

        usleep(1000);  // sleep 0.001 s
        current_mode = readOperationMode();
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Successfully changed operation mode for drive "
                << device_name_ << " to " << current_mode;

    return true;
}

void CanDriveTwitter::commandSetPoint()
{
    std::unique_lock<std::mutex> lock(command_mutex_);

    while (1)
    {
        command_cv_.wait(lock);

        int cnt = 100;

        while (checkSetPointAcknowledge())
        {
            if (cnt-- == 0)
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_
                            << " not ready for new set point";
                break;
            }

            usleep(1000);  // sleep 0.001 s
        }

        output_->control_word |= 0x0030;  // new set point & change set point immediately

        cnt = 100;

        while (!checkSetPointAcknowledge())
        {
            //LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_ << " Position "
            //           << position_inc << " Time "
            //           << std::chrono::duration_cast<std::chrono::milliseconds>(
            //                  std::chrono::system_clock::now().time_since_epoch())
            //                  .count();

            if (cnt-- == 0)
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": New set point " << output_->target_position
                            << " was not acknowledged by drive " << device_name_;
                break;
            }

            usleep(1000);  // sleep 0.001 s
        }

        output_->control_word &= 0xffef;  // no new set point
    }
}

void CanDriveTwitter::commandPositionRad(double position_rad)
{
    std::unique_lock<std::mutex> lock(command_mutex_);

    double position_inc = position_rad * (params_.encoder_on_output ? 1.0 : params_.gear_ratio)
                          * params_.encoder_increments / (2.0 * M_PI);
    output_->target_position = position_inc;
    commandOperationMode(OM_PROFILE_POSITION);
    // commandOperationMode(OM_CYCSYNC_POSITION);

    command_cv_.notify_one();
}

void CanDriveTwitter::commandVelocityRadSec(double velocity_rad_sec)
{
    std::unique_lock<std::mutex> lock(command_mutex_);

    double velocity_inc = velocity_rad_sec * (params_.encoder_on_output ? 1.0 : params_.gear_ratio)
                          * params_.encoder_increments / (2.0 * M_PI);
    output_->target_velocity = velocity_inc;
    commandOperationMode(OM_PROFILE_VELOCITY);
}

void CanDriveTwitter::commandTorqueNm(double torque_nm)
{
    std::unique_lock<std::mutex> lock(command_mutex_);

    double input_torque_nm = torque_nm / params_.gear_ratio;
    output_->target_torque = input_torque_nm * 1000.0 / params_.motor_rated_torque_nm;
    commandOperationMode(OM_PROFILE_TORQUE);
}

bool CanDriveTwitter::checkTargetReached()
{
    unsigned char bit10 = (unsigned char)((input_->status_word >> 10) & 0x0001);

    return (bool)bit10;
}

bool CanDriveTwitter::checkSetPointAcknowledge()
{
    unsigned char bit12 = (unsigned char)((input_->status_word >> 12) & 0x0001);

    return (bool)bit12;
}

double CanDriveTwitter::readPositionRad()
{
    double position_inc = input_->actual_position;
    double position_rad =
        position_inc * 2.0 * M_PI
        / ((params_.encoder_on_output ? 1.0 : params_.gear_ratio) * params_.encoder_increments);

    return position_rad;
}

double CanDriveTwitter::readVelocityRadSec()
{
    double velocity_inc = input_->actual_velocity;
    double velocity_rad_sec =
        velocity_inc * 2.0 * M_PI
        / ((params_.encoder_on_output ? 1.0 : params_.gear_ratio) * params_.encoder_increments);

    return velocity_rad_sec;
}

double CanDriveTwitter::readTorqueNm()
{
    double input_torque_nm = input_->actual_torque * params_.motor_rated_torque_nm / 1000.0;
    double output_torque_nm = input_torque_nm * params_.gear_ratio;

    return output_torque_nm;
}

double CanDriveTwitter::readAnalogInputV() { return input_->analog_input * 1.0 / 1000.0; }

CanDriveTwitter::DriveState CanDriveTwitter::readDriveState()
{
    unsigned char status_lower = (unsigned char)input_->status_word;
    unsigned char bits0to3 = status_lower & 0x0f;
    unsigned char bit5 = (status_lower >> 5) & 0x01;
    unsigned char bit6 = (status_lower >> 6) & 0x01;

    switch (bits0to3)
    {
        case 0x0:
            if (!bit6)
                return ST_NOT_READY_TO_SWITCH_ON;
            else
                return ST_SWITCH_ON_DISABLED;
        case 0x1:
            if (bit5 && !bit6) return ST_READY_TO_SWITCH_ON;
        case 0x3:
            if (bit5 && !bit6) return ST_SWITCHED_ON;
        case 0x7:
            if (bit5 && !bit6)
                return ST_OPERATION_ENABLE;
            else if (!bit5 && !bit6)
                return ST_QUICK_STOP_ACTIVE;
        case 0x8:
            if (!bit6) return ST_FAULT;
        case 0xf:
            if (!bit6) return ST_FAULT_REACTION_ACTIVE;
    }

    LOG_WARN_S << __PRETTY_FUNCTION__ << ": Drive " << device_name_
               << " in unknown state! Lower byte of status word: " << status_lower;
    return ST_UNKNOWN;
}

bool CanDriveTwitter::isError()
{
    DriveState state = readDriveState();

    return (state == ST_FAULT_REACTION_ACTIVE) || (state == ST_FAULT);
}

unsigned int CanDriveTwitter::getError()
{
    unsigned char status_upper = (unsigned char)(input_->status_word >> 8);

    return status_upper;
}

bool CanDriveTwitter::requestEmergencyStop()
{
    uint16_t control_word = output_->control_word;

    // enable quick stop
    control_word &= 0b111111101111011;
    control_word |= 0b000000000000010;

    output_->control_word = control_word;

    int cnt = 1000;

    DriveState state;

    do
    {
        usleep(1000);  // sleep 0.001 s
        state = readDriveState();

        if (cnt-- == 0)
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not emergency stop drive "
                        << device_name_ << ". Last state was " << state;
            return false;
        }
    } while (state != ST_QUICK_STOP_ACTIVE);

    return true;
}
