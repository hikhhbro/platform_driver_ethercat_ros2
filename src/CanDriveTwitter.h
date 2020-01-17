#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "CanDevice.h"
#include "PlatformDriverEthercatTypes.h"

namespace platform_driver_ethercat
{

/**
 * Interface description for a drive type of class.
 */
class CanDriveTwitter : public CanDevice
{
  public:
    /**
     * The constructor
     */
    CanDriveTwitter(EthercatInterface& ethercat,
                    unsigned int slave_id,
                    std::string name,
                    DriveParams params);

    /**
     * The destructor
     */
    ~CanDriveTwitter();

    bool configure();
    void setInputPdo(unsigned char* input_pdo);
    void setOutputPdo(unsigned char* output_pdo);

    /**
     * Brings the drive to operation enable state.
     * After calling the drive accepts velocity and position commands.
     * @return True if drive is started successfully. StatusRegister is also evaluated to ensure a
     * non-faulty state. False otherwise.
     */
    bool startup();

    /**
     * Brings the drive to switch on disabled state.
     * After calling the drive won't accepts velocity and position commands.
     * @return True if drive shutdown successful.
     */
    bool shutdown();

    /**
     * Resets the drive.
     * @return True if re-initialization was successful. False otherwise.
     */
    bool reset();

    /**
     * Sends position command
     * @param position_rad Position command in Radians
     */
    void commandPositionRad(double position_rad);

    /**
     * Sends velocity command.
     * Use this function only in velocity control mode.
     * @param velocity_rad_sec Velocity command in Radians/sec.
     */
    void commandVelocityRadSec(double velocity_rad_sec);

    /**
     * Sends Torque command
     * Use this function only in torque control mode.
     * @param torque_nm is the required motor torque in Nm.
     */
    void commandTorqueNm(double torque_nm);

    /**
     * Reads the last received value of the drive position.
     * @return The value of the current position of the motor.
     */
    double readPositionRad();

    /**
     * Reads the last received value of the drive Velocity.
     * @return The value of the current Velocity of the motor.
     */
    double readVelocityRadSec();

    /**
     * Reads the last received value of the motor Torque.
     * @return The value (in Nm) of the current motor torque is stored in this pointer.
     */
    double readTorqueNm();

    /**
     * Returns received value from analog input.
     */
    double readAnalogInputV();

    /**
     * Returns true if an error has been detected.
     * @return boolean with result.
     */
    bool isError();

    /**
     * Returns a bitfield containing information about the current error.
     * @return unsigned int with bitcoded error.
     */
    unsigned int getError();

    /**
     * Enable the emergency stop.
     * @return true if the result of the process is successful
     */
    bool requestEmergencyStop();

  private:
    enum class DriveObject
    {
        // Error control objects
        ABORT_CONNECTION_OPTION_CODE = 0x6007,
        ERROR_CODE = 0x603f,

        // Device Control Objects
        CONTROL_WORD = 0x6040,
        STATUS_WORD = 0x6041,

        // Halt, stop and fault objects
        QUICK_STOP_OPTION_CODE = 0x605a,
        SHUTDOWN_OPTION_CODE = 0x605b,
        DISABLE_OPERATION_OPTION_CODE = 0x605c,
        HALT_OPTION_CODE = 0x605d,
        FAULT_REACTION_OPTION_CODE = 0x605e,

        // Modes of operation
        MODES_OF_OPERATION = 0x6060,
        MODES_OF_OPERATION_DISPLAY = 0x6061,
        SUPPORTED_DRIVE_MODES = 0x6502,

        // Position control
        POSITION_DEMAND_VALUE = 0x6062,
        POSITION_ACTUAL_INTERNAL_VALUE = 0x6063,
        POSITION_ACTUAL_VALUE = 0x6064,
        FOLLOWING_ERROR_WINDOW = 0x6065,
        FOLLOWING_ERROR_TIMEOUT = 0x6066,
        POSITION_WINDOW = 0x6067,
        POSITION_WINDOW_TIME = 0x6068,
        TARGET_POSITION = 0x607a,
        POSITION_RANGE_LIMIT = 0x607b,
        SOFTWARE_POSITION_LIMIT = 0x607d,
        MAX_PROFILE_VELOCITY = 0x607f,
        MAX_MOTOR_SPEED = 0x6080,
        PROFILE_VELOCITY = 0x6081,
        END_VELOCITY = 0x6082,
        PROFILE_ACCELERATION = 0x6083,
        PROFILE_DECELRATION = 0x6084,
        QUICK_STOP_DECELERATION = 0x6085,
        MOTION_PROFILE_TYPE = 0x6086,
        MAX_ACCELERATION = 0x60c5,
        MAX_DECELERATION = 0x60c6,
        POSITION_OPTION_CODE = 0x60f2,
        FOLLOWING_ERROR_ACTUAL_VALUE = 0x60f4,
        CONTROL_EFFORT = 0x60fa,
        POSITION_DEMAND_INTERNAL_VALUE_INCREMENTS = 0x60fc,

        // Velocity control
        VELOCITY_SENSOR_ACTUAL_VALUE = 0x6069,
        SENSOR_SELECTION_CODE = 0x606a,
        VELOCITY_DEMAND_VALUE = 0x606b,
        VELOCITY_ACTUAL_VALUE = 0x606c,
        VELOCITY_WINDOW = 0x606d,
        VELOCITY_WINDOW_TIME = 0x606e,
        VELOCITY_THRESHOLD = 0x606f,
        VELOCITY_THRESHOLD_TIME = 0x6070,
        TARGET_VELOCITY = 0x60ff,

        // Torque control
        TARGET_TORQUE = 0x6071,
        MAX_TORQUE = 0x6072,
        MAX_CURRENT = 0x6073,
        TORQUE_DEMAND_VALUE = 0x6074,
        MOTOR_RATE_CURRENT = 0x6075,
        MOTOR_RATE_TORQUE = 0x6076,
        TORQUE_ACTUAL_VALUE = 0x6077,
        CURRENT_ACTUAL_VALUE = 0x6078,
        DC_LINK_CIRCUIT_VOLTAGE = 0x6079,
        TORQUE_SLOPE = 0x6087,
        POSITIVE_TORQUE_LIMIT_VALUE = 0x60e0,
        NEGATIVE_TORQUE_LIMIT_VALUE = 0x60e1,

        // Factors
        POLARITY = 0x607e,
        POSITION_NOTATION_INDEX = 0x6089,
        POSITION_DIMENSION_INDEX = 0x608a,
        VELOCITY_NOTATION_INDEX = 0x608b,
        VELOCITY_DIMENSION_INDEX = 0x608c,
        ACCELERATION_NOTATION_INDEX = 0x608d,
        ACCELERATION_DIMENSION_INDEX = 0x608e,
        POSITION_ENCODER_RESOLUTION = 0x608f,
        VELOCITY_ENCODER_RESOLUTION = 0x6090,
        GEAR_RATIO = 0x6091,
        FEED_CONSTANT = 0x6092,
        POSITION_FACTOR = 0x6093,
        VELOCITY_ENCODER_FACTOR = 0x6094,
        VELOCITY_FACTOR_1 = 0x6095,
        VELOCITY_FACTOR = 0x6096,
        ACCELERATION_FACTOR = 0x6097,

        // Cyclic synchronous modes
        POSITION_OFFSET = 0x60b0,
        VELOCITY_OFFSET = 0x60b1,
        TORQUE_OFFSET = 0x60b2,

        // Drive data objects
        ANALOG_INPUT = 0x2205,
        DIGITAL_INPUTS = 0x60fd,
        DIGITAL_OUTPUTS = 0x60fe,
    };

    /**
     * States of the CANOpen drive state machine.
     */
    enum DriveState
    {
        ST_NOT_READY_TO_SWITCH_ON,
        ST_SWITCH_ON_DISABLED,
        ST_READY_TO_SWITCH_ON,
        ST_SWITCHED_ON,
        ST_OPERATION_ENABLE,
        ST_QUICK_STOP_ACTIVE,
        ST_FAULT_REACTION_ACTIVE,
        ST_FAULT,
        ST_UNKNOWN
    };

    /**
     * Enum with different operation modes of the controller, either position, velocity or torque
     * control.
     */
    enum OperationMode
    {
        OM_PROFILE_POSITION = 1,
        OM_PROFILE_VELOCITY = 3,
        OM_PROFILE_TORQUE = 4,
        OM_CYCSYNC_POSITION = 8,
        OM_CYCSYNC_VELOCITY = 9,
        OM_CYCSYNC_TORQUE = 10
    };

    typedef struct RxPdo
    {
        uint16_t control_word;
        uint16_t operation_mode;
        int32_t target_position;
        int32_t target_velocity;
        int16_t target_torque;
    } RxPdo;

    typedef struct TxPdo
    {
        uint16_t status_word;
        uint8_t operation_mode_display;
        int32_t actual_position;
        int32_t actual_velocity;
        int16_t actual_torque;
        int16_t analog_input;
    } TxPdo;

    DriveParams params_;

    TxPdo* input_;
    RxPdo* output_;

    std::thread command_thread_;
    std::mutex command_mutex_;
    std::condition_variable command_cv_;

    /**
     * Returns the state of the drive
     */
    DriveState readDriveState();

    OperationMode readOperationMode();
    bool commandOperationMode(OperationMode mode);

    /**
     * Checks if the target set point was already reached.
     * @return True if the target set point was already reached.
     */
    bool checkTargetReached();

    /**
     * Checks if the trajectory generator has assumed the new
     * positioning value.
     * @return True if the new set point was acknowledged.
     */
    bool checkSetPointAcknowledge();

    void commandSetPoint();
};
}
