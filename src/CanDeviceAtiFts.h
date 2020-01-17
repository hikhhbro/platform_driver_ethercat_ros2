#pragma once

#include <Eigen/Dense>
#include "CanDevice.h"

namespace platform_driver_ethercat
{

/**
 * Interface description for a drive type of class.
 */
class CanDeviceAtiFts : public CanDevice
{
  public:
    /**
     * The constructor
     */
    CanDeviceAtiFts(EthercatInterface& ethercat,
                    unsigned int slave_id,
                    std::string device_name);

    /**
     * The destructor
     */
    ~CanDeviceAtiFts();

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

    Eigen::Vector3d readForceN();
    Eigen::Vector3d readTorqueNm();

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

  private:
    enum DictionaryObject
    {
        TOOL_TRANSFORMATION = 0x2020,
        CALIBRATION = 0x2040,
        MONITOR_CONDITION = 0x2060,
        DIAGNOSTIC_READINGS = 0x2080,
        VERSION = 0x2090,
        READING_DATA = 0x6000,
        STATUS_CODE = 0x6010,
        SAMPLE_COUNTER = 0x6020,
        CONTROL_CODES = 0x7010,
    };

    typedef struct TxPdo
    {
        int32_t fx;
        int32_t fy;
        int32_t fz;
        int32_t tx;
        int32_t ty;
        int32_t tz;
        uint32_t status_code;
        uint32_t sample_count;
    } TxPdo;

    typedef struct RxPdo
    {
        uint32_t control_1;
        uint32_t control_2;
    } RxPdo;

    TxPdo* input_;
    RxPdo* output_;

    int counts_per_force_;
    int counts_per_torque_;

    Eigen::Vector3d force_bias_;
    Eigen::Vector3d torque_bias_;
};
}
