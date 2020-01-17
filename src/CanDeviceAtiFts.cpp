#include <iostream>

#include "CanDeviceAtiFts.h"
#include "EthercatInterface.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDeviceAtiFts::CanDeviceAtiFts(EthercatInterface& ethercat,
                                 unsigned int slave_id,
                                 std::string device_name)
    : CanDevice(ethercat, slave_id, device_name),
      input_(NULL),
      output_(NULL),
      counts_per_force_(1),
      counts_per_torque_(1),
      force_bias_(0, 0, 0),
      torque_bias_(0, 0, 0)

{
    if (device_name == "FTS_FL")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_FR")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_CL")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_CR")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_BL")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
    else if (device_name == "FTS_BR")
    {
        force_bias_ = Eigen::Vector3d(0, 0, 0);
        torque_bias_ = Eigen::Vector3d(0, 0, 0);
    }
}

CanDeviceAtiFts::~CanDeviceAtiFts() {}

bool CanDeviceAtiFts::configure()
{
    LOG_DEBUG_S << "Configuring device " << device_name_ << " ...";

    typedef struct SdoWrite
    {
        uint16_t index;
        uint8_t subindex;
        uint8_t fieldsize;
        int32_t data;
    } SdoWrite;

    std::vector<SdoWrite> sdo_writes;

    // sdo_writes.push_back(SdoWrite{0x1c12, 0, 1, 0x00});

    bool success = true;

    for (auto sdo_write : sdo_writes)
    {
        success &= ethercat_.sdoWrite(
            slave_id_, sdo_write.index, sdo_write.subindex, sdo_write.fieldsize, sdo_write.data);
    }

    int force_unit;
    int torque_unit;

    success &= ethercat_.sdoRead(slave_id_, DictionaryObject::CALIBRATION, 0x29, &force_unit);
    success &= ethercat_.sdoRead(slave_id_, DictionaryObject::CALIBRATION, 0x2a, &torque_unit);

    LOG_DEBUG_S << "Force unit of sensor " << device_name_ << " is " << force_unit;
    LOG_DEBUG_S << "Torque unit of sensor " << device_name_ << " is " << torque_unit;

    success &=
        ethercat_.sdoRead(slave_id_, DictionaryObject::CALIBRATION, 0x31, &counts_per_force_);
    success &=
        ethercat_.sdoRead(slave_id_, DictionaryObject::CALIBRATION, 0x32, &counts_per_torque_);

    if (success)
    {
        LOG_INFO_S << "Device " << device_name_ << " configured";
        return true;
    }
    else
    {
        LOG_ERROR_S << "Failed to configure device " << device_name_;
        return false;
    }
}

void CanDeviceAtiFts::setInputPdo(unsigned char* input_pdo) { input_ = (TxPdo*)input_pdo; }

void CanDeviceAtiFts::setOutputPdo(unsigned char* output_pdo)
{
    output_ = (RxPdo*)output_pdo;

    output_->control_1 = 0x00000000;
    output_->control_2 = 0x00000000;
}

bool CanDeviceAtiFts::startup() { return true; }

bool CanDeviceAtiFts::shutdown() { return true; }

bool CanDeviceAtiFts::reset() { return shutdown() && startup(); }

Eigen::Vector3d CanDeviceAtiFts::readForceN()
{
    double fx = input_->fx * 1.0 / counts_per_force_;
    double fy = input_->fy * 1.0 / counts_per_force_;
    double fz = input_->fz * 1.0 / counts_per_force_;

    Eigen::Vector3d force = Eigen::Vector3d(fx, fy, fz);
    force -= force_bias_;

    return force;
}

Eigen::Vector3d CanDeviceAtiFts::readTorqueNm()
{
    double tx = input_->tx * 1.0 / counts_per_torque_;
    double ty = input_->ty * 1.0 / counts_per_torque_;
    double tz = input_->tz * 1.0 / counts_per_torque_;

    Eigen::Vector3d torque = Eigen::Vector3d(tx, ty, tz);
    torque -= torque_bias_;

    return torque;
}

bool CanDeviceAtiFts::isError() { return false; }

unsigned int CanDeviceAtiFts::getError() { return 0; }
