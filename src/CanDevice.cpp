#include <iostream>

#include "CanDevice.h"
#include "base-logging/Logging.hpp"

using namespace platform_driver_ethercat;

CanDevice::CanDevice(EthercatInterface& ethercat, unsigned int slave_id, std::string device_name)
    : ethercat_(ethercat), slave_id_(slave_id), device_name_(device_name)
{
}

CanDevice::~CanDevice() {}

unsigned int CanDevice::getSlaveId() { return slave_id_; }

std::string CanDevice::getDeviceName() { return device_name_; }
