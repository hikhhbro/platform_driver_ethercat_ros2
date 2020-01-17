#pragma once

#include <string>

namespace platform_driver_ethercat
{

class EthercatInterface;

class CanDevice
{
  public:
    CanDevice(EthercatInterface& ethercat, unsigned int slave_id, std::string device_name);
    virtual ~CanDevice() = 0;

    virtual bool configure() = 0;
    virtual bool startup() = 0;
    virtual bool shutdown() = 0;
    virtual bool reset() = 0;
    virtual bool isError() = 0;

    virtual void setInputPdo(unsigned char* input_pdo) = 0;
    virtual void setOutputPdo(unsigned char* output_pdo) = 0;

    unsigned int getSlaveId();
    std::string getDeviceName();

  protected:
    EthercatInterface& ethercat_;
    unsigned int slave_id_;
    std::string device_name_;
};
}
