#pragma once

#include <map>
#include <memory>
#include <string>
#include <thread>

namespace platform_driver_ethercat
{

class CanDevice;

class EthercatInterface
{
  public:
    EthercatInterface(const std::string interface_address, const unsigned int num_slaves);
    ~EthercatInterface();
    bool init();
    void close();
    bool isInit();
    bool addDevice(std::shared_ptr<CanDevice> device);
    unsigned char* getInputPdoPtr(uint16_t slave);
    unsigned char* getOutputPdoPtr(uint16_t slave);

    static bool sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int* data);
    static bool sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data);

  private:
    const std::string interface_address_;
    const unsigned int num_slaves_;
    char io_map_[4096];
    bool is_initialized_;
    std::map<unsigned int, std::shared_ptr<CanDevice>> devices_;
    std::thread ethercat_thread_;

    static int expected_wkc_;
    static volatile int wkc_;

    void pdoCycle();
};
}
