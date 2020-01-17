//#include <chrono>

#include "CanDevice.h"
#include "EthercatInterface.h"
#include "base-logging/Logging.hpp"
#include "ethercat.h"

using namespace platform_driver_ethercat;

const int EC_TIMEOUTMON = 500;

int EthercatInterface::expected_wkc_ = 0;
volatile int EthercatInterface::wkc_ = 0;

EthercatInterface::EthercatInterface(const std::string interface_address,
                                     const unsigned int num_slaves)
    : interface_address_(interface_address), num_slaves_(num_slaves), is_initialized_(false)
{
}

EthercatInterface::~EthercatInterface() { close(); }

bool EthercatInterface::init()
{
    if (isInit()) return true;

    is_initialized_ = false;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(interface_address_.c_str()))
    {
        LOG_INFO_S << __PRETTY_FUNCTION__ << ": Initialization on interface "
                   << interface_address_.c_str() << " succeeded";
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            LOG_INFO_S << __PRETTY_FUNCTION__ << ": " << ec_slavecount << " slaves found";

            if (num_slaves_ != ec_slavecount)
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Expected number of slaves (" << num_slaves_
                            << ") differs from number of slaves found (" << ec_slavecount << ")";
                return false;
            }

            if (devices_.size() > ec_slavecount)
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Number of added devices ("
                            << devices_.size() << ") is greater than number of slaves found ("
                            << ec_slavecount << ")";
                return false;
            }

            /* configure all devices via sdo */
            for (auto& device : devices_)
            {
                unsigned int slave_id = device.first;

                if (slave_id > ec_slavecount)
                {
                    LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Slave id " << slave_id
                                << " outside range";
                    return false;
                }

                device.second->configure();
            }

            // Disable complete access
            // Workaround for bug of FT sensors according to
            // https://github.com/OpenEtherCATsociety/SOEM/issues/251
            for (int i = 1; i <= ec_slavecount; i++)
            {
                ec_slave[i].CoEdetails &= ~ECT_COEDET_SDOCA;
            }

            ec_config_map(&io_map_);
            ec_configdc();

            /* set pointers to pdo map for all devices */
            for (auto& device : devices_)
            {
                unsigned int slave_id = device.first;

                device.second->setInputPdo(ec_slave[slave_id].inputs);
                device.second->setOutputPdo(ec_slave[slave_id].outputs);
            }

            LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Slaves mapped, state to SAFE_OP";
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Calculated workcounter " << expected_wkc_;

            LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Request operational state for all slaves";
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            int chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Operational state reached for all slaves";

                /* create thread for pdo cycle */
                // pthread_create(&_thread_handle, NULL, &pdoCycle, NULL);
                ethercat_thread_ = std::thread(&EthercatInterface::pdoCycle, this);

                is_initialized_ = true;
                return true;
            }
            else
            {
                LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Not all slaves reached operational state";
                is_initialized_ = false;

                ec_readstate();

                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        LOG_DEBUG("%s: Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                  __PRETTY_FUNCTION__,
                                  i,
                                  ec_slave[i].state,
                                  ec_slave[i].ALstatuscode,
                                  ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }

                close();
                return false;
            }
        }
        else
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": No slaves found";
            close();
            return false;
        }
    }
    else
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Initialization on interface "
                    << interface_address_.c_str() << " not succeeded";
        return false;
    }
}

void EthercatInterface::close()
{
    if (isInit())
    {
        LOG_INFO_S << __PRETTY_FUNCTION__ << ": Request init state for all slaves";
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);

        // cancel pdo_cycle thread
        // pthread_cancel(_thread_handle);
        // pthread_join(_thread_handle, NULL);

        is_initialized_ = false;
    }

    LOG_INFO_S << __PRETTY_FUNCTION__ << ": Close socket";
    ec_close();
}

bool EthercatInterface::isInit() { return is_initialized_; }

bool EthercatInterface::addDevice(std::shared_ptr<CanDevice> device)
{
    if (isInit())
    {
        LOG_WARN_S << __PRETTY_FUNCTION__
                   << ": EtherCAT interface already initialized, device cannot be added afterwards";
        return false;
    }

    devices_.insert(std::make_pair(device->getSlaveId(), device));

    return true;
}

bool EthercatInterface::sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int* data)
{
    int fieldsize = sizeof(data);

    int wkc = ec_SDOread(slave, idx, sub, FALSE, &fieldsize, data, EC_TIMEOUTTXM);
    LOG_DEBUG("%s: Read from slave %d at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)",
              __PRETTY_FUNCTION__,
              slave,
              idx,
              sub,
              wkc,
              2 * fieldsize,
              data,
              data);

    if (wkc == 1)
        return true;
    else
        return false;
}

bool EthercatInterface::sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data)
{
    int wkc = ec_SDOwrite(slave, idx, sub, FALSE, fieldsize, &data, EC_TIMEOUTRXM);
    LOG_DEBUG("%s: Write to slave %d at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)",
              __PRETTY_FUNCTION__,
              slave,
              idx,
              sub,
              wkc,
              3 * fieldsize,
              data,
              data);

    if (wkc == 1)
        return true;
    else
        return false;
}

unsigned char* EthercatInterface::getInputPdoPtr(uint16_t slave) { return ec_slave[slave].inputs; }

unsigned char* EthercatInterface::getOutputPdoPtr(uint16_t slave)
{
    return ec_slave[slave].outputs;
}

void EthercatInterface::pdoCycle()
{
    int currentgroup = 0;

    while (1)
    {
        ec_send_processdata();
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

        while (EcatError) printf("%s", ec_elist2string());

        if ((wkc_ < expected_wkc_) || ec_group[currentgroup].docheckstate)
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup)
                    && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Slave " << slave
                                    << " is in SAFE_OP + ERROR, attempting ack";
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        LOG_WARN_S << __PRETTY_FUNCTION__ << ": Slave " << slave
                                   << " is in SAFE_OP, change to OPERATIONAL";
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        // devices_.at(slave)->configure();

                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Slave " << slave
                                        << " reconfigured";
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Slave " << slave << " lost";
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Slave " << slave
                                        << " recovered";
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Slave " << slave << " found";
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                LOG_INFO_S << __PRETTY_FUNCTION__ << ": All slaves resumed OPERATIONAL";
        }

        //osal_usleep(1000);  // roughly 1000 Hz
        osal_usleep(5000);  // roughly 200 Hz
        //osal_usleep(10000);  // roughly 100 Hz
        //LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); 
    }
}
