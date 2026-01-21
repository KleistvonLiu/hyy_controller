#ifndef __dh_modbus_rtu__
#define __dh_modbus_rtu__

#include <cstdint>

#include "dh_transport.h"

class ModbusRtuMaster
{
public:
    ModbusRtuMaster();
    explicit ModbusRtuMaster(ITransport* transport);

    void SetTransport(ITransport* transport);

    bool WriteSingleRegister(uint8_t slave_id, uint16_t reg, uint16_t value);
    bool ReadHoldingRegister(uint8_t slave_id, uint16_t reg, uint16_t &value);

private:
    static uint16_t CRC16(const uint8_t* nData, uint16_t wLength);
    bool WriteRegisterFunc(uint8_t slave_id, uint16_t reg, uint16_t value, int retry);
    bool ReadRegisterFunc(uint8_t slave_id, uint16_t reg, uint16_t &value, int retry);

    ITransport* transport_;
};

#endif //__dh_modbus_rtu__
