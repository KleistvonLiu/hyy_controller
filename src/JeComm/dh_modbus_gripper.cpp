#include "dh_modbus_gripper.h"

DH_Modbus_Gripper::DH_Modbus_Gripper(int id, std::string Portname, int Baudrate)
    :DH_Gripper(), _gripper_id(id),_PortName(Portname),_BaudRate(Baudrate),_transport(nullptr),_modbus()
{
    this->gripper_axis = 1;
}

DH_Modbus_Gripper::~DH_Modbus_Gripper()
{

}

int DH_Modbus_Gripper::open()
{
    if (_PortName.find(":") != _PortName.npos)
        _transport = std::unique_ptr<ITransport>(new TcpTransport(_PortName));
    else
        _transport = std::unique_ptr<ITransport>(new SerialTransport(_PortName, _BaudRate));

    if(!_transport->open())
    {
        std::cout << "DH_Modbus_Gripper open failed"<<std::endl;
        _modbus.SetTransport(nullptr);
        _transport.reset();
        return -1;
    }
    else
    {
        std::cout << "DH_Modbus_Gripper open successful"<<std::endl;
        _modbus.SetTransport(_transport.get());
        return _transport->native_handle();
    }
}

void DH_Modbus_Gripper::close()
{
    if (_transport)
    {
        _transport->close();
        _modbus.SetTransport(nullptr);
        _transport.reset();
    }
}


bool DH_Modbus_Gripper::Initialization()
{
   return WriteRegisterFunc(0x0100,0xA5);
}


bool DH_Modbus_Gripper::SetTargetPosition(int refpos)
{
    return WriteRegisterFunc(0x0103,refpos);
}

bool DH_Modbus_Gripper::SetTargetForce(int force)
{
    return WriteRegisterFunc(0x0101,force);
}

bool DH_Modbus_Gripper::SetTargetSpeed(int speed)
{
    return WriteRegisterFunc(0x0104,speed);
}


bool DH_Modbus_Gripper::GetCurrentPosition(int &curpos)
{
    return ReadRegisterFunc(0x0202,curpos);
}

bool DH_Modbus_Gripper::GetTargetPosition(int &tarpos)
{
    return ReadRegisterFunc(0x0103,tarpos);
}


bool DH_Modbus_Gripper::GetTargetForce(int &curTarforce)
{
    return ReadRegisterFunc(0x0101,curTarforce);
}

bool DH_Modbus_Gripper::GetTargetSpeed(int &curTarpos)
{
    return ReadRegisterFunc(0x0104,curTarpos);
}


bool DH_Modbus_Gripper::GetInitState(int &i_state)
{
     return ReadRegisterFunc(0x0200,i_state);
}

bool DH_Modbus_Gripper::GetGripState(int &g_state)
{
     return ReadRegisterFunc(0x0201,g_state);
}

bool DH_Modbus_Gripper::GetRunStates(int states[])
{
    if(this->GetInitState(states[0])) 
        if(this->GetGripState(states[1])) 
            if(this->GetCurrentPosition(states[2]))
                if(this->GetTargetPosition(states[3]))
                     if(this->GetTargetForce(states[4]))
                    {
                        return true;
                    }
    return false;
}

bool DH_Modbus_Gripper::WriteRegisterFunc(int index, int value)
{
    return _modbus.WriteSingleRegister(static_cast<uint8_t>(_gripper_id),
                                       static_cast<uint16_t>(index),
                                       static_cast<uint16_t>(value));
}


bool DH_Modbus_Gripper::ReadRegisterFunc(int index,int &value)
{
    uint16_t tmp = 0;
    bool ok = _modbus.ReadHoldingRegister(static_cast<uint8_t>(_gripper_id),
                                          static_cast<uint16_t>(index),
                                          tmp);
    if (ok)
        value = static_cast<int>(tmp);
    return ok;
}
