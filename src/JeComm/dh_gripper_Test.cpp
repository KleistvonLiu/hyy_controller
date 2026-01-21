#include "dh_modbus_gripper.h"

void ModbusGripper_test()
{
    DH_Modbus_Gripper m_gripper(1, "/dev/DH_hand", 115200);
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(initstate);
    if(initstate != DH_Modbus_Gripper::S_INIT_FINISHED)
    {
        m_gripper.Initialization();
        std::cout<< " Send grip init " << std::endl;

        //wait for gripper initialization
        initstate = 0;
        std::cout<< " Send grip GetInitState " << std::endl;
        while(initstate != DH_Modbus_Gripper::S_INIT_FINISHED )
            m_gripper.GetInitState(initstate); 
        std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    int currpos= 0;
    int g_state = 0;
    int loop = 100;
    while(loop--)
    {

    //set gripper target position 1000
        m_gripper.SetTargetPosition(1000);
        
    //wait gripper arrived the target postion
        g_state = 0;
        while(g_state == DH_Modbus_Gripper::S_GRIP_MOVING)
            m_gripper.GetGripState(g_state);
        std::cout<< "1 current grip state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(currpos);
        std::cout<< "2 current posistion " << currpos << std::endl;

    //set gripper target position 0
        m_gripper.SetTargetPosition(0);
    
    //wait gripper catch a object or arrived target position
        g_state = 0;
        while(g_state == DH_Modbus_Gripper::S_GRIP_MOVING)
            m_gripper.GetGripState(g_state);
        std::cout<< "3 current state " << g_state << std::endl;

    //get gripper current position
        m_gripper.GetCurrentPosition(currpos);
        std::cout<< "6 current position " << currpos << std::endl;

        std::cout<<  std::endl;
        std::cout<<  std::endl;
     }
     m_gripper.close();
}

int main()
{
	ModbusGripper_test();

	return 0;
}
