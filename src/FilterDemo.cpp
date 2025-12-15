#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int FilterDemo()
{
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);
    double dt=get_control_cycle(get_deviceName(0,NULL));
    int dof=get_group_dof(robot_name);
	TIIRFilters fvel;
	initTIIRFilters(&fvel, iirLPF, 10, 1.0/dt, dof);
    RTimer timer;
    initUserTimer(&timer,0,1);//定时周期为1倍总线周期
    while(robot_ok())
    {
        userTimer(&timer);
        double velocity[20];
        GetGroupVelocity(robot_name,velocity);
        IIRFilters(&fvel, velocity, &(velocity[6]));//滤波
        RSaveDataFast1("filler_test",1, 100, 12, velocity );//保存到filler_test.txt中
    }
    return 0;
}