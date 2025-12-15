#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int ReadDataDemo()
{
    int ret=0;
    double target_position[10];
    //创建读服务,打开文件readtest.txt,内部存放按照控制周期离散的数
    CreateReadData1("readtest", 1, 200, 6);
    Rsleep(1000);//1s,等待读数据服务准备好
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);
    IMPORTSPEED(v100);
    SETJOINT6(jinit,1,1,1,1,1,1);
    MoveA(jinit,v100,NULL,NULL,NULL);//运动到初始位置
    RTimer timer;
    initUserTimer(&timer, 0, 1);
    //循环读取数据并下发给设备
    while (robot_ok())
    {
        userTimer(&timer);
        ret=RReadData("readtest", target_position );//读一行数据
        if (ret<0)//数据读完
        {
            printf("RReadData, ret=%d\n",ret);
            break;
        }
        SetGroupPosition(robot_name, target_position);
    }
    DeleteReadData("readtest");//删除读服务
    return 0;
}