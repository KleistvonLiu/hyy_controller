#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int TeachDemo()
{
    set_robot_index(0);//选择要示教的机器人
    set_robot_teach_coordinate(1);//设置选中机器人的坐标系机器人为基坐标系
    robot_teach_enable();//选中的机器人上电
    robot_teach_move(2,1);//沿指定的基座系的z轴正向移动（非阻塞）
    Rsleep(2000);//2s
    robot_teach_stop();//停止示教移动
    start_robot_c_project("test");//启动test.c项目程序
    Rsleep(5000);//5s
    close_robot_c_project();//停止c项目的运行
    return 0;
}