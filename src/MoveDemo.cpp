#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int MoveDemo()
{
    //初始化数据
	SETJOINT6(rb,0.1,0.5,-1,0.8,-0.3,-0.8);
	SETSPEEDTIME(rbv,5);
	SETJOINT2(ab,0.02,0.03);
	SETSPEEDTIME(abv,5);
	IMPORTWOBJ(wobj1);
	IMPORTPOSE(pa);
    //开启机器人与附加轴协同服务
	int ret=StartAdditionServer(NULL, wobj1,0);
	if (0!=ret)
	{
		printf("StartAdditionServer failure\n");
	}
    //附加轴组与机器人移动(阻塞运动)
	MoveJoint(rb,ab,rbv,abv,NULL3);
    //设置运动指令为非阻塞方式并设置为同步启动模式
	setMoveThread1(1,9e9);
	MoveL(pa,rbv,NULL,NULL,wobj1);//机器人移动(未实际运动)
	MoveAdd(ab,abv,NULL,NULL,NULL);//附加轴组移动(未实际运动)
    //100ms,等待MoveL与MoveAdd规划完成
    //如果不加可能出现附加轴或机器人不运动的情况
	Rsleep(100);
	MoveSyncStart();//同时启动机器人与附加轴组移动
    //100ms,等待机器人与附加轴组运动(如果不加可能出现下面循环误判)
    Rsleep(100);
    //等待运动完成
	while(robot_runing()||addition_runing())
    {
       Rsleep(100);//100ms 
    }
	StopAdditionServer(0);//关闭机器人与附加轴协同服务
    return 0;
}