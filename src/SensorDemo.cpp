#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SensorDemo()
{
	int ret=0;
	//创建传感器(名称在配置文件中已指定)
	ret=CreateTorqueSensor("mysensor");
	printf("CreateTorqueSensor,ret=%d\n",ret);
	int robot_index=0;
	//获取索引robot_index的机器人名称
	const char* robot_name=get_name_robot_device(get_deviceName(0,NULL), robot_index);
	//获取使用的工具
    IMPORTTOOL(tool0);
    IMPORTWOBJ(wobj0);
	//设置传感器转换数据
	ret=SetSensorTorqueTransformData("mysensor", robot_name, tool0, wobj0);
	printf("setSensorTorqueTransformData,ret=%d\n",ret);
	//创建定时器
	RTimer mytimer;
	initUserTimer(&mytimer, 0, 1);
    //循环数据采集
	while(robot_ok())
	{
		//定时
		userTimer(&mytimer);
		//获取传感器的原始数据
		double torque[6];
		ret=GetSensorTorque("mysensor", torque);
		if (0!=ret)
		{
			printf("TorqueSensorCalibration,ret=%d\n",ret);
		}
        //记录运行采集传感器数据(快速返回，不阻塞时钟)
        RSaveDataFast1("sensor",1, 100, 6, torque );//保存到sensor.txt中
        double angle[10];
        double torque_tool[6];
        GetGroupPosition(robot_name,angle);
		ret=GetSensorTorqueTransformToTool("mysensor", angle, torque_tool);
		//记录运行采集传感器数据(快速返回，不阻塞时钟)
		RSaveDataFast1("sensor_tool",1, 100, 6, torque_tool );//保存到sensor_tool.txt中
	}
    return 0;
}