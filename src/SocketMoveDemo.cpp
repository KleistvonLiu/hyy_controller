#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SokcetMoveDemo()
{
    double z_offs=0.01;//mm
	CreateGrip("dhgrip");//创建夹爪
    //创建clinet
	int ret=ClientCreate("192.168.1.100", 8888, "camera");
	if (0!=ret)
	{
		printf("ClientCreate failure, err=%d\n",ret);
		return ret;
	}
	double camera_data[10];
	IMPORTJOINT(j1);
	IMPORTPOSE(p2);
	IMPORTSPEED(v10);
	IMPORTTOOL(tool10);	
	robpose p2_1=Offs(p2, 0, 0, z_offs, 0, 0, 0);
	while (robot_ok())//循环操作
	{
		MoveA(j1,v10,NULL3);
		//触发相机拍照并计算目标数据
		SocketSendString("S", "camera");
        //获取运动目标数据
		SocketRecvDoubleArray(camera_data, "camera");
		if (0==camera_data[0])//无目标，从新开始
		{
			continue;
		}
		SETPOSE(pc,camera_data[1],camera_data[2],camera_data[3],
        camera_data[4],camera_data[5],camera_data[6]);//设置目标数据
		robpose pc_1=Offs(pc, 0, 0, z_offs, 0, 0, 0);
		MoveL(&pc_1,v10,NULL,tool10,NULL);
		MoveL(pc,v10,NULL,tool10,NULL);
		ControlGrip("dhgrip", 1);//夹爪闭合
		Rsleep(1000);
		MoveL(&pc_1,v10,NULL,tool10,NULL);
		MoveL(&p2_1,v10,NULL,tool10,NULL);
		MoveL(p2,v10,NULL,tool10,NULL);
		ControlGrip("dhgrip", 0);//夹爪打开
		Rsleep(1000);
		MoveL(&p2_1,v10,NULL,tool10,NULL);
	}	
	DestroyGrip("dhgrip");//释放夹爪数据
	SocketClose("camera");//关闭client
    return 0;
}