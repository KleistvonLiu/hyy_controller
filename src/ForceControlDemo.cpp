#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int ForceControlDemo()
{
    //齿轮装配应用
	IMPORTPOSE(p2);
	IMPORTPOSE(p3);
	IMPORTPOSE(p5);
	IMPORTPOSE(p6);
	IMPORTPOSE(p8);
	IMPORTPOSE(p21);
	IMPORTPOSE(p22);
	IMPORTPOSE(p23);
	IMPORTPOSE(p24);
	IMPORTJOINT(j1);
	IMPORTJOINT(j7);
	IMPORTSPEED(v10);
	IMPORTSPEED(v100);
	IMPORTZONE(z0);
	IMPORTTOOL(tool10);
	IMPORTWOBJ(wobj0);
	//移动到初始位置
	MoveA(j1,v100,NULL,NULL,NULL);
    //循环装配
	while (robot_ok())
	{
	    //初始化力控制
	    int ret=SFCInit("SFCtest", 0, 0, tool10, wobj0, 0);
	    printf("SFCInit,ret=%d\n",ret);
        //设置力控参数
	    double M[6]={10,10,10,10,10,1};
        double B[6]={1000,1000,1000,1000,5};
        double K[6]={100,100,0,10000,10000,2};
	    ret=SFCSetAdmittanceCtrlParam("SFCtest", M, B, K);
	    printf("SFCSetAdmittanceCtrlParam,ret=%d\n",ret);
        //设置期望力（非零力控，零导纳控制）
	    double target_force[6]={0,0,10,0,0,0};
	    SFCSetTargetForce("SFCtest", target_force);

		SFCSetForceSensorName("SFCtest", "sensor", 1);

        //移动到装配位置
	    MoveL(p22,v100,z0,tool10,wobj0);
        MoveL(p21,v100,z0,tool10,wobj0);
	    //开启力控制
	    ret=SFCStart("SFCtest");
	    double is_valid[6]={0,0,1,0,0,0};
	    robpose robpose_condition=Offs(p21, 0, 0, -0.015, 0, 0, 0);
        //开始装配
        while (robot_ok())
	    {
            //判断是否完成装配
		    if (SFCIsSatisfyForceCondition("SFCtest", target_force,is_valid,1,0)&&
			    SFCIsSatisfyCartesianCondition("SFCtest", &robpose_condition,is_valid,0.005,0))
		    {
			    break;//退出装配过程
		    }
            //主动旋转，促进装配
		    MoveL(p23,v10,z0,tool10,wobj0);
		    MoveL(p24,v10,z0,tool10,wobj0);
	    }
        //关闭力控
	    ret=SFCEnd("SFCtest");
	    printf("SFCEnd,ret=%d\n",ret);
        //退出装配位置
	    robpose pout;
	    GetCurrentCartesian(tool10,wobj0, &pout, 0);
	    robpose pout1=Offs(&pout, 0, 0, 0.1, 0, 0, 0);
	    MoveL(&pout1,v10,z0,tool10,wobj0);
   	}
    return 0;
}