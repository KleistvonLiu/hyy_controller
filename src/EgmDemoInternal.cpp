#include "HYYRobotInterface.h"
#include "math.h"
using namespace HYYRobotBase;//仅c++需要

//轨迹修正程序
void* egm_function(void* arg)
{
    RTimer mytimer;
	initUserTimer(&mytimer, 0, 1);
	double t=0;
    double target[6]={0,0,0,0,0,0};
	while (robot_ok())
	{
		userTimer(&mytimer);
		if (t<=5)
		{
		    target[1]=0.1*cos(2*3.14*0.2*t)-0.1;
		    SetEGMInput(target,0);
		    t+=0.001;
		}

	}
    return NULL;
}

int EgmDemoInternal()
{
    //创建外部修正数据区
	int ret=EGMCreate(2,NULL, NULL,0);
	if (0!=ret)
    {
        printf("EGMCreate:%d\n",ret);
        return 0;
    }
    //开始修正
	ret=EGMStart(1, 0);
    if (0!=ret)
    {
	    printf("EGMStart:%d\n",ret);
        return 0;
    }
	Rsleep(1000);

    //创建修正线程
    ThreadCreat(egm_function, NULL, "egm_test", 1);

    //移动机器人
	robpose pospose;
	GetCurrentTargetCartesian(NULL,NULL, &pospose, 0);
    robpose pospose1=Offs(&pospose,0.1,0,0.05,0.02,0.01,-0.03);
	IMPORTSPEED(v10);
	while (robot_ok())
	{
	    MoveL(&pospose1,v10,NULL,NULL,NULL);
	    MoveL(&pospose,v10,NULL,NULL,NULL);
	}
	ThreadDataFree("egm_test");
	EGMStop(0);
	EGMDelete(0);
	return 0;
}