#include "HYYRobotInterface.h"
#include "math.h"
using namespace HYYRobotBase;//仅c++需要

//轨迹引导程序
void* egm_guide_function(void* arg)
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
		    target[0]=0.1*cos(2*3.14*0.2*t)-0.1;
		    SetEGMInput(target,0);
		    t+=0.001;
		}

	}
    return NULL;
}

int EgmGuideDemoInternal()
{
    //创建外部修正数据区
	int ret=EGMCreate(1,NULL, NULL,0);
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
	//启用外部引导
	EGMGuideMove(0);

	//创建引导线程
    ThreadCreat(egm_guide_function, NULL, "egm_test", 1);

	Rsleep(1000000000);
	ThreadDataFree("egm_test");
	EGMStop(0);
	EGMDelete(0);
	return 0;
}