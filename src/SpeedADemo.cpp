#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SpeedADemo()
{

	SETJOINT7(home,0,0,0,0,0,0,0);
	SETSPEED(sp,0.2,0.2);
	MoveA(home,sp,NULL3);
	printf("xxxx\n");
	setMoveThread(1);
	double vel[7]={0.1,0.03,0,0,0,-0.05,0.08};
	int ret=SpeedA(vel,NULL, NULL);
	printf("==%d\n",ret);

	sleep(10);

	DeviceStopRun() ;

	printf("xxxx");
	sleep(5);

    return 0;
}