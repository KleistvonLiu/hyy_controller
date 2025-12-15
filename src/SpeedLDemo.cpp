#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SpeedLDemo()
{
 	SETJOINT7(home,0.1,0.1,0.1,1,0.1,1,0.1);
	SETSPEED(sp,0.2,0.2);
	MoveA(home,sp,NULL3);
	printf("xxxx\n");
	setMoveThread(1);
	double vel[6]={0.004,0.005,-0.0015,0.1,-0.01,0.03};
	int ret=SpeedL(vel,NULL, NULL);
	printf("==%d\n",ret);

	sleep(10);

	DeviceStopRun() ;

	printf("xxxx\n");
	sleep(5);
    return 0;
}