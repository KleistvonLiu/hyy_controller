#include "HYYRobotInterface.h"
#include <math.h>
using namespace HYYRobotBase;//仅c++需要

static double gaussrand() {
    /* 简单高斯噪声 (Box-Muller) */
    static int has = 0;
    static double v2;
    if (has) { has = 0; return v2; }
    double u1, u2, s = 0.0;
    do {
        u1 = (rand()+1.0) / (RAND_MAX+2.0);
        u2 = (rand()+1.0) / (RAND_MAX+2.0);
        u1 = 2*u1 - 1; u2 = 2*u2 - 1;
        s = u1*u1 + u2*u2;
    } while (s >= 1.0 || s == 0.0);
    double mult = sqrt(-2.0 * log(s) / s);
    v2 = u2 * mult;
    has = 1;
    return u1 * mult;
}

int ServoCartesianDemo()
{
	double t=0;
	double A=0.05;
	double f=0.1;
	double joint_init[7];
	GetCurrentTargetJoint(joint_init,0);
	RTimer timer;
  	initUserTimer(&timer, 0, 50);
    SETJOINT7(home,1,1,1,1,1,1,1);
	SETSPEED(sp,0.1,0.1);
	MoveA(home,sp,NULL3);
	robpose pospose;
	robpose pospose_target;
	GetCurrentTargetCartesian(NULL,NULL, &pospose, 0);
	memcpy(&pospose_target,&pospose,sizeof(robpose));
 	ServoStart(1e-6,0.2,0);
	while (robot_ok())
	{
		userTimer(&timer);
		t=t+0.05;

		if (t>10)
		{
			break;
		}

		double pos=A*cos(2*3.14*f*t)-A+pospose.xyz[0]+gaussrand()*0.003;
		pospose_target.xyz[0]=pos;
		ServoCartesian(&pospose_target,t,NULL,NULL,0);
	}

	ServoEnd(0);
	sleep(1);

    return 0;
}