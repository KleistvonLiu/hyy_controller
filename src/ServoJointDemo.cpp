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

int ServoJointDemo()
{


	double t=0;
	double A=1;
	double f=0.1;
	double joint_init[7];
	GetCurrentTargetJoint(joint_init,0);
	double joint[7];
	robjoint joint_target;
	joint_target.dof=7;
	RTimer timer;
  	initUserTimer(&timer, 0, 50);

	ServoStart(1e-6,0.2,0);
	while (robot_ok())
	{
		userTimer(&timer);
		t=t+0.05;

		if (t>10)
		{
			break;
		}

		double pos=A*cos(2*3.14*f*t)-A+joint_init[0]+gaussrand()*0.01;
		memcpy(joint_target.angle,joint_init,sizeof(joint_init));
		joint_target.angle[0]=pos;
		ServoJoint(&joint_target,t,0);
	}

	ServoEnd(0);
	sleep(1);

    return 0;
}