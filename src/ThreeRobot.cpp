#include "HYYRobotInterface.h"
using namespace HYYRobotBase;

void* robot2_move(void* arg)
{
    IMPORTTOOL(R2_tool);
    IMPORTWOBJ(R2_wobjx);
    IMPORTJOINT(R2_jinit_1);
    IMPORTPOSE(R2_init);
    SETSPEED(sp,0.02,0.02);
    SETZONE(zo,0.05);

    SETPOSE(R2_p1,0.038,0,0.19,0,0,0);
    SETPOSE(R2_p2,0.076,0.038,0.19,0,0,0);
    SETPOSE(R2_p3,0.0,0.038,0.19,0,0,0);
    SETPOSE(R2_p4,0.0,-0.038,0.19,0,0,0);
    SETPOSE(R2_p5,-0.076,-0.038,0.19,0,0,0);
    SETPOSE(R2_p6,-0.038,0,0.19,0,0,0);
    SETPOSE(R2_p7,0.076,-0.038,0.19,0,0,0);
    SETPOSE(R2_p8,-0.076,0.038,0.19,0,0,0);
    while (robot_ok())
    {
        MultiMoveL(R2_p1,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveC(R2_p3,R2_p2,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveL(R2_p4,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveC(R2_p6,R2_p5,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveL(R2_p1,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveC(R2_p4,R2_p7,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveL(R2_p3,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveC(R2_p6,R2_p8,sp,zo,R2_tool,R2_wobjx,2);
        MultiMoveL(R2_init,sp,NULL,R2_tool,R2_wobjx,2);
    }
    return NULL;
}

int ThreeRobot()
{
    int ret=0;
    IMPORTTOOL(R2_tool);
    IMPORTWOBJ(R2_wobjx);
    IMPORTJOINT(R2_jinit_1);
    IMPORTPOSE(R2_init);
    SETSPEED(sp,0.02,0.02);
    SETZONE(zo,0.05);



    MultiMoveA(R2_jinit_1,sp,NULL,R2_tool,R2_wobjx,2);
    MultiMoveL(R2_init,sp,NULL,R2_tool,R2_wobjx,2);


    ret=ThreadCreat(robot2_move,NULL,"robot2_move",1 );
	if (0!=ret)
	{
		printf("robot 2 ThreadCreat failure\n");
		return 0;
	}

    while (robot_ok())
    {
        sleep(1);



    }

    return 0;
}



