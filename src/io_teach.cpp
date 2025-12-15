#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "HYYRobotInterface.h"

using namespace HYYRobotBase;
void io_test()
{
    RTimer timer;
    initUserTimer(&timer, 0, 100);
    int io1_index=19;
    int io2_index=20;
    int io1_1=GetDi(io1_index);
    int io1=0;
    int io2_1=GetDi(io2_index);
    int io2=0;
    while (1)
    {
        userTimer(&timer);
        io1=GetDi(io1_index);
        io2=GetDi(io2_index);
        if (robot_ok())
        {
            if ((0==io1_1)&&(0!=io1))
            {
                robot_teach_joint(0, 1);
            }
            else if ((0==io2_1)&&(0!=io2))
            {
                robot_teach_joint(0, 1);
            }
            else if ((0==io1)&&(0==io2))
            {
                robot_teach_stop();
            }
        }
        io1_1=io1;
        io2_1=io2;
    }
}



