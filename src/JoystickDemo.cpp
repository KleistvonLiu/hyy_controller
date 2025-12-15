#include "HYYRobotInterface.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <linux/input.h>
#include <linux/joystick.h>
#include <stdint.h>
#include <math.h>
using namespace HYYRobotBase;//仅c++需要

#define DX 0.2
#define DY 0.2
#define DZ 0.1
#define SV 0.1
#define PROB 0.15

#define STARTP R0_STARTP
#define OUTP R0_OUTP

#define MIN(a,b) ((a)<(b)?(a):(b))
#define SIGN(a) (((a)>=0)?1:-1)

double random_num=0;

robpose GetTargetP(int dir)
{
    IMPORTPOSE(R0_STARTP);
    robpose cp;
    GetCurrentCartesian(NULL,NULL,&cp,0);
    double tx=0,ty=0;
    switch (dir)
    {
    case 0://x+
        tx=STARTP->xyz[0]+DX-cp.xyz[0];
        break;
    case 1://x-
        tx=STARTP->xyz[0]-DX-cp.xyz[0];
        break;
    case 2://y+
        ty=STARTP->xyz[1]+DY-cp.xyz[1];
        break;
    case 3://y-
        ty=STARTP->xyz[1]-DY-cp.xyz[1];
        break;
    case 4://x+,y+
        tx=SIGN(STARTP->xyz[0]+DX-cp.xyz[0])*MIN(fabs(STARTP->xyz[0]+DX-cp.xyz[0]),fabs(STARTP->xyz[1]+DY-cp.xyz[1]));
        ty=SIGN(STARTP->xyz[1]+DY-cp.xyz[1])*MIN(fabs(STARTP->xyz[0]+DX-cp.xyz[0]),fabs(STARTP->xyz[1]+DY-cp.xyz[1]));
        break;
    case 5://x+,y-
        tx=SIGN(STARTP->xyz[0]+DX-cp.xyz[0])*MIN(fabs(STARTP->xyz[0]+DX-cp.xyz[0]),fabs(STARTP->xyz[1]-DY-cp.xyz[1]));
        ty=SIGN(STARTP->xyz[1]-DY-cp.xyz[1])*MIN(fabs(STARTP->xyz[0]+DX-cp.xyz[0]),fabs(STARTP->xyz[1]-DY-cp.xyz[1]));
        break;
    case 6://x-,y+
        tx=SIGN(STARTP->xyz[0]-DX-cp.xyz[0])*MIN(fabs(STARTP->xyz[0]-DX-cp.xyz[0]),fabs(STARTP->xyz[1]+DY-cp.xyz[1]));
        ty=SIGN(STARTP->xyz[1]+DY-cp.xyz[1])*MIN(fabs(STARTP->xyz[0]-DX-cp.xyz[0]),fabs(STARTP->xyz[1]+DY-cp.xyz[1]));
        break;
    case 7://x+,y-
        tx=SIGN(STARTP->xyz[0]-DX-cp.xyz[0])*MIN(fabs(STARTP->xyz[0]-DX-cp.xyz[0]),fabs(STARTP->xyz[1]-DY-cp.xyz[1]));
        ty=SIGN(STARTP->xyz[1]-DY-cp.xyz[1])*MIN(fabs(STARTP->xyz[0]-DX-cp.xyz[0]),fabs(STARTP->xyz[1]-DY-cp.xyz[1]));
        break;
    }
    cp.xyz[0]+=tx;
    cp.xyz[1]+=ty;
    cp.xyz[2]=STARTP->xyz[2]+DZ;
    return cp;
}

void MoveXY(robpose tp)
{
    SETSPEED(v,SV,SV);
    setMoveThread(1);
    MoveL(&tp,v,NULL3);
    setMoveThread(0);
}

void MoveStop()
{
     RobotStopRun(0);
}

void OpenGrip()
{
    SetDo(0,0);
    usleep(10000);
    SetDo(0,1);
    usleep(10000);
    SetDo(0,0);
}

void CloseGrip()
{
    SetDo(1,0);
    usleep(10000);
    SetDo(1,1);
    usleep(10000);
    SetDo(1,0);
}

static int IsRunMoveOut=0;
void* _MoveOut(void* arg)
{
    IMPORTPOSE(R0_STARTP);
    // IMPORTZONE(z1);
    robpose cp;
    GetCurrentCartesian(NULL,NULL,&cp,0);
    SETSPEED(v,SV,SV);
    cp.xyz[2]=STARTP->xyz[2];
    MoveL(&cp,v,NULL3);//down
    CloseGrip();
    usleep(700000);
    cp.xyz[2]=STARTP->xyz[2]+DZ;
    MoveL(&cp,v,NULL3);//up
    if (random_num>PROB)
    {
        OpenGrip();
        usleep(700000);
        CloseGrip();
    }
    IMPORTPOSE(R0_OUTP);
    robpose outup=Offs(OUTP,0,0,DZ,0,0,0);
    MoveL(&outup,v,NULL,NULL2);
    MoveL(OUTP,v,NULL3);//down
    OpenGrip();
    usleep(700000);
    MoveL(&outup,v,NULL,NULL2);//up
    STARTP->xyz[2]+=DZ;
    MoveL(STARTP,v,NULL3);//up
    ThreadDataFree("MoveOut");
    IsRunMoveOut=0;
    return NULL;
}


void MoveOut()
{
    IsRunMoveOut=1;
    ThreadCreat(_MoveOut, NULL, "MoveOut", 1);
}




int JoystickDemo()
{
    InputEvent ie;
    struct js_event js;
    srand(time(NULL));
    int ret=input_event_open(&ie,"/dev/input/js0",0xFF,7,1);
    if (0!=ret)
    {
        printf("JoystickDemo:input_event_open failure(%d)\n",ret);
        return 0;
    }
    IMPORTPOSE(R0_STARTP);
    SETSPEED(v,0.1,0.1);
    R0_STARTP->xyz[2]+=DZ;
    MoveL(R0_STARTP,v,NULL3);
    OpenGrip();
    while (1)
    {
        if (0==input_js_event_read(&ie,&js))
        {
            // printf("%u,%u,%d\n",js.type,js.number,js.value);
            if (!IsRunMoveOut)
            {
                if (1==js.type && (js.number>=0&&js.number<=7)&&1==js.value)
                {
                    if (7==js.number)
                    {
                        random_num=0;
                    }
                    else
                    {
                        random_num = (double)rand() / RAND_MAX;
                    }
                    printf("button:%u\n",js.number);
                    MoveOut();
                }
                if (2==js.type)
                {
                    if (1==js.number&&js.value>0)//x+
                    {
                        printf("x+\n");
                        MoveStop();
                        MoveXY(GetTargetP(0));
                    }else if (1==js.number&&js.value<0)//x-
                    {printf("x-\n");
                        MoveStop();
                        MoveXY(GetTargetP(1));
                    }else if (0==js.number&&js.value>0)//y+
                    {printf("y+\n");
                        MoveStop();
                        MoveXY(GetTargetP(2));
                    }else if (0==js.number&&js.value<0)//y-
                    {printf("y-\n");
                        MoveStop();
                        MoveXY(GetTargetP(3));
                    }else
                    {printf("stop\n");
                       MoveStop();
                    }
                }
            }
        }
    }

    return 0;
}