#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
#include <math.h>
static double kps[3]={-3.141587,-0.066132,0};
const double max_value=10000.0;
static int _move_state=0;//0:stop; 1:x+; -1:x-; 2:y+; -2:y-; 3:z+; -3:z-
static int _move_state_1=0;//0:stop; 1:x+; -1:x-; 2:y+; -2:y-; 3:z+; -3:z-
static void MoveAxis(int cmd,speed* sp,tool* to)
{
    robpose pospose;
    GetCurrentTargetCartesian(to,NULL, &pospose, 0);
    pospose.kps[0]=kps[0];pospose.kps[1]=kps[1];pospose.kps[2]=kps[2];
    double x=0,y=0,z=0;
    switch (cmd)
    {
    case 1:
        x=max_value;
        break;
    case -1:
        x=-max_value;
        break;
    case 2:
        y=max_value;
        break;
    case -2:
        y=-max_value;
        break;
    case 3:
        z=max_value;
        break;
    case -3:
        z=-max_value;
        break; 
    }
    robpose tmp = Offs(&pospose, x, y, z, 0, 0, 0);
    MoveL(&tmp,sp,NULL,to,NULL);
}

static void MoveAxisStop()
{
    DeviceStopRun();
}

int MainModule()
{
    SETJOINT6(jinit,0.0,30.0/180.0*3.1415926,0.0,0.0,55.8/180.0*3.14159265,0.0);
    SETSPEED(v100,0.1,0.1);
    SETSPEED(v50,0.05,0.05);
    IMPORTTOOL(tool15);
    //-------------------设置允许空间---------------------------
    int ret=0;

    if (1)//设置外沿缓慢区域
    {
        double centreb[3]={0,0,0.17};
        double centreh[3]={0,0,0.6};//2-半径 740，高度 340 开始由外 //0.50改0.36
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0.8, 1.19, _constraint_allowslow, NULL, NULL, "cyl_ext_slow");
        SetSpatialConstraintSlowCoeff(0.2, "cyl_ext_slow");
    }

    if (1)//设置蓝色桶 //3-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    {
        double centreb[3]={0.7359,0.9417,0};
        double centreh[3]={0.7359,0.9417,0.5936};
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0, 0.235, _constraint_slowallow, NULL, NULL, "cylinder_blue_slow");
	    SetSpatialConstraintSlowCoeff(0.2, "cylinder_blue_slow");
    }

    if (1)//设置蓝色桶 //3-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    {
        double centreb[3]={0.748,-0.9429,0};
        double centreh[3]={0.748,-0.9429,0.5936};
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0, 0.235, _constraint_slowallow, NULL, NULL, "cylinder_red_slow");
	    SetSpatialConstraintSlowCoeff(0.2, "cylinder_red_slow");
    }

    //----------------设置不允许空间---------------

    if (1)//设置外沿不允许区域
    {
        double centreb[3]={0,0,0.081};
        double centreh[3]={0,0,0.65};//2-半径 740，高度 340 开始由外 //0.50改0.36
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0.754, 1.22, _constraint_allow, NULL, NULL, "cyl_ext_no");
    }
    if (1)//设置蓝色桶 //3-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    {
        double centreb[3]={0.7359,0.9417,0};
        double centreh[3]={0.7359,0.9417,0.5436};
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0, 0.230, _constraint_noallow, NULL, NULL, "cylinder_blue_no");
    }

    if (1)//设置蓝色桶 //3-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    {
        double centreb[3]={0.748,-0.9429,0};
        double centreh[3]={0.748,-0.9429,0.5436};
        ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0, 0.230, _constraint_noallow, NULL, NULL, "cylinder_red_no");
    }

    if (0!=ret)
    {
	printf("AddSpatialConstraintCy failure!\n");
	return -1;
    }
    OffSpatialConstraint();//关闭笛卡尔空间约束检测，允许强行恢复运动
    
    MoveA(jinit,v100,NULL3);
    robpose pinit;
    GetCurrentCartesian(tool15,NULL, &pinit, 0);

    OnSpatialConstraint();//开启笛卡尔空间约束检测
    setMoveThread(1);

    ret=SocketCreate1(NULL, 8888, 1e6, "game");
    if (0!=ret)
    {
        printf("SocketCreate1 failure\n");
        return -2;
    }

    uint8_t data[1024];
    int len=0;
    uint8_t count=0;

    while (1)
    {
        len=SocketRecvByteI(data, 1024, "game");
        if (len<=0)
        {
            MoveAxisStop();
	        _move_state_1=0;
            count++;
            if (10==count)
            {
                break;
            }
            continue;
        }
        count=0;
        //判断数据格式是否正确
        if (!IsProtocolRight(data,len))
        {
            MoveAxisStop();
	    _move_state_1=0; 
            continue;
        }
        int rs=get_robot_move_state(0);
        if (rs<0)
        {
            clear_robot_move_error(0);//强行清楚错误，允许强行恢复运动
            OffSpatialConstraint();//关闭笛卡尔空间约束检测，允许强行恢复运动
            robpose pcurr;
            GetCurrentCartesian(tool15,NULL, &pcurr, 0);
            robpose ptarget;
            ptarget.xyz[0]=(pinit.xyz[0]-pcurr.xyz[0])*0.3+pcurr.xyz[0];
            ptarget.xyz[1]=(pinit.xyz[1]-pcurr.xyz[1])*0.3+pcurr.xyz[1];
            ptarget.xyz[2]=(pinit.xyz[2]-pcurr.xyz[2])*0.3+pcurr.xyz[2];
            ptarget.kps[0]=kps[0];
            ptarget.kps[1]=kps[1];
            ptarget.kps[2]=kps[2];
            MoveL(&ptarget,v50,NULL,tool15,NULL);
	        wait_move_finish(0);
            _move_state_1=0;
            OnSpatialConstraint();//开启笛卡尔空间约束检测
            continue;
        }

        //解析数据
        GetProtocolDataInt(data,"cmd",&_move_state);
        if (_move_state==_move_state_1)
        {
            continue;
        }
        rs=get_robot_move_state(0);
        if ((0!=_move_state)&&(0!=_move_state_1))
        {
            MoveAxisStop();
        }

        if (0!=_move_state)
        {
            MoveAxis(_move_state,v100,tool15);
        }
        else
        {
            MoveAxisStop();
        }
        
        _move_state_1=_move_state;
    }
    SocketClose("game");
    DeleteAllSpatialConstraint();
    printf("game end\n");
    return 0;
}