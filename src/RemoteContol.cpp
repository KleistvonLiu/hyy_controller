
#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要

static double kps[3]={-3.1415926,0,0};
const double max_value=10000.0;
static int _move_state=0;//0:stop; 1:x+; -1:x-; 2:y+; -2:y-; 3:z+; -3:z-
static int _move_state_1=0;//0:stop; 1:x+; -1:x-; 2:y+; -2:y-; 3:z+; -3:z-
static void MoveAxis(int cmd,speed* sp)
{
    robpose pospose;
    GetCurrentTargetCartesian(NULL,NULL, &pospose, 0);
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
    MoveL(&tmp,sp,NULL,NULL,NULL);
}

static void MoveAxisStop()
{
    DeviceStopRun();
}

int RemoteControl()
{
    SETJOINT6(jinit,0.0,0.0,0.0,0.0,1.5707963267949,0.0);
    SETSPEED(v100,0.1,0.1);
    //-------------------设置允许空间---------------------------
    int ret=0;
    //设置底面
    double centre1[3]={-10000, -10000,0};
    double length[3]={10000,-10000,0};
    double width[3]={-10000,10000,0};
    double high[3]={-10000, -10000,0.315};//1-修改夹爪最低能触碰限位 315 向上范围
    ret=ret|AddSpatialConstraintCuboid(centre1, length, width, high, _constraint_slowallow, NULL, NULL, "cuboid1");
 	
    //设置附近台阶
    double centreb[3]={0,0,0};
    double centreh[3]={0,0,0.315};//2-半径 740，高度 340 开始由外
    ret=ret|AddSpatialConstraintCylinder(centreb, centreh, 0, 0.730, _constraint_slowallow, NULL, NULL, "cylinder1");

    //设置蓝色桶 //3-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    double centreb1[3]={0.753,-0.925,0};
    double centreh1[3]={0.753,-0.925,0.750};
    ret=ret|AddSpatialConstraintCylinder(centreb1, centreh1, 0, 0.235, _constraint_slowallow, NULL, NULL, "cylinder2");

    //设置红色桶 //4-圆柱体以外的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    double centreb2[3]={0.753,0.925,0};//圆柱体的下面
    double centreh2[3]={0.753,0.925,0.750};//圆柱体的上面，200 为圆柱体的半径
    ret=ret|AddSpatialConstraintCylinder(centreb2, centreh2, 0, 0.235, _constraint_slowallow, NULL, NULL, "cylinder3");

    //----------------设置不允许空间---------------
    //设置底面
    double high1[3]={-10000, -10000,0.305};//1-315以下
    ret=ret|AddSpatialConstraintCuboid(centre1, length, width, high1, _constraint_noallow, NULL, NULL, "cuboid2");
	
    //设置附近台阶
    double centrehx[3]={0,0,0.500};//2-半径 740，高度 500 开始由内
    ret=ret|AddSpatialConstraintCylinder(centreb, centrehx, 0, 0.730, _constraint_noallow, NULL, NULL, "cylinder4");

    //设置蓝色桶 //3-圆柱体以内的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    ret=ret|AddSpatialConstraintCylinder(centreb1, centreh1, 0, 0.235, _constraint_noallow, NULL, NULL, "cylinder5");

    //设置红色桶 //4-圆柱体以内的距离为允许空间，以200为半径确定圆柱体上面和下面圆心，给定高度
    ret=ret|AddSpatialConstraintCylinder(centreb2, centreh2, 0, 0.235, _constraint_noallow, NULL, NULL, "cylinder6");
    if (0!=ret)
    {
	    printf("AddSpatialConstraintCy failure!\n");
	    return -1;
    }
    OffSpatialConstraint();//关闭笛卡尔空间约束检测，允许强行恢复运动
    
    MoveA(jinit,v100,NULL3);

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
    double joint[10];
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
            SETJOINT6(rj,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
            MoveA(rj,v100,NULL,NULL,NULL);
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
        if (rs>=0)
        {
            GetCurrentTargetJoint(joint, 0);
        }
        if ((0!=_move_state)&&(0!=_move_state_1))
        {
            MoveAxisStop();
        }

        if (0!=_move_state)
        {
            MoveAxis(_move_state,v100);
        }
        else
        {
            MoveAxisStop();
        }
        
        _move_state_1=_move_state;
    }
    SocketClose("game");
    DeleteAllSpatialConstraint();

    return 0;
}


