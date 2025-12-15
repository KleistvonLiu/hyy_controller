#include "HYYRobotInterface.h"
#include <math.h>
using namespace HYYRobotBase;//仅c++需要
int ServoDemo()
{
    RTimer timer;
    initUserTimer(&timer,0,1);//定时周期为1倍总线周期
    //获取索要操作的机器人名称
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);
    double td=get_control_cycle(get_deviceName(0,NULL));//获取总线周期
    double joint[10];
    int dof=get_group_dof(robot_name);//获取机器人关节数目
    GetGroupPosition(robot_name,joint);//获取所有关节位置
    int i=0;
    for(i=0;i<dof;i++)
    {
        printf("%d:%f\n",i,joint[i]);
    }
    //group_power_off(robot_name);//机器人所有轴下电
    group_power_on(robot_name);
    sleep(1);
    int axis_ID=1;//指定操作单轴id
    //axis_power_on(robot_name,axis_ID);//单轴上电
    double A=1;double f=0.1;double t=0;
    //获取单轴的关节位置
    double pos_base=GetAxisPosition(robot_name,axis_ID);
    double pos_target=0;double pos_real=0;double pos_target_1=0;
    while(robot_ok())//循环，当程序被强制停止或遇到错误时循环退出
    {
        userTimer(&timer);//定时
        pos_target=A*cos(3.14*2*f*t)-A+pos_base;//设置期望数据
        //获取当前单轴关节目标位置
        pos_target_1=GetAxisTargetPosition(robot_name,axis_ID);
        pos_real=GetAxisPosition(robot_name,axis_ID);//获取当前单轴关节位置
        SetAxisPosition(robot_name,pos_target,axis_ID);//设置单轴关节目标位置
        double tmp[2]={pos_target_1,pos_real};
        //将运行过程中的数据保存到servo_data.txt中，可用于分析
        RSaveDataFast1("servo_data",1,100,2,tmp);
        t+=td;
    }
    return 0;
};



int CartesianServo(robpose* rp,tool* to,wobj* wo,int robot_index)
{
    R7_KINE rkine;
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),robot_index);
    int dof=get_group_dof(robot_name);
    double joint[ROBOT_MAX_DOF];
    GetGroupTargetPosition(robot_name,joint);
    init_R7_KINE2(&rkine,joint,&dof, (TOOL*)to, (WOBJ*)wo);
    set_R7_KINE_pose(&rkine, rp->xyz, rp->kps);
    int ret=Kine_Inverse(robot_name, &rkine);
    if (0!=ret)
    {
        return ERR_INVERSEKINEMATICS;
    }
    ret=IsRobotPositionLimitMargin(robot_name,rkine.joint, 1);
    if (0!=ret)
    {
        return ERR_ROBOTJOINTLIMIT;
    }
    SetGroupPosition(robot_name,rkine.joint);
    return 0;
}

void TorqueServo()
{
    const char* add_name=get_name_additionaxis_device(get_deviceName(0,NULL), 0);

    signed char mode=10;
    set_axis_mode(add_name,mode,1);
    sleep(1);
    int axis_ID=1;//指定操作单轴id
    axis_power_on(add_name,axis_ID);//单轴上电

    RTimer timer;
    initUserTimer(&timer,0,1);//定时周期为1倍总线周期
    double A=1;double f=0.1;double t=0;
    double td=get_control_cycle(get_deviceName(0,NULL));//获取总线周期
    while (1)
    {
        userTimer(&timer);//定时
        double target_torque=A*cos(3.14*2*f*t)-A;
        SetAxisTorque(add_name, target_torque, 1);
        t+=td;
    }
}



