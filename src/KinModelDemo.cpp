#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int KinModelDemo()
{
    int ret=0;
    //获取索要操作的机器人名称
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);
    int dof=get_group_dof(robot_name);//获取机器人关节数目
    double joint[10];
    GetGroupPosition(robot_name,joint);//获取所有关节位置
	R7_KINE rkine;
    init_R7_KINE2(&rkine,joint,&dof, NULL, NULL);
    ret=Kine_Forward(robot_name, &rkine);//正运动学求解
    printf("Kine_Forward:%d\n",ret);
    double xyz[3];
    double rpy[3];
    get_R7_KINE_pose(&rkine, xyz, rpy);//对应rkine.X,rkine.kps
    printf("x:%f,y:%f,z:%f,k:%f,p:%f,s:%f\n",
    xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]);
    //设置逆运动学参数
    joint[0]+=0.001;joint[5]+=0.001;
    set_R7_KINE_joint(&rkine, joint);//设置当前位置(用于选解)
    set_R7_KINE_pose(&rkine, xyz, rpy);//设置待求目标
    ret=Kine_Inverse(robot_name, &rkine);//逆运动学
    printf("Kine_Inverse:%d\n",ret);
    double joint_ik[10];
    get_R7_KINE_joint(&rkine, joint_ik);//对应rkine.joint
    printf("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f\n",
    joint_ik[0],joint_ik[1],joint_ik[2],joint_ik[3],joint_ik[4],joint_ik[5]);
    //超过关节位置约束的0.8倍, 返回超限轴ID, 否则返回0
    ret=IsRobotPositionLimitMargin(robot_name,joint_ik, 0.8);
    printf("IsRobotPositionLimitMargin:%d\n",ret);
    return 0;
}