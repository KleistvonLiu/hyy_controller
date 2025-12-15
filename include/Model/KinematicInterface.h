/**
 * @file KinematicInterface.h
 *
 * @brief  机器人运动学接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef KINEMATICINTERFACE_H_
#define KINEMATICINTERFACE_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif


/*-------------------------------------------------------------------------*/
/**
  @brief	运动学数据

	描述机器人笛卡尔位姿数据和关节数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double		R[3][3];/**< 姿态矩阵*/
	double		X[3];/**< 位置*/
	double		joint[10];/**< 关节位置*/
	double		kps[3];/**< 姿态*/
	int       		dof;/**< 机器人自由度*/
	double      redundancy;/**< 机器人冗余角*/

	TOOL tool;/**< 工具描述*/
	WOBJ wobj;/**< 工件描述*/
}R7_KINE;

/*-------------------------------------------------------------------------*/
/**
  @brief	运动速度数据

	描述机器人笛卡尔位姿数据和关节速度数据
 */
/*-------------------------------------------------------------------------*/
typedef struct{
	double joint[ROBOT_MAX_DOF];/**< 关节位置*/
	double joint_vel[ROBOT_MAX_DOF];/**< 关节速度*/
	double pose_vel[ROBOT_MAX_DOF];/**< 笛卡尔速度*/
	double joint_acc[ROBOT_MAX_DOF];/**< 关节加速度*/
	double pose_acc[ROBOT_MAX_DOF];/**< 笛卡尔加速度*/
	int dof;/**< 机器人自由度*/
	TOOL tool;/**< 工具描述*/
	WOBJ wobj;/**< 工件描述*/
}R_KINE_VEL;

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 */
extern void init_R7_KINE(R7_KINE* rkine);

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param angle 机器人关节,单位: rad
 * @param dof 机器人自由度
 */
extern void init_R7_KINE1(R7_KINE* rkine, const double* angle, const int* dof);

/**
 * @brief 初始化运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param angle 机器人关节,单位: rad
 * @param dof 机器人自由度
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void init_R7_KINE2(R7_KINE* rkine, const double* angle, const int* dof, const TOOL* tool, const WOBJ* wobj);

/**
 * @brief 设置运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param joint 机器人关节,单位：rad
 */
extern void set_R7_KINE_joint(R7_KINE* rkine, const double* joint);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param X 位置，单位：m
 * @param RPY 姿态，单位：rad
 */
extern void set_R7_KINE_pose(R7_KINE* rkine, const double* X, const double* RPY);

/**
 * @brief 获取运动学数据(R7_KINE)
 *
 * @param rkine 运动数据
 * @param joint 返回机器人关节,单位：rad
 */
extern void get_R7_KINE_joint(R7_KINE* rkine, double* joint);

/**
 * @brief 获取运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param X 返回位置，单位：m
 * @param RPY 返回姿态，单位：rad
 */
extern void get_R7_KINE_pose(R7_KINE* rkine, double* X, double* RPY);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void setToolWobjToR7_KINE(R7_KINE* rkine, const TOOL* tool, const WOBJ* wobj);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param tool 工具描述
 */
extern void setToolToR7_KINE(R7_KINE* rkine, const TOOL* tool);

/**
 * @brief 设置运动学数据(R7_KINE)
 * @param rkine 运动数据
 * @param wobj 工件描述
 */
extern void setWobjToR7_KINE(R7_KINE* rkine, const WOBJ* wobj);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL(R_KINE_VEL* rkine_vel, const int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：rad/s
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL1(R_KINE_VEL* rkine_vel, const double* joint,const double* joint_vel, const int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param pose_vel 笛卡尔位姿速度，单位：m/s，rad/s
 * @param dof 机器人自由度
 */
extern void init_R_KINE_VEL2(R_KINE_VEL* rkine_vel,const  double* joint,const double* pose_vel, const int dof);

/**
 * @brief 初始化运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：rad/s
 * @param pose_vel 笛卡尔位姿速度，单位：m/s，rad/s
 * @param joint_acc 关节运动加速度，单位：rad/s^2
 * @param pose_acc 笛卡尔位姿加速度，单位：m/s^2，rad/s^2
 * @param dof 机器人自由度
 * @param tool 工具描述
 * @param wobj 工具描述
 */
extern void init_R_KINE_VEL3(R_KINE_VEL* rkine_vel, const double* joint, const double* joint_vel, const double* joint_acc,const double* pose_vel,const double* pose_acc, const int dof, const TOOL* tool, const WOBJ* wobj);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：m, rad
 * @param joint_vel 关节运动速度，单位：m/s, rad/s
 */
extern void set_R_KINE_VEL_joint(R_KINE_VEL* rkine_vel, const double* joint, const double* joint_vel);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint 关节角度，单位：rad
 * @param pose_vel 笛卡尔位姿速度，单位：m/s，rad/s
 */
extern void set_R7_KINE_VEL_pose(R_KINE_VEL* rkine_vel, const double* joint, const double* pose_vel);

/**
 * @brief 设置运动加速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动加速度
 * @param joint 关节角度，单位：rad
 * @param joint_vel 关节运动速度，单位：m/s，rad/s
 * @param joint_acc 关节运动加速度，单位：m/s^2,rad/s^2
 */
extern void set_R_KINE_ACC_joint(R_KINE_VEL* rkine_vel, const double* joint, const double* joint_vel, const double* joint_acc);

/**
 * @brief 设置运动加速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动加速度
 * @param joint 关节角度，单位：rad
 * @param pose_vel 笛卡尔位姿速度，单位：m/s，rad/s
 * @param pose_acc 笛卡尔位姿加速度，单位：m/s^2，rad/s^2
 */
extern void set_R7_KINE_ACC_pose(R_KINE_VEL* rkine_vel, const double* joint, const double* pose_vel,const double* pose_acc);

/**
 * @brief 获取运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param joint_vel 关节运动速度，单位：rad/s
 */
extern void get_R_KINE_VEL_joint(R_KINE_VEL* rkine_vel, double* joint_vel);

/**
 * @brief 获取运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param pose_vel 笛卡尔位姿速度，单位：m/s，rad/s
 */
extern void get_R7_KINE_VEL_pose(R_KINE_VEL* rkine_vel, double* pose_vel);

/**
 * @brief 获取运动加速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动加速度
 * @param joint_acc 关节运动加速度，单位：m/s^2, rad/s^2
 * @return 关节运动加速度
 */
extern double* get_R_KINE_ACC_joint(R_KINE_VEL* rkine_vel, double* joint_acc);

/**
 * @brief 获取运动加速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动加速度
 * @param pose_acc 笛卡尔位姿加速度，单位：m/s^2，rad/s^2
 * @return 笛卡尔位姿加速度
 */
extern double* get_R7_KINE_ACC_pose(R_KINE_VEL* rkine_vel, double* pose_acc);


/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param tool 工具描述
 * @param wobj 工件描述
 */
extern void setToolWobjToR_KINE_VEL(R_KINE_VEL* rkine_vel, const TOOL* tool,const  WOBJ* wobj);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param tool 工具描述
 */
extern void setToolToR_KINE_VEL(R_KINE_VEL* rkine_vel, const TOOL* tool);

/**
 * @brief 设置运动速度数据(R_KINE_VEL)
 *
 * @param rkine_vel 运动速度
 * @param wobj 工件描述
 */
extern void setWobjToR_KINE_VEL(R_KINE_VEL* rkine_vel, const WOBJ* wobj);

/**
 * @brief 正运动学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param r7kine 运动学数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_Forward(const char* serialLinkName, R7_KINE* r7kine);

/**
 * @brief 逆运动学
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param r7kine 运动学数据
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_Inverse(const char* serialLinkName, R7_KINE* r7kine);

/**
 * @brief 获取机器人齐次矩阵
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param index 齐次矩阵索（0:基坐附加矩阵;1~dof:对应关节的齐次矩阵；dof+1:末端附加矩阵）
 * @param M 返回的齐次矩阵
 */
extern void Kine_HomogeneousMatrix(const char* serialLinkName, double *position,int index,double M[4][4]);

/**
 * @brief 机器人雅可比
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param jac 返回雅克比(6*机器人自由度),注意分配适合大小的内存空间,(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 */
extern void Kine_Jacob(const char* serialLinkName, double *position, double** jac,TOOL* to,WOBJ* wo);

/**
 * @brief 机器人各轴雅可比
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param axis_id 关节位置1,2,...,dof,dof+1(考虑末端附加变换和工具)
 * @param jac 返回雅克比(6*机器人自由度),注意分配适合大小的内存空间,(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 */
extern void Kine_JacobSub(const char* serialLinkName, double *position, int axis_id, double** jac,TOOL* to,WOBJ* wo);

/**
 * @brief 机器人末端雅可比
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param jac 返回雅克比(6*机器人自由度),注意分配适合大小的内存空间,(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 */
extern void Kine_JacobEnd(const char* serialLinkName, double *position, double** jac,TOOL* to,WOBJ* wo);


/**
 * @brief 机器人雅可比逆/伪逆
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param jacpinv 返回雅克比逆/伪逆(机器人自由度*6),注意分配适合大小的内存空间(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 * @return int 0:成功;其它:雅可比不可逆
 */
extern int Kine_JacobInv(const char* serialLinkName, double *position, double** jacpinv,TOOL* to,WOBJ* wo);

/**
 * @brief 机器人末端雅可比逆/伪逆
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param jacpinv 返回雅克比逆/伪逆(机器人自由度*6),注意分配适合大小的内存空间(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 * @return int 0:成功;其它:雅可比不可逆
 */
extern int Kine_JacobEndInv(const char* serialLinkName, double *position, double** jacpinv,TOOL* to,WOBJ* wo);


/**
 * @brief 机器人雅可比导数与速度导数乘积
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param position 关节位置
 * @param velocity 关节速度
 * @param jacdot 返回雅可比导数与速度导数乘积(6),注意分配适合大小的内存空间(工具到工件下的描述)
 * @param to 工具
 * @param wo 工件
 */
extern void Kine_JacobDot(const char* serialLinkName, double *position, double *velocity,double* jacdot,TOOL* to,WOBJ* wo);

/**
 * @brief 给定笛卡尔速度求关节速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 运动速度数据(设置关节位置，笛卡尔速度，反回关节速度)
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_EndToJointVelocity(const char* serialLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定关节速度求笛卡尔速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 运动速度数据(设置关节位置，关节速度，反回笛卡尔速度)
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_JointToEndVelocity(const char* serialLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定笛卡尔加速度求关节加速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 笛卡尔运动速度数据及加速度
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_EndToJointAcceleration(const char* serialLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定关节加速度求笛卡尔加速度
 *
 * @param serialLinkName 机器人名字(同EtherCAT的机器人子设备名字)
 * @param rkine_vel 关节运动速度数据及加速度
 * @return int 0: 成功；其他: 失败
 */
extern int Kine_JointToEndAcceleration(const char* serialLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 外部轴组正运动学
 *
 * @param addLinkName 外部轴名字(同EtherCAT的外部轴组子设备名字)
 * @param r7kine 运动学数据
 * @return int 0: 成功；其他: 失败
 */
extern int Add_Forward(const char* addLinkName, R7_KINE* r7kine);

/**
 * @brief 获取外部轴组齐次矩阵
 *
 * @param addLinkName 机器人名字(同EtherCAT的外部轴组子设备名字)
 * @param position 关节位置
 * @param index 齐次矩阵索（0:基坐附加矩阵;1~dof:对应关节的齐次矩阵；dof+1:末端附加矩阵）
 * @param M 返回的齐次矩阵
 */
extern void Add_HomogeneousMatrix(const char* addLinkName, double *position,int index,double M[4][4]);

/**
 * @brief 外部轴组雅可比
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param position 关节位置
 * @param jac 返回雅克比(6*机器人自由度),注意分配适合大小的内存空间
 */
extern void Add_Jacob(const char* addLinkName, double *position, double** jac);

/**
 * @brief 外部轴组雅可比逆/伪逆
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param position 关节位置
 * @param jacpinv 返回雅克比逆/伪逆(机器人自由度*6),注意分配适合大小的内存空间
 * @return int 0:成功;其它:雅可比不可逆
 */
extern int Add_JacobInv(const char* addLinkName, double *position, double** jacpinv);

/**
 * @brief 机器人雅可比导数与速度导数乘积
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param position 关节位置
 * @param velocity 关节速度
 * @param jacdot 返回雅可比导数与速度导数乘积(6),注意分配适合大小的内存空间
 */
extern void Add_JacobDob(const char* addLinkName, double *position,double *velocity,double* jacdot);

/**
 * @brief 给定笛卡尔速度求关节速度
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param rkine_vel 运动速度数据(设置关节位置，笛卡尔速度，反回关节速度)
 * @return int 0: 成功；其他: 失败
 */
extern int Add_EndToJointVelocity(const char* addLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定关节速度求笛卡尔速度
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param rkine_vel 运动速度数据(设置关节位置，关节速度，反回笛卡尔速度)
 * @return int 0: 成功；其他: 失败
 */
extern int Add_JointToEndVelocity(const char* addLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定笛卡尔加速度求关节加速度
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param rkine_vel 笛卡尔运动速度数据及加速度
 * @return int 0: 成功；其他: 失败
 */
extern int Add_EndToJointAcceleration(const char* addLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 给定关节加速度求笛卡尔加速度
 *
 * @param addLinkName 外部轴组名字(同EtherCAT的外部轴组子设备名字)
 * @param rkine_vel 关节运动速度数据及加速度
 * @return int 0: 成功；其他: 失败
 */
extern int Add_JointToEndAcceleration(const char* addLinkName, R_KINE_VEL* rkine_vel);

/**
 * @brief 获取机器人关联的外部轴组索引
 *
 * @param robot_index 机器人索引
 * @return int >=0: 机器人所关联的外部轴组索引；其他: 无关联外部轴组
 */
extern int GetRobotLinkAddIndex(int robot_index);

/**
 * @brief 获取外部轴组关联的机器人
 *
 * @param addition_index 外部轴组索引
 * @return int 0:无关联机器人;>0:关联对应非0位的机器人,如：返回5 表示关联索引0和2的机器人
 */
extern unsigned int GetLinkRobot(int addition_index);


#ifdef __cplusplus
}
}
#endif


#endif /* KINEMATICINTERFACE_H_ */
