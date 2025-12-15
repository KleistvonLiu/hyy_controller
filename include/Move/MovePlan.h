/**
 * @file MovePlan.h
 * 
 * @brief  机器人运动控制接口函数
 * @author hanbing
 * @version 11.3.0
 * @date 2020-03-31
 * 
 */

#ifndef MOVEPLAN_H_
#define MOVEPLAN_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 设置指令线程运行方式，设置后对所有指令起作用
 *
 * @param isthread 1:线程运行，0:阻塞运行
 */
extern void setMoveThread(int isthread);

/**
 * @brief 设置指令线程运行方式，设置后对所有指令起作用
 *
 * @param isthread 1:线程运行，0:阻塞运行
 * @param wait_time_ns 线程同步等待时间(ns)
 */
extern void setMoveThread1(int isthread,uint64_t wait_time_ns);

/**
 * @brief 以线程方式运行指令时，用改指令启动执行，可用于多运动指令同步执行
 *
 * @return 0成功,其他失败
 */
extern int MoveSyncStart();

/**
 * @brief 等待运动结束（开线程运行时有效）
 *
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int wait_move_finish(int robot_index);

/**
 * @brief 等待机器人运动结束
 *
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止
 */
extern int WaitRobotMoveFinish(int robot_index);

/**
 * @brief 等待外部轴组运动结束
 *
 * @param addition_index 外部轴组索引号
 * @return int 0：正常，<0:异常，7：外部轴组被强制停止
 */
extern int WaitAdditionMoveFinish(int addition_index);

/**
 * @brief 等待机器人和外部轴组运动结束
 *
 */
extern void WaitDeviceMoveFinish();

/**
 * @brief 为机器人启用附加轴服务（机器人的运动将受附加轴影响）
 *
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引号
 * @return 0: 成功; 其他: 失败
 */
extern int StartAdditionServer(tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 为机器人关闭附加轴服务
 *
 * @param robot_index 机器人索引号
 * @return 0: 成功; 其他: 失败
 */
extern int StopAdditionServer(int robot_index);

/**
 * @brief 获取机器人运动状态
 *
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_robot_move_state(int robot_index);

/**
 * @brief 设置机器人运动状态
 *
 * @param robot_index 机器人索引号
 * @param state 待设置状态
 * @return int 当前状态(与一致标识设置成功,否则设置不成功) 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int set_robot_move_state(int robot_index,int state);

/**
 * @brief 获取附加轴运动状态
 *
 * @param addition_index 附加轴组索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int get_addition_move_state(int addition_index);

/**
 * @brief 设置附加轴运动状态
 *
 * @param addition_index 附加轴组索引号
 * @param state 待设置状态
 * @return int 当前状态(与一致标识设置成功,否则设置不成功)0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int set_addition_move_state(int addition_index,int state);

/**
 * @brief 清楚机器人运动错误
 *
 * @param robot_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_robot_move_error(int robot_index);

/**
 * @brief 清楚机器人驱动错误
 *
 * @param robot_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_robot_driver_error(int robot_index);

/**
 * @brief 清楚附加轴组运动错误
 *
 * @param addition_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_addition_move_error(int addition_index);

/**
 * @brief 清楚附加轴组驱动错误
 *
 * @param addition_index 机器人索引号
 * @return int 0: 成功; 其他: 失败
 */
extern void clear_addition_driver_error(int addition_index);

/**
 * @brief 清楚所有机器人运动错误
 *
 */
extern void ClearRobotMoveError();

/**
 * @brief 清楚所有机器人驱动错误
 *
 */
extern void ClearRobotDriverError();

/**
 * @brief 清楚所有机器人运动和驱动错误
 *
 */
extern void ClearRobotError();

/**
 * @brief 清楚所有附加轴运动错误
 *
 */
extern void ClearAdditionMoveError();

/**
 * @brief 清楚所有附加轴组驱动错误
 *
 */
extern void ClearAdditionDriverError();

/**
 * @brief 清楚所有附加轴组运动和驱动错误
 *
 */
extern void ClearAdditionError();

/**
 * @brief 清楚所有子设备运动和驱动错误
 *
 */
extern void ClearDeviceError();

/**
 * @brief 休眠
 *
 * @param millisecond 休眠时间（毫秒）
 */
extern void Rsleep(int millisecond);

/**
 * @brief 设置机器人运动速度的百分比（加速度和加加速度）
 *
 * @param acc 加速度（0~1）
 * @param jerk 加加速度（0~1）
 * @param robot_index 机器人索引号
 */
extern void AccSet(double acc, double jerk, int robot_index);

/**
 * @brief 所有子设备上电
 * return int 0:成功;其他：失败
 */
extern int DevicePower(void);

/**
 * @brief 所有子设备停止当前运动并下电(后续运动命令无法执行)
 *
 */
extern void DevicePoweroff(void);

/**
 * @brief 所有子设备停止当前运动(不下电，后续运动命令可以执行)
 *
 */
extern void DeviceStopRun(void);

/**
 * @brief 所有子设备停止当前运动(不下电，后续运动命令不能执行)
 *
 */
extern void DeviceStop(void);

/**
 * @brief 回复因DeviceStop(void)触发的强制停止状态
 *
 */
extern void DeviceStopRecover(void);

/**
 * @brief 所有子设备暂停
 *
 */
extern void DeviceSuspend(void);

/**
 * @brief 所有子设备继续
 *
 */
extern void DeviceContinue(void);

/**
 * @brief 所有子设备使能状态
 * @return int 1:所有轴均使能 0:至少存在一个未使能轴
 */
extern int DevicePowerState(void);

/**
 * @brief 机器人上电
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotPower(int robot_index);

/**
 * @brief 外部轴组上电
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionPower(int addition_index);

/**
 * @brief 机器人下电
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotPoweroff(int robot_index);

/**
 * @brief 外部轴组下电
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionPoweroff(int addition_index);

/**
 * @brief 停止机器人当前运动(不下电，后续运动命令可以执行)
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotStopRun(int robot_index);

/**
 * @brief 停止机器人当前运动(不下电，后续运动命令不能执行)
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotStop(int robot_index);

/**
 * @brief 回复因DeviceStop(void)/RobotStop(int robot_index)触发的强制停止状态
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotStopRecover(int robot_index);

/**
 * @brief 暂停机器人当前运动
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotSuspend(int robot_index);

/**
 * @brief 继续机器人当前运动
 * @param robot_index 机器人索引
 * @return int 0:成功;其他：失败
 */
extern int RobotContinue(int robot_index);

/**
 * @brief 停止外部轴组当前运动(不下电，后续运动命令可以执行)
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionStopRun(int addition_index);

/**
 * @brief 停止外部轴组当前运动(不下电，后续运动命令不能执行)
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionStop(int addition_index);

/**
 * @brief 回复因DeviceStop(void)/AdditionStop(int addition_index)触发的强制停止状态
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionStopRecover(int addition_index);

/**
 * @brief 暂停外部轴组当前运动
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionSuspend(int addition_index);

/**
 * @brief 继续外部轴组当前运动
 * @param addition_index 外部轴组索引
 * @return int 0:成功;其他：失败
 */
extern int AdditionContinue(int addition_index);

/**
 * @brief 机器人使能状态
 * @param robot_index 机器人索引
 * @return int 1:所有轴均使能 0:至少存在一个未使能轴
 */
extern int GetRobotPowerState(int robot_index);

/**
 * @brief 外部轴组使能状态
 * @param addition_index 外部轴组索引
 * @return int 1:所有轴均使能 0:至少存在一个未使能轴
 */
extern int GetAdditionPowerState(int addition_index);

/**
 * @brief 机器人伺服驱动是否存在错误
 * @param robot_index 机器人索引
 * @return int 0:无错误 大于0:错误轴数
 */
extern int IsRobotDriverError(int robot_index);

/**
 * @brief 外部轴组伺服驱动是否存在错误
 * @param addition_index 外部轴组索引
 * @return int 0:无错误 大于0:错误轴数
 */
extern int IsAdditionDriverError(int addition_index);

/**
 * @brief 所有子设备伺服驱动是否存在错误
 * @return int 0:无错误 大于0:错误子设备数
 */
extern int IsDriverError();

/**
 * @brief 机器人是否存在运动状态错误
 * @param robot_index 机器人索引
 * @return int 0:无错误 大于0:错误
 */
extern int IsRobotMoveError(int robot_index);

/**
 * @brief 外部轴组是否存在运动状态错误
 * @param addition_index 外部轴组索引
 * @return int 0:无错误 大于0:错误
 */
extern int IsAdditionMoveError(int addition_index);

/**
 * @brief 所有子设备是否存在运动状态错误
 * @return int 0:无错误 大于0:错误子设备数
 */
extern int IsDeviceMoveError();

/**
 * @brief 所有子设备是否存在运动状态和驱动错误
 * @return int 0:无错误 大于0:动状态或驱动错误
 */
extern int IsDeviceError();

/**
 * @brief 绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 绝对位置运动指令
*
* @param rjoint 期望位置（关节） 单位：rad
* @param rspeed 运动速度   单位：rad/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveA(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);


/**
 * @brief 双臂绝对位置运动指令
 * 
 * @param rjoint1 机器人1的期望位置（关节）  单位：rad
 * @param rjoint2 机器人2的期望位置（关节）   单位：rad
 * @param rspeed1 机器人1的运动速度   单位：rad/s
 * @param rspeed2 机器人2的运动速度   单位：rad/s
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveA(robjoint *rjoint1, robjoint *rjoint2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂绝对位置运动指令
*
* @param rjoint1 机器人1的期望位置（关节）  单位：rad
* @param rjoint2 机器人2的期望位置（关节）   单位：rad
* @param rspeed1 机器人1的运动速度   单位：rad/s
* @param rspeed2 机器人2的运动速度   单位：rad/s
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveA(robjoint* rjoint1,robjoint* rjoint2, speed* rspeed1,speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad
 * @param rspeed 运动速度   单位：rad/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做绝对位置运动指令
*
* @param rjoint 期望位置（关节） 单位：rad
* @param rspeed 运动速度   单位：rad/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveA(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 关节运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行 
 */
extern int moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 关节运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveJ(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂关节运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveJ(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂关节运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveJ(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, zone* rzone1,zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);;

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做绝对位置运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveJ(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 线性运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 线性运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveL(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂线性运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveL(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂线性运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveL(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, zone* rzone1,zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做线性运动指令
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做线性运动指令
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveL(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

/**
* @brief 圆弧运动指令
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveC(robpose* rpose, robpose* rpose_mid, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂圆弧运动指令
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rpose_mid1 机器人1的途经点期望位置（位姿）
 * @param rpose_mid2 机器人2的途经点期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveC(robpose *rpose1, robpose *rpose2, robpose *rpose_mid1, robpose *rpose_mid2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂圆弧运动指令
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rpose_mid1 机器人1的途经点期望位置（位姿）
* @param rpose_mid2 机器人2的途经点期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveC(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做圆弧运动指令
 * 
 * @param rpose 期望位置（位姿）
 * @param rpose_mid 途经点期望位置（位姿）
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做圆弧运动指令
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveC(robpose* rpose, robpose* rpose_mid, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(m)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系 
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 螺旋线运动指令
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(m)
 * @param screw 螺距(m)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 平面螺旋线运动指令
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂螺旋线运动指令
 * 
 * @param rpose1 机器人1的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose2 机器人2的圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid1 机器人1的圆弧中间位置（位姿）（不影响姿态运动）
 * @param rpose_mid2 机器人2的圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line1 机器人1的方向位置（位姿）(决定最终运动姿态)
 * @param pose_line2 机器人2的方向位置（位姿）(决定最终运动姿态)
 * @param screw1 机器人1螺距(m)
 * @param screw2 机器人2螺距(m)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveH(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, robpose* pose_line1,robpose* pose_line2, double screw1,double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
* @brief 双臂螺旋线运动指令
*
* @param rpose1 机器人1的圆弧位置（位姿）（不影响姿态运动）
* @param rpose2 机器人2的圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid1 机器人1的圆弧中间位置（位姿）（不影响姿态运动）
* @param rpose_mid2 机器人2的圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line1 机器人1的方向位置（位姿）(决定最终运动姿态)
* @param pose_line2 机器人2的方向位置（位姿）(决定最终运动姿态)
* @param screw1 机器人1螺距(m)
* @param screw2 机器人2螺距(m)
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveH(robpose* rpose1, robpose* rpose2, robpose* rpose_mid1, robpose* rpose_mid2, robpose* pose_line1,robpose* pose_line2, double screw1,double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 双臂平面螺旋线运动指令
 *
 * @param pose_line1 机器人1的方向位置（位姿）
 * @param pose_line2 机器人2的方向位置（位姿）
 * @param radius1 机器人1半径（m）
 * @param radius2 机器人2半径（m）
 * @param screw1 机器人1螺距(m)
 * @param screw2 机器人2螺距(m)
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rzone1 机器人1的转动空间
 * @param rzone2 机器人2的转动空间
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveHP(robpose* pose_line1, robpose* pose_line2, double radius1, double radius2, double screw1, double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
* @brief 双臂平面螺旋线运动指令
*
* @param pose_line1 机器人1的方向位置（位姿）
* @param pose_line2 机器人2的方向位置（位姿）
* @param radius1 机器人1半径（m）
* @param radius2 机器人2半径（m）
* @param screw1 机器人1螺距(m)
* @param screw2 机器人2螺距(m)
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveHP(robpose* pose_line1, robpose* pose_line2, double radius1, double radius2, double screw1, double screw2,speed* rspeed1, speed* rspeed2, zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做螺旋线运动指令
 * 
 * @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
 * @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
 * @param pose_line 方向位置（位姿）(决定最终运动姿态)
 * @param screw 螺距(m)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系  
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做螺旋线运动指令
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveH(robpose* rpose, robpose* rpose_mid, robpose* pose_line, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 选取多机械臂中的一个做平面螺旋线运动指令
 *
 * @param pose_line 方向位置（位姿）
 * @param radius 半径(m)
 * @param screw 螺距(m)
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做平面螺旋线运动指令
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHP(robpose* pose_line, double radius, double screw, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param filename 轨迹点存放文件名
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveS1(const char* filename, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveS(robpose* rpose, int num, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 双臂关节空间的B样条运动
 * 
 * @param filename1 机器人1的轨迹点存放文件名
 * @param filename2 机器人2的轨迹点存放文件名
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveS(char *filename1, char *filename2, speed *rspeed1, speed *rspeed2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);

/**
* @brief 双臂关节空间的B样条运动
*
* @param filename1 机器人1的轨迹点存放文件名
* @param filename2 机器人2的轨迹点存放文件名
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveS(const char* filename1, const char* filename2, speed* rspeed1, speed* rspeed2,zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
* @brief 双臂关节空间的B样条运动
*
* @param rpose1 机器人1样条必经点
* @param num1 机器人1样条必经点数目
* @param rpose2 机器人2样条必经点
* @param num2 机器人2样条必经点数目
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rzone1 机器人1的转动空间
* @param rzone2 机器人2的转动空间
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveS1(robpose* rpose1,int num1, robpose* rpose2,int num2, speed* rspeed1, speed* rspeed2,zone* rzone1, zone* rzone2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2);

/**
 * @brief 选取多机械臂中的一个做关节空间的B样条运动
 * 
 * @param filename 轨迹点存放文件名
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动
*
* @param filename 轨迹点存放文件名
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveS1(const char* filename, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param rspeed 运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveS(robpose* rpose, int num, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 动态目标笛卡尔空间运动
 * 
 * @param rpose 期望位置（位姿）
 * @param rspeed 运动速度
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj,int stop_cond);

/**
* @brief 动态目标笛卡尔空间运动
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj, int stop_cond);

/**
 * @brief 双臂动态目标笛卡尔空间运动
 * 
 * @param rpose1 机器人1的期望位置（位姿）
 * @param rpose2 机器人2的期望位置（位姿）
 * @param rspeed1 机器人1的运动速度
 * @param rspeed2 机器人2的运动速度
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具
 * @param rwobj1 机器人1的坐标系
 * @param rwobj2 机器人2的坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveD(robpose* rpose1,robpose* rpose2, speed* rspeed1,speed* rspeed2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2,int stop_cond);

/**
* @brief 双臂动态目标笛卡尔空间运动
*
* @param rpose1 机器人1的期望位置（位姿）
* @param rpose2 机器人2的期望位置（位姿）
* @param rspeed1 机器人1的运动速度
* @param rspeed2 机器人2的运动速度
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @param rwobj1 机器人1的坐标系
* @param rwobj2 机器人2的坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveD(robpose* rpose1, robpose* rpose2, speed* rspeed1,speed* rspeed2, tool* rtool1, tool* rtool2, wobj* rwobj1, wobj* rwobj2, int stop_cond);


/**
 * @brief 选取多机械臂中的一个做动态目标笛卡尔空间运动
 * 
 * @param rpose 期望位置（位姿） 
 * @param rspeed 运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveD(robpose* rpose, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int stop_cond,int _index);

/**
* @brief 选取多机械臂中的一个做动态目标笛卡尔空间运动
*
* @param rpose 期望位置（位姿）
* @param rspeed 运动速度
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param stop_cond 到达目标后的状态；0:持续规划；1:到达目标后停止规划
* @param _index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveD(robpose* rpose, speed* rspeed, tool* rtool, wobj* rwobj, int stop_cond,int _index);

/**
 * @brief 选取多机械臂中的一个设置动态目标笛卡尔空间运动的目标
 * 
 * @param rpose 期望位置（位姿） 
 * @param robot_index 机器人索引号
 */
extern void set_multi_moveD_target(robpose* rpose, int robot_index);

/**
* @brief 选取多机械臂中的一个设置动态目标笛卡尔空间运动的目标
*
* @param rpose 期望位置（位姿）
* @param robot_index 机器人索引号
*/
extern void SetMultiMoveDTarget(robpose* rpose, int robot_index);

/**
 * @brief 设置动态目标笛卡尔空间运动的目标
 * 
 * @param rpose 期望位置（位姿） 
 */
extern void set_moveD_target(robpose* rpose);

/**
* @brief 设置动态目标笛卡尔空间运动的目标
*
* @param rpose 期望位置（位姿）
*/
extern void SetMoveDTarget(robpose* rpose);

/**
 * @brief 关节空间多项式规划
 * 
 * @param rtool 末端工具
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int moveAP(tool* rtool);

/**
* @brief 关节空间多项式规划
*
* @param rtool 末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveAP(tool* rtool);

/**
 * @brief 双臂关节空间多项式规划
 * 
 * @param rtool1 机器人1的末端工具
 * @param rtool2 机器人2的末端工具 
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int dual_moveAP(tool* rtool1, tool* rtool2);

/**
* @brief 双臂关节空间多项式规划
*
* @param rtool1 机器人1的末端工具
* @param rtool2 机器人2的末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int DualMoveAP(tool* rtool1, tool* rtool2);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划
 * 
 * @param _index 机器人索引号
 * @param rtool 末端工具
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int multi_moveAP(int _index,tool* rtool);

/**
* @brief 选取多机械臂中的一个做关节空间多项式规划
*
* @param robot_index 机器人索引号
* @param rtool 末端工具
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveAP(int robot_index,tool* rtool);

/**
 * @brief 关节空间多项式规划插入路点
 * 
 * @param time 时间（s）
 * @param joint 关节位置 (rad 或 m)
 * @param velocity 关节速度 (rad/s 或 m/s)
 * @param acceleration 关节加速度（rad/s^2  或 m/s^2）
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int moveAP_push_way_point(double time, double *joint,double* velocity, double* acceleration);

/**
* @brief 关节空间多项式规划插入路点
*
* @param time 时间（s）
* @param joint 关节位置 (rad 或 m)
* @param velocity 关节速度 (rad/s 或 m/s)
* @param acceleration 关节加速度（rad/s^2  或 m/s^2）
* @return int >=0:路点数目; <0:插入路点失败
*/
extern int PushMoveAPWayPoint(double time, double *joint,double* velocity, double* acceleration);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划插入路点
 * @param time 时间（s）
 * @param joint 关节位置 (rad 或 m)
 * @param velocity 关节速度 (rad/s 或 m/s)
 * @param acceleration 关节加速度（rad/s^2  或 m/s^2） 
 * @param _index 机器人索引号
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int multi_moveAP_push_way_point(double time, double *joint,double* velocity, double* acceleration, int _index);

/**
* @brief 选取多机械臂中的一个做关节空间多项式规划插入路点
* @param time 时间（s）
* @param joint 关节位置 (rad 或 m)
* @param velocity 关节速度 (rad/s 或 m/s)
* @param acceleration 关节加速度（rad/s^2  或 m/s^2）
* @param robot_index 机器人索引号
* @return int >=0:路点数目; <0:插入路点失败
*/
extern int MultiPushMoveAPWayPoint(double time, double *joint,double* velocity, double* acceleration, int robot_index);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划删除路点
 * @param robot_index 机器人索引号
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int MultiPopMoveAPWayPoint(int robot_index);

/**
 * @brief 关节空间多项式规划删除路点
 * @return int >=0:路点数目; <0:插入路点失败
 */
extern int PopMoveAPWayPoint(void);

/**
 * @brief 选取多机械臂中的一个做关节空间多项式规划删除所有路点
 * @param robot_index 机器人索引号
 */
extern void MultiEmptyMoveAPWayPoint(int robot_index);

/**
 * @brief 关节空间多项式规划删除所有路点
 */
extern void EmptyMoveAPWayPoint(void);

/**
 * @brief 附加轴绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param _index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveAdd(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj, int _index);

/**
 * @brief 选取多附加轴中的一个做绝对位置运动指令
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveAdd(robjoint* rjoint, speed* rspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令(含附加轴组)
 * 
 * @param rjoint 期望位置（关节） 单位：rad or m
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m
 * @param rspeed 运动速度   单位：rad/s or m/s
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveJoint(robjoint* rjoint,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 选取多机械臂中的一个做绝对位置运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿） 
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m 
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveJoint1(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 选取多机械臂中的一个做线性运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿）
 * @param ajoint 附加轴组期望位置（关节） 单位：rad or m 
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度   单位：rad/s or m/s
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @param robot_index 机器人索引号
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MultiMoveLine(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做圆弧运动指令(含附加轴组)
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 中间位置（位姿）
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveCircle(robpose* rpose,robpose* rpose_mid,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做螺旋线运动指令(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHelical(robpose* rpose,robpose* rpose_mid,robpose* pose_line,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做平面螺旋线运动指令(含附加轴组)
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节） 单位：rad or m
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度   单位：rad/s or m/s
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveHelicalPlane(robpose* pose_line,double radius,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 选取多机械臂中的一个做关节空间的B样条运动(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param num 数据数目
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MultiMoveBSpline(robpose* rpose,int num,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj, int robot_index);

/**
 * @brief 绝对位置运动指令(含附加轴组)
 * 
 * @param rjoint 期望位置（关节）
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveJoint(robjoint* rjoint,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 绝对位置运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿） 
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveJoint1(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
 * @brief 线性运动指令(含附加轴组)
 * 
 * @param rpose 期望位置（位姿）
 * @param ajoint 附加轴组期望位置（关节）
 * @param rspeed 运动速度
 * @param aspeed 附加轴组运动速度
 * @param rzone 转动空间
 * @param rtool 末端工具
 * @param rwobj 工件坐标系
 * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
 */
extern int MoveLine(robpose* rpose,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 圆弧运动指令(含附加轴组)
*
* @param rpose 期望位置（位姿）
* @param rpose_mid 途经点期望位置（位姿）
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveCircle(robpose* rpose,robpose* rpose_mid,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 螺旋线运动指令(含附加轴组)
*
* @param rpose 旋转圆弧位置（位姿）（不影响姿态运动）
* @param rpose_mid 旋转圆弧中间位置（位姿）（不影响姿态运动）
* @param pose_line 方向位置（位姿）(决定最终运动姿态)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHelical(robpose* rpose,robpose* rpose_mid,robpose* pose_line,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 平面螺旋线运动指令(含附加轴组)
*
* @param pose_line 方向位置（位姿）
* @param radius 半径(m)
* @param screw 螺距(m)
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveHelicalPlane(robpose* pose_line,double radius,double screw,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 关节空间的B样条运动
*
* @param rpose 样条必经点
* @param num 样条必经点数目
* @param ajoint 附加轴组期望位置（关节）
* @param rspeed 运动速度
* @param aspeed 附加轴组运动速度
* @param rzone 转动空间
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
extern int MoveBSpline(robpose* rpose,int num,robjoint* ajoint, speed* rspeed, speed* aspeed, zone* rzone, tool* rtool, wobj* rwobj);

/**
* @brief 关节速度指令
*
* @param velocity 关节运动速度向量   单位：rad/s
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
int MultiSpeedA(double* velocity,tool *rtool, wobj *rwobj,int robot_index);

/**
* @brief 关节速度指令
*
* @param velocity 关节运动速度向量   单位：rad/s
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
int SpeedA(double* velocity,tool *rtool, wobj *rwobj);

/**
* @brief 笛卡尔速度指令
*
* @param velocity 笛卡尔速度速度向量   单位：rad/s
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @param robot_index 机器人索引号
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
int MultiSpeedL(double* velocity, tool* rtool, wobj* rwobj, int robot_index);

/**
* @brief 笛卡尔速度指令
*
* @param velocity 笛卡尔速度速度向量   单位：rad/s
* @param rtool 末端工具
* @param rwobj 工件坐标系
* @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
*/
int SpeedL(double* velocity, tool* rtool, wobj* rwobj);


/**
 * @brief 机器人位姿绝对工件偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(m)
 * @param y y方向补偿值(m)
 * @param z z方向补偿值(m)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose Offs(const robpose* rpose, double x, double y, double z, double k, double p, double s);

/**
 * @brief 机器人位姿相对工具偏移补偿
 *
 * @param rpose 机器人初始位姿
 * @param x x方向补偿值(m)
 * @param y y方向补偿值(m)
 * @param z z方向补偿值(m)
 * @param k 姿态变量1补偿值(rad)
 * @param p 姿态变量2补偿值(rad)
 * @param s 姿态变量3补偿值(rad)
 * @return robpose 机器人补偿后位姿
 */
extern robpose OffsRel(const robpose* rpose, double x, double y, double z, double k, double p, double s);

/**
 * @brief 获取机器人当前关节角
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void robot_getJoint(double* joint, int _index);

/**
* @brief 获取机器人当前关节角
*
* @param joint 当前关节角数据(rad/m)
* @param _index 机器人索引号
*/
extern void GetCurrentJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前目标关节角(可能读取当期周期写入的期望)
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void GetCurrentTargetJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前目标关节角(上一个周期的期望数据)
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void GetCurrentLastTargetJoint(double* joint, int _index);

/**
 * @brief 获取附加轴组当前关节角
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void GetCurrentAdditionJoint(double* joint, int _index);

/**
 * @brief 获取附加轴组当前目标关节角(可能读取当期周期写入的期望)
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void GetCurrentTargetAdditionJoint(double* joint, int _index);

/**
 * @brief 获取附加轴组当前目标关节角(上一个周期的期望数据)
 * 
 * @param joint 当前关节角数据(rad/m)
 * @param _index 机器人索引号
 */
extern void GetCurrentLastTargetAdditionJoint(double* joint, int _index);

/**
 * @brief 获取机器人当前位姿
 * 
 * @param rtool 工具
 * @param rwobj 工件
 * @param pospose 当前位姿数据(m,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int robot_getCartesian(tool* rtool,wobj* rwobj, double* pospose, int _index);

/**
* @brief 获取机器人当前位姿
*
* @param rtool 工具
* @param rwobj 工件
* @param pospose 当前位姿数据(m,rad)
* @param _index 机器人索引号
* @return int 0:成功；其他失败
*/
extern int GetCurrentCartesian(tool* rtool,wobj* rwobj, robpose* pospose, int _index);

/**
 * @brief 获取机器人当前目标位姿(可能读取当期周期写入的期望)
 * 
 * @param rtool 工具
 * @param rwobj 工件
 * @param pospose 当前位姿数据(m,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentTargetCartesian(tool* rtool,wobj* rwobj, robpose* pospose, int _index);

/**
 * @brief 获取机器人当前目标位姿(上一个周期的期望数据)
 * 
 * @param rtool 工具
 * @param rwobj 工件
 * @param pospose 当前位姿数据(m,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentLastTargetCartesian(tool* rtool,wobj* rwobj, robpose* pospose, int _index);

/**
* @brief 获取附加轴组当前位姿
*
* @param pospose 当前位姿数据(m,rad)
* @param _index 机器人索引号
* @return int 0:成功；其他失败
*/
extern int GetCurrentAdditionCartesian(robpose* pospose, int _index);

/**
 * @brief 获取附加轴组当前目标位姿(可能读取当期周期写入的期望)
 * 
 * @param pospose 当前目标位姿数据(m,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentAdditionTargetCartesian(robpose* pospose, int _index);

/**
 * @brief 获取附加轴组当前目标位姿(上一个周期的期望数据)
 * 
 * @param pospose 当前目标位姿数据(m,rad)
 * @param _index 机器人索引号
 * @return int 0:成功；其他失败
 */
extern int GetCurrentAdditionLastTargetCartesian(robpose* pospose, int _index);


/**
 * @brief 获取机器人移动速度系数（0~1）
 * 
 * @param robot_index 机器人索引
 * @return double 速度系数（0~1），如果小于1，机器人运动速度将低于设定速度运行
 */
double GetRobotMoveSpeedCoeff(int robot_index);

/**
 * @brief 设置机器人移动速度系数（0~1），动态生效
 * 
 * @param robot_index 机器人索引
 * @param value 速度系数（0~1）
 * @return int 0:正确，其他错误
 */
int SetRobotMoveSpeedCoeff(double value, int robot_index);

/**
 * @brief 开启伺服功能(根据伺服数据噪声程度选择q,r)
 * @param q 过程噪声,越小越平滑，响应越慢;越大越贴近伺服数据,易受噪声影响.
 * @param r 测量噪声,越小越近伺服数据,易受噪声影响; 越大越平滑，响应越慢;
 * @param robot_index 机器人索引
 * @return int 0:正确，其他错误
 */
extern int ServoStart(double q,double r, int robot_index);

/**
 * @brief 关节伺服，要求开启伺服功能
 * 
 * @param joint 关节目标数据
 * @param time 数据时间
 * @param robot_index 机器人索引
 * @return int 0:正确，其他错误
 */
extern int ServoJoint(robjoint* joint, double time, int robot_index);

/**
 * @brief 关节伺服，要求开启伺服功能
 * 
 * @param pose 笛卡尔位姿目标目标数据
 * @param time 数据时间
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引
 * @return int 0:正确，其他错误
 */
extern int ServoCartesian(robpose* pose, double time,tool* rtool,wobj* rwobj, int robot_index);

/**
 * @brief 关闭伺服功能
 * 
 * @param robot_index 机器人索引
 * @return int 0:正确，其他错误
 */
extern int ServoEnd(int robot_index);

/**
 * @brief 设置数字量输出
 * 
 * @param id 数字量输出索引号
 * @param flag 0或1
 */
extern void SetDo(int id, int flag);

/**
 * @brief 获取数字量输入
 * 
 * @param id 数字量输入索引号
 * @return int 返回值0或1
 */
extern int GetDi(int id);

/**
 * @brief 获取数字量输出
 * 
 * @param id 数字量输出索引号
 * @return int 返回值0或1
 */
extern int GetDo(int id);

/**
 * @brief 等待数字量输入赋值
 * 
 * @param id 数字量输入索引号
 * @param value 触发信号0或1
 */
extern void WaitDi(int id, int value);

/**
 * @brief 设置模拟量输出
 * 
 * @param id 模拟量输出索引号
 * @param flag 模拟量
 */
extern void SetAo(int id, double flag);

/**
 * @brief 获取模拟量输入
 * 
 * @param id 模拟量输入索引号
 * @return double 模拟量
 */
extern double GetAi(int id);



/**
 * @brief 获取机器人自由度
 * 
 * @param _index 机器人索引号
 * @return int 返回机器人自由度
 */
extern int robot_getDOF(int _index);

/**
 * @brief 获取机器人附加轴数量
 * 
 * @param _index 机器人附加轴分组索引
 * @return int 返回附加轴数量 
 */
extern int additionaxis_getDOF(int _index);

/**
 * @brief 获取机器人数量
 * 
 * @return int 返回机器人数量
 */
extern int robot_getNUM();

/**
 * @brief 获取附加轴组数量
 *
 * @return int 返回附加轴组数量
 */
extern int additionaxis_getNUM();

/**
 * @brief 获取整型寄存器数量
 *
 * @return int 返回整型寄存器数量
 */
extern int GetIntRegisterNum();

/**
 * @brief 设置整型寄存器
 * 
 * @param value 寄存器数据
 * @param index 寄存器索引0,1,2...
 * 
 * @return int 0:正确，其他错误
 */
extern int SetIntRegister(int value, int index);

/**
 * @brief 获取整型寄存器
 * 
 * @param index 寄存器索引0,1,2...
 * 
 * @return int 寄存器数据
 */
extern int GetIntRegister(int index);

/**
 * @brief 获取所有整型寄存器
 * 
 * @param values 返回寄存器数据
 */
extern void GetIntRegisters(int* values);

/**
 * @brief 获取浮点寄存器数量
 *
 * @return int 返回浮点寄存器数量
 */
extern int GetDoubleRegisterNum();

/**
 * @brief 设置浮点寄存器
 * 
 * @param value 寄存器数据
 * @param index 寄存器索引0,1,2...
 * 
 * @return int 0:正确，其他错误
 */
extern int SetDoubleRegister(double value, int index);

/**
 * @brief 获取浮点寄存器
 * 
 * @param index 寄存器索引0,1,2...
 * 
 * @return double 寄存器数据
 */
extern double GetDoubleRegister(int index);

/**
 * @brief 获取所有浮点寄存器
 * 
 * @param values 返回寄存器数据
 */
extern void GetDoubleRegisters(double* values);

//----------------------------------------------------------------------------------------thread interface--------------------------------------------------------------------------
/**
 * @brief 创建线程
 * 
 * @param fun 线程回调执行函数
 * @param arg 执行参数
 * @param name 线程名
 * @param detached_flag 线程属性标识thread attribute,0:PTHREAD_CREATE_JOINABLE  other:PTHREAD_CREATE_DETACHED
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadCreat(void *(*fun)(void *), void *arg, const char *name, int detached_flag);

/**
 * @brief 线程数据释放
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadDataFree(const char *name);


/**
 * @brief thread join (当线程属性为PTHREAD_CREATE_JOINABLE时有效)
 * 
 * @param name 线程名
 * @return int 成功返回0，失败返回其他
 */
extern int ThreadWait(const char *name);


#ifdef __cplusplus
}
}
#endif


#endif /* MOVEPLAN_H_ */
