/**
 * @file tarjectory_interface.h
 *
 * @brief  激励轨迹接口
 * @author hanbing
 * @version 11.4.6
 * @date 2023-11-08
 *
 */

#ifndef TARJECTORY_INTERFACE_H_
#define TARJECTORY_INTERFACE_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

#define TARJECTORY_PARAMETER_MAX_NUM 100 //!< 轨迹参数最大支持数目

/*-------------------------------------------------------------------------*/
/**
  @brief	激励轨迹数据结构
 */
/*-------------------------------------------------------------------------*/
typedef struct TarjectoryParam{
	double f; ///< 轨迹基频
	double param[TARJECTORY_PARAMETER_MAX_NUM]; ///< 轨迹参数,ai1,bi1,......,ain,bin,qi0, a(i+1)1,b(i+1)1,......
	int order; ///< 轨迹阶次
	int dof; ///< 轨迹维度
}TarjectoryParam;

/**
* @brief 初始化轨迹
*
* @param tarj 轨迹数据
* @param f 轨迹基频
* @param param 轨迹参数,ai1,bi1,......,ain,bin,qi0, a(i+1)1,b(i+1)1,......
* @param order 轨迹阶次
* @param dof 轨迹维度
*/
extern void initTarjectoryParam(TarjectoryParam* tarj, double f, const double* param, int order, int dof);

/**
* @brief 获取轨迹
*
* @param tarj 轨迹数据
* @param time 采样时刻
* @param q 轨迹位置
* @param qd 轨迹速度
* @param qdd 轨迹加速度
* @return int 轨迹参数指针
*/
extern int getTarjectoryData(TarjectoryParam* tarj, double time, double* q, double* qd,double* qdd);

/**
* @brief 获取轨迹(多条轨迹拼接而成的总轨迹)
*
* @param tarj 轨迹数据向量
* @param n 轨迹数据向量维度
* @param time 采样时刻
* @param q 轨迹位置
* @param qd 轨迹速度
* @param qdd 轨迹加速度
* @return int 轨迹参数指针
*/
extern int getTarjectoryData_n(TarjectoryParam* tarj, int n, double time, double* q, double* qd,double* qdd);

/**
* @brief 从控制器中获取轨迹参数
*
* @param trajectory_index 轨迹索引
* @param n 轨迹参数数目
* @param name 轨迹名称,于对应机器人名称一致
* @param tarjectory_parameter 返回的轨迹参数，为NULL将使用内部数区存放参数变量
* @return void* 轨迹参数指针
*/
extern void* get_tarjectory_parameter(int trajectory_index,int n,const char* name,double* tarjectory_parameter);

/**
* @brief 从控制器中获取负载辨识的轨迹参数
*
* @param trajectory_index 轨迹索引
* @param n 轨迹参数数目
* @param name 轨迹名称,于对应机器人名称一致
* @param tarjectory_parameter 返回的轨迹参数，为NULL将使用内部数区存放参数变量
* @return void* 轨迹参数指针
*/
extern void* get_payload_tarjectory_parameter(int trajectory_index,int n,const char* name,double* tarjectory_parameter);


#ifdef __cplusplus
}
}
#endif


#endif /*TARJECTORY_INTERFACE_H_*/

