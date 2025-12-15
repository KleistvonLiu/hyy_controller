/**
 * @file Trajectory.h
 * 
 * @brief 轨迹规划
 * @author HanBing
 * @version 12.0.0
 * @date 2024-03-18
 * 
 */
#ifndef INCLUDE_TRAJECTORY_H_
#define INCLUDE_TRAJECTORY_H_
#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

typedef struct JointTrajectory JointTrajectory;///<关节轨迹数据结构
typedef struct LineTrajectory LineTrajectory;///<笛卡尔直线轨迹数据结构
typedef struct CircleTrajectory CircleTrajectory;///<笛卡尔圆弧轨迹数据结构
typedef struct BSplineTrajectory BSplineTrajectory;///<笛卡尔B样条轨迹数据结构
typedef struct PloynomialTrajectory PloynomialTrajectory;///<多项式轨迹数据结构

/**
* @brief 创建关节轨迹
*
* @param start 关节起始点向量
* @param end 关节终点向量
* @param vmax 关节速度向量
* @param amax 关节加速度向量
* @param jmax 关节加加速度向量
* @param dof 关节向量维度
* @param duration 规划时间(<=0:按照最大速度规划;>0:按照指定规划时间规划)
* @param errorcode 规划错误代码(失败时返回负数,-1:内存分别配失败;-2:路径规划失败;-3:多轴同步失败;-4:速度规划失败)

* @return JointTrajectory 返回规划数据指针,失败时为NULL(需要调用释放接口主动释放资源)
*/
extern JointTrajectory* CreateJointTrajectory(double* start, double* end, double* vmax, double* amax, double* jmax,int dof,double duration,int* errorcode);

/**
* @brief 获取关节轨迹
*
* @param tarj 轨迹数据
* @param time 时间(从0开始，到最大规划时间)
* @param position 关节位置向量
* @param velocity 关节速度向量
* @param acceleration 关节加速度向量
* @param jeck 关节加加速度向量

* @return int 0:轨迹结束; >0：轨迹未结束
*/
extern int GetJointTrajectory(JointTrajectory* tarj,double time, double* position, double* velocity, double*acceleration, double* jeck);

/**
* @brief 释放关节轨迹资源
*
* @param tarj 轨迹数据
*/
extern void FreeJointTrajectory(JointTrajectory* tarj);

/**
* @brief 创建笛卡尔直线轨迹
*
* @param start 笛卡尔起始点位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param end 笛卡尔终点位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param pvmax 笛卡尔位置速度向量
* @param pamax 笛卡尔位置加速度向量
* @param pjmax 笛卡尔位置加加速度向量
* @param qvmax 笛卡尔姿态速度向量
* @param qamax 笛卡尔姿态加速度向量
* @param qjmax 笛卡尔姿态加加速度向量
* @param duration 规划时间(<=0:按照最大速度规划;>0:按照指定规划时间规划)
* @param errorcode 规划错误代码(失败时返回负数,-1:内存分别配失败;-2:路径规划失败;-3:多轴同步失败;-4:速度规划失败)

* @return LineTrajectory 返回规划数据指针,失败时为NULL(需要调用释放接口主动释放资源)
*/
extern LineTrajectory* CreateLineTrajectory(double start[7], double end[7], double pvmax, double pamax, double pjmax,
double qvmax, double qamax, double qjmax,double duration,int* errorcode);

/**
* @brief 获取笛卡尔直线轨迹
*
* @param tarj 轨迹数据
* @param time 时间(从0开始，到最大规划时间)
* @param position 位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param velocity 位姿速度向量
* @param acceleration 位姿加速度向量
* @param jeck 位姿加加速度向量

* @return int 0:轨迹结束; >0：轨迹未结束
*/
extern int GetLineTrajectory(LineTrajectory* tarj,double time, double position[7], double velocity[6], double acceleration[6], double jeck[6]);

/**
* @brief 释放笛卡尔直线轨迹资源
*
* @param tarj 轨迹数据
*/
extern void FreeLineTrajectory(LineTrajectory* tarj);

/**
* @brief 创建笛卡尔圆弧轨迹
*
* @param start 笛卡尔起始点位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param middle 笛卡尔中间的位置向量(x,y,z)(位置)
* @param end 笛卡尔终点位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param scale 圆弧完整度(0<=0:运动到指定终点;1<：不完整圆；=1：整圆；1>：一圈以上的圆;)
* @param pvmax 笛卡尔位置速度向量
* @param pamax 笛卡尔位置加速度向量
* @param pjmax 笛卡尔位置加加速度向量
* @param qvmax 笛卡尔姿态速度向量
* @param qamax 笛卡尔姿态加速度向量
* @param qjmax 笛卡尔姿态加加速度向量
* @param duration 规划时间(<=0:按照最大速度规划;>0:按照指定规划时间规划)
* @param errorcode 规划错误代码(失败时返回负数,-1:内存分别配失败;-2:路径规划失败;-3:多轴同步失败;-4:速度规划失败)

* @return CircleTrajectory 返回规划数据指针,失败时为NULL(需要调用释放接口主动释放资源)
*/
extern CircleTrajectory* CreateCircleTrajectory(double start[7],double middle[3], double end[7], double scale, double pvmax, double pamax, double pjmax,
double qvmax, double qamax, double qjmax,double duration,int* errorcode);

/**
* @brief 获取笛卡尔圆弧轨迹
*
* @param tarj 轨迹数据
* @param time 时间(从0开始，到最大规划时间)
* @param position 位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param velocity 位姿速度向量
* @param acceleration 位姿加速度向量
* @param jeck 位姿加加速度向量

* @return int 0:轨迹结束; >0：轨迹未结束
*/
extern int GetCircleTrajectory(CircleTrajectory* tarj,double time, double position[7], double velocity[6], double acceleration[6], double jeck[6]);

/**
* @brief 释放笛卡尔圆弧轨迹资源
*
* @param tarj 轨迹数据
*/
extern void FreeCircleTrajectory(CircleTrajectory* tarj);

/**
* @brief 创建笛卡尔B样条轨迹
*
* @param pwaypoint 笛卡尔位置路点(x,y,z)
* @param qwaypoint 笛卡尔单位四元数姿态路点(qx,qy,qz,qw)
* @param n 路点数目
* @param pvmax 笛卡尔位置速度向量
* @param pamax 笛卡尔位置加速度向量
* @param pjmax 笛卡尔位置加加速度向量
* @param qvmax 笛卡尔姿态速度向量
* @param qamax 笛卡尔姿态加速度向量
* @param qjmax 笛卡尔姿态加加速度向量
* @param duration 规划时间(<=0:按照最大速度规划;>0:按照指定规划时间规划)
* @param errorcode 规划错误代码(失败时返回负数,-1:内存分别配失败;-2:路径规划失败;-3:多轴同步失败;-4:速度规划失败)

* @return BSplineTrajectory 返回规划数据指针,失败时为NULL(需要调用释放接口主动释放资源)
*/
extern BSplineTrajectory* CreateBSplineTrajectory(double pwaypoint[][3], double qwaypoint[][4], int n, double pvmax, double pamax, double pjmax,
double qvmax, double qamax, double qjmax,double duration,int* errorcode);

/**
* @brief 获取笛卡尔B样条轨迹
*
* @param tarj 轨迹数据
* @param time 时间(从0开始，到最大规划时间)
* @param position 位姿向量(x,y,z,qx,qy,qz,qw)(位置及单位四元数姿态)
* @param velocity 位姿速度向量
* @param acceleration 位姿加速度向量
* @param jeck 位姿加加速度向量

* @return int 0:轨迹结束; >0：轨迹未结束
*/
extern int GetBSplineTrajectory(BSplineTrajectory* tarj,double time, double position[7], double velocity[6], double acceleration[6], double jeck[6]);

/**
* @brief 释放笛卡尔B样条轨迹资源
*
* @param tarj 轨迹数据
*/
extern void FreeBSplineTrajectory(BSplineTrajectory* tarj);

/**
* @brief 创建5阶多项式轨迹
*
* @param spoition 起点位置向量
* @param svelicity 起点速度向量
* @param sacceleration 起点加速度向量
* @param stime 起点时间
* @param epoition 终点位置向量
* @param evelicity 终点速度向量
* @param eacceleration 终点加速度向量
* @param etime 终点时间
* @param n 向量维度
* @param errorcode 规划错误代码(失败时返回负数,-1:内存分别配失败;-2:轨迹规划失败;)

* @return PloynomialTrajectory 返回规划数据指针,失败时为NULL(需要调用释放接口主动释放资源)
*/
extern PloynomialTrajectory* CreatePloynomialTrajectory(double* spoition, double* svelicity, double* sacceleration, double stime,double* epoition, double* evelicity, double* eacceleration, double etime, int n,int* errorcode);

/**
* @brief 获取5阶多项式轨迹
*
* @param tarj 轨迹数据
* @param time 时间(从0开始，到最大规划时间)
* @param position 位置向量
* @param velocity 速度向量
* @param acceleration 加速度向量
* @param jeck 加加速度向量

* @return int 0:轨迹结束; >0：轨迹未结束
*/
extern int GetPloynomialTrajectory(PloynomialTrajectory* tarj,double time, double* position, double* velocity, double* acceleration, double* jeck);

/**
* @brief 释放5阶多项式轨迹资源
*
* @param tarj 轨迹数据
*/
extern void FreePloynomialTrajectory(PloynomialTrajectory* tarj);




#ifdef __cplusplus
}
}
#endif

#endif /* INCLUDE_TRAJECTORY_H_ */