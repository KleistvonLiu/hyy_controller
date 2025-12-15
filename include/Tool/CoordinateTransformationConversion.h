/**
 * @file CoordinateTransformationConversion.h
 *
 * @brief  姿态描述转换接口
 * @author hanbing
 * @version 12.0.0
 * @date 2024-03-04
 *
 */

#ifndef COORDINATETRANSFORMATIONCONVERSION_H_
#define COORDINATETRANSFORMATIONCONVERSION_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
* @brief 旋转矩阵转ZYX欧拉角
*
* @param r 旋转矩阵
* @param eulerZ 绕Z轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerX 绕X轴旋转角(rad)
*/
void Rotation2EulerZYX(const double r[9],double* eulerZ, double* eulerY,double* eulerX);

/**
* @brief ZYX欧拉角转旋转矩阵
*
* @param eulerZ 绕Z轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerX 绕X轴旋转角(rad)
* @param r 旋转矩阵
*/
void EulerZYX2Rotation(const double eulerZ, const double eulerY, const double eulerX,double r[9]);

/**
* @brief 旋转矩阵转XYZ欧拉角
*
* @param r 旋转矩阵
* @param eulerX 绕X轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerZ 绕Z轴旋转角(rad)
*/
void Rotation2EulerXYZ(const double r[9], double* eulerX, double* eulerY, double* eulerZ);

/**
* @brief XYZ欧拉角转旋转矩阵
*
* @param eulerX 绕X轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerZ 绕Z轴旋转角(rad)
* @param r 旋转矩阵
*/
void EulerXYZ2Rotation(const double eulerX, const double eulerY,const double eulerZ,double r[9]);

/**
* @brief 旋转矩阵转ZYZ欧拉角
*
* @param r 旋转矩阵
* @param alpha 绕Z轴旋转角(rad)
* @param beta 绕Y轴旋转角(rad)
* @param gamma 绕Z轴旋转角(rad)
*/
void Rotation2EulerZYZ(const double r[9],double* alpha,double* beta,double* gamma);

/**
* @brief ZYZ欧拉角转旋转矩阵
*
* @param alpha 绕Z轴旋转角(rad)
* @param beta 绕Y轴旋转角(rad)
* @param gamma 绕Z轴旋转角(rad)
* @param r 旋转矩阵
*/
void EulerZYZ2Rotation(const double alpha,const double beta,const double gamma,double r[9]);

/**
* @brief 旋转矩阵转XYZ固定角
*
* @param r 旋转矩阵
* @param roll 绕X轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param yaw 绕Z轴旋转角(rad)
*/
void Rotation2RPY(const double r[9],double* roll, double* pitch, double* yaw);

/**
* @brief XYZ固定角转旋转矩阵
*
* @param roll 绕X轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param yaw 绕Z轴旋转角(rad)
* @param r 旋转矩阵
*/
void RPY2Rotation(const double roll, const double pitch, const double yaw,double r[9]);

/**
* @brief 旋转矩阵转ZYX固定角
*
* @param r 旋转矩阵
* @param yaw 绕Z轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param roll 绕X轴旋转角(rad)
*/
void Rotation2YPR(const double r[9],double* yaw, double* pitch, double* roll);

/**
* @brief ZYX固定角转旋转矩阵
*
* @param yaw 绕Z轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param roll 绕X轴旋转角(rad)
* @param r 旋转矩阵
*/
void YPR2Rotation(const double yaw, const double pitch, const double roll,double r[9]);

/**
* @brief 旋转矩阵转轴角
*
* @param r 旋转矩阵
* @param axis 旋转轴
* @param angle 旋转角(rad)
*/
void Rotation2AxisAngle(const double r[9],double axis[3], double* angle);

/**
* @brief 轴角转旋转矩阵
*
* @param axis 旋转轴
* @param angle 旋转角(rad)
* @param r 旋转矩阵
*/
void AxisAngle2Rotation(const double axis[3], const double angle,double r[9]);

/**
* @brief 旋转矩阵转四元数
*
* @param r 旋转矩阵
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void Rotation2Quaternion(const double r[9],double* x,double* y,double* z,double* w);

/**
* @brief 四元数转旋转矩阵
*
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
* @param r 旋转矩阵
*/
void Quaternion2Rotation(const double x,const double y,const double z,const double w,double r[9]);

/**
* @brief XYZ固定角转四元数
*
* @param roll 绕X轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param yaw 绕Z轴旋转角(rad)
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void RPY2Quaternion(const double roll, const double pitch, const double yaw,double* x,double* y,double* z,double* w);

/**
* @brief 四元数转XYZ固定角
*
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
* @param roll 绕X轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param yaw 绕Z轴旋转角(rad)
*/
void Quaternion2RPY(const double x,const double y,const double z,const double w,double* roll, double* pitch, double* yaw);

/**
* @brief ZYX欧拉角转四元数
*
* @param eulerZ 绕Z轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerX 绕X轴旋转角(rad)
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void EulerZYX2Quaternion(const double eulerZ, const double eulerY,const double eulerX,double* x,double* y,double* z,double* w);

/**
* @brief 四元数转ZYX欧拉角
*
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
* @param eulerZ 绕Z轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerX 绕X轴旋转角(rad)
*/
void Quaternion2EulerZYX(const double x,const double y,const double z,const double w,double* eulerZ, double* eulerY,double* eulerX);

/**
* @brief XYZ欧拉角转四元数
*
* @param eulerX 绕X轴旋转角(rad)
* @param eulerY 绕Y轴旋转角(rad)
* @param eulerZ 绕Z轴旋转角(rad)
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void EulerXYZ2Quaternion(const double eulerX, const double eulerY,const double eulerZ,double* x,double* y,double* z,double* w);

/**
* @brief ZYX固定角转四元数
*
* @param yaw 绕Z轴旋转角(rad)
* @param pitch 绕Y轴旋转角(rad)
* @param roll 绕X轴旋转角(rad)
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void YPR2Quaternion(const double yaw, const double pitch, const double roll,double* x,double* y,double* z,double* w);

/**
* @brief 轴角转四元数
*
* @param axis 旋转轴
* @param angle 旋转角(rad)
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
*/
void AxisAngle2Quaternion(const double axis[3], const double angle,double* x,double* y,double* z,double* w);

/**
* @brief 四元数转轴角
*
* @param x 四元数向量元素
* @param y 四元数向量元素
* @param z 四元数向量元素
* @param w 四元数系数元素
* @param axis 旋转轴
* @param angle 旋转角(rad)
*/
void Quaternion2AxisAngle(const double x,const double y,const double z,const double w,double axis[3], double* angle);





#ifdef __cplusplus
}
}
#endif


#endif /* COORDINATETRANSFORMATIONCONVERSION_H_ */
