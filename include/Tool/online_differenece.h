/**
 * @file online_differenece.h
 *
 * @brief  在线微分
 * @author hanbing
 * @version 11.4.6
 * @date 2023-11-08
 *
 */

#ifndef ONLINE_DIFFERENTIAL_H_
#define ONLINE_DIFFERENTIAL_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

#define CENTRALDIFFERENCENUM 10 //!< 微分向量最大维度

/*-------------------------------------------------------------------------*/
/**
  @brief 在线微分数据
 */
/*-------------------------------------------------------------------------*/
typedef struct CentralDifference{
	double Ts; ///< 采样周期
	double value_1; ///< 数据记录
	double value_2; ///< 数据记录
	int one_flag; ///< 内部标识位
}CentralDifference;

/*-------------------------------------------------------------------------*/
/**
  @brief 在线微分向量数据
 */
/*-------------------------------------------------------------------------*/
typedef struct CentralDifferences{
	CentralDifference cd[CENTRALDIFFERENCENUM]; ///< 向量微分
}CentralDifferences;

/**
* @brief 初始化在线微分数据
*
* @param cd 微分数据
* @param Ts 采样周期
*/
extern void init_central_difference_online( CentralDifference* cd, double Ts);

/**
* @brief 在线微分数据
*
* @param cd 微分数据
* @param value 待微分数据
* @return double 微分后数据
*/
extern double central_difference_online( CentralDifference* cd, double value);

/**
* @brief 初始化在线微分向量数据
*
* @param cds 微分数据
* @param Ts 采样周期
* @param n 可用的最大向量维度
*/
extern void init_central_difference_onlines( CentralDifferences* cds, double Ts, int n);

/**
* @brief 在线微分向量数据
*
* @param cds 微分数据
* @param value_in 待微分向量数据
* @param value_out 微分后向量数据
* @param n 微分向量的维度(不能超过初始化设定值)
*/
extern void central_difference_onlines( CentralDifferences* cds, double* value_in, double* value_out, int n);

/**
* @brief 离线中值微分
*
* @param in 输入数据向量
* @param out 输出数据向量
* @param n 数据维度
* @param ts 数据采样时间
*/
extern void central_difference( double in[], double out[], int n, double ts);

#ifdef __cplusplus
}
}
#endif


#endif /*ONLINE_DIFFERENTIAL_H_*/