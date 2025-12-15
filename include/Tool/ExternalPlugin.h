/**
 * @file ExternalPlugin.h
 *
 * @brief  姿态描述转换接口
 * @author hanbing
 * @version 12.2.1
 * @date 2025-11-07
 *
 */

#ifndef EXTERNALPLUGIN_H_
#define EXTERNALPLUGIN_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 加载外部库
 * @param libname 库名称，非系统路径下需要包含路径
 * @return int 0:成功; 其他：失败
 */
extern int RobotExternalPluginUpload(const char* libname);

/**
 * @brief 卸载外部库
 */
extern void RobotExternalPluginUnload();

/**
 * @brief 外部功能开启
 * @param name 名称（确保唯一）
 * @param arg 输入参数(注：使用时与函数内自定义实现用的数据类型保持一致)
 * @return int 返回错误代码(错误代码自行定义)
 */
extern int RobotExternalPluginOpen(const char* name, void* arg);

/**
 * @brief 外部功能操作
 * @param name 名称(与接口保持一致)
 * @param value 输入参数(注：使用时与函数内自定义实现用的数据类型保持一致)
 * @return int 返回错误代码(错误代码自行定义)
 */
extern int RobotExternalPluginFunction(const char* name, void* value);

/**
 * @brief 外部功能关闭(与接口保持一致)
 * @param name 名称(与接口保持一致)
 * @return int 返回错误代码(错误代码自行定义)
 */
extern int RobotExternalPluginClose(const char* name);

#ifdef __cplusplus
}
}
#endif


#endif /* EXTERNALPLUGIN_H_ */