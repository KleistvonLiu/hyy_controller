/**
 * @file RobotSystem.h
 *
 * @brief  机器人控制系统初始化必要接口
 * @author hanbing
 * @version 11.4.1
 * @date 2020-04-08
 *
 */

#ifndef ROBOTSYSTEM_H_
#define ROBOTSYSTEM_H_
/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"
#include <stdarg.h>

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 程序优先级设置
 * @param priority  优先级
 *
 * @return int 0:正确，错误返回其他
 */
extern int initPriority(int priority);

/**
 * @brief 线程优先级设置
 * @param priority  优先级
 *
 * @return int 0:正确，错误返回其他
 */
extern int initCurrentThreadPriority(int priority);

/**
 * @brief 线程名称设置
 * @param name  线程名
 *
 * @return int 0:正确，错误返回其他
 */
extern int setCurrentThreadName(char* name);

/**
 * @brief 解析程序命令行
 *
 * @param argc  用于存放命令行参数的个数
 * @param argv  是个字符指针的数组，每个元素都是一个字符指针，指向一个字符串，即命令行中的每一个参数。
 * @param arg  返回解析结果
 *
 * @return int 0:正确，错误返回其他
 */
extern int commandLineParser(int argc, char *argv[],command_arg* arg);

/**
 * @brief 命令行参数输入
 *
 * @param carg  命令行参数，如：”--path /home/robot/Work/system/robot_config --port 6665 --iscopy true“。（NULL为默认）
 * @param arg  返回解析结果
 *
 * @return int 0:正确，错误返回其他
 */
extern int commandLineParser1(const char* carg, command_arg* arg);

/**
 * @brief 系统初始化
 *
 * @param arg 系统启动参数(NULL为默认)
 * @return int 0:成功; 其他:失败
 */
extern int system_initialize(command_arg* arg);

/**
 * @brief 系统是否已初始化
 *
 * @return int 0:未初始化; 1:已初始化
 */
extern int is_system_initialize();

/**
 * @brief 获取系统仿真状态
 *
 * @return int 0:真机; 其他:仿真
 */
extern int GetSystemSimulationState();

/**
 * @brief 仅通信驱动初始化
 *
 * @param name 应用名称
 * @param config_file 配置文件夹路径
 * @return int 0:成功; 其他失败
 */
extern int DeviceInitialize(const char* name,const char* config_file);

/**
 * @brief 机器人状态是否正常
 *
 * @return int 1:状态正常; 0:状态错误或强制退出
 */
extern int robot_ok();

/**
 * @brief 附加轴组状态是否正常
 *
 * @return int 1:状态正常; 0:状态错误或强制退出
 */
extern int addition_ok();

/**
 * @brief 机器人运动状态是否正常
 *
 * @return int 1:可以运动; 0:状态错误或有机器人正在运动
 */
extern int robot_move_ok();

/**
 * @brief 附加轴组运动状态是否正常
 *
 * @return int 1:可以运动; 0:状态错误或有附加轴组正在运动
 */
extern int addition_move_ok();

/**
 * @brief 机器人是否正在运行
 *
 * @return int 1:在运行; 0:未运行
 */
extern int robot_runing();

/**
 * @brief 附加轴组是否正在运行
 *
 * @return int 1:在运行; 0:未运行
 */
extern int addition_runing();

/**
 * @brief 机器人是否处于强制停止或错误状态
 *
 * @param robot_index 机器人索引
 * @return int 0:机器人处于强制停止或错误状态; 1:机器人未处于强制停止或错误状态;
 */
extern int robot_ok_i(int robot_index);

/**
 * @brief 附加轴组是否处于强制停止或错误状态
 *
 * @param addition_index 附加轴组索引
 * @return int 0:附加轴组处于强制停止或错误状态; 1:附加轴组未处于强制停止或错误状态;
 */
extern int addition_ok_i(int addition_index);

/**
 * @brief 所有机器人是否处于强制停止或错误状态
 *
 * @return int 0:有机器人处于强制停止或错误状态; 1:所有机器人未处于强制停止或错误状态;
 */
extern int RobotOk();

/**
 * @brief 所有附加轴组是否处于强制停止或错误状态
 *
 * @return int 0:有附加轴组处于强制停止或错误状态; 1:所有附加轴组未处于强制停止或错误状态;
 */
extern int AdditionOk();

/**
 * @brief 所有机器人是否处于强制停止或错误状态
 *
 * @return int 0:所有机器人处于强制停止或错误状态; 1:有机器人未处于强制停止或错误状态;
 */
extern int RobotNoOK();

/**
 * @brief 所有附加轴组是否处于强制停止或错误状态
 *
 * @return int 0:所有附加轴组处于强制停止或错误状态; 1:有附加轴组未处于强制停止或错误状态;
 */
extern int AdditionNoOk();

/**
 * @brief 获取控制器运行时间
 *
 * @return int 控制器运行时间（ms）
 */
extern int GetSystemRunTime();

/**
 * @brief 导入license（系统重启后生效）
 *
 * @param licensestr  字符形式的license序列号
 * @return int 0:输入匹配的license并成功倒入；其他：license错误或导入失败
 */
extern int ImportLicense(const char* licensestr);

/**
 * @brief 获取获取系统cpu ID和网卡mac信息
 *
 * @param cpu_id  返回cpu id 空间大小至少为17个字节
 * @param cup_id_num  cpu_id 的大小
 * @param net_mac  返回网卡 mac 空间大小至少为13个字节
 * @param net_mac_num  net_mac 的大小
 * @return int 0:操作成功：其他：失败
 */
extern int GetHardwareInformation(char* cpu_id,int cup_id_num, char* net_mac, int net_mac_num);

/**
 * @brief 获取系统版本
 *
 * @param major  返回主版本号
 * @param minor  返回子版本号
 * @param build  返回修复版本号
 */
extern void GetSystemVerison(int* major,int* minor, int* build);

/**
 * @brief 系统是否授权
 *
 * @return int 0: 未授权；1:授权
 */
extern int IsSystemAuthorizationRun();

/**  
 * @brief 在指定日志级别下记录机器人系统的日志消息。  
 *   
 * 此函数用于输出日志消息，以帮助调试和监控机器人系统的状态和行为。  
 * 日志级别允许对消息进行分类，如信息、警告或错误等。  
 *   
 * @param log_level 整数，指定日志消息的严重性或重要性。  
 *                  常见的日志级别可能包括：  
 *                  - 0: 调试 (DEBUG)  
 *                  - 1: 信息 (INFO)  
 *                  - 2: 注意 (NOTICE)  
 *                  - 3: 警告 (WARNING)  
 *                  - 4: 错误 (ERROR)  
 *   
 * @param __format  C 字符串，包含要写入日志的文本。  
 *                  它可以选择性地包含嵌入的格式说明符，这些说明符将被后续提供的附加参数所替换，  
 *                  并按照 printf 类似的语法进行格式化。  
 *   
 * @param ...       可选参数，指定格式化数据，这些数据将插入到格式字符串中的格式说明符位置。  
 *   
 * @return 一个整数，表示日志操作的成功(0)或失败(<0)。 
 */  
extern int robot_log_out(int log_level,const char *__restrict __format, ...);

#define ROBOT_DEBUG(...) robot_log_out(0,__VA_ARGS__) //!< DEBUG
#define ROBOT_INFO(...) robot_log_out(1,__VA_ARGS__) //!< INFO
#define ROBOT_NOTICE(...) robot_log_out(2,__VA_ARGS__) //!< NOTICE
#define ROBOT_WARNING(...) robot_log_out(3,__VA_ARGS__) //!< WARNING
#define ROBOT_ERROR(...) robot_log_out(4,__VA_ARGS__) //!< ERROR

#ifdef __cplusplus
}
}
#endif

#endif /* ROBOTSYSTEM_H_ */
