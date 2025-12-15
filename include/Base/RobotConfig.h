/**
 * @file RobotConfig.h
 *
 * @brief  机器人配置数据获取接口
 * @author hanbing
 * @version 11.4.6
 * @date 2023-11-08
 *
 */

#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/**
 * @brief 系统版本
 */
/*-------------------------------------------------------------------------*/
struct ConfigVersion{
	int major; ///< 主版本
	int minor; ///< 次版本
	int build; ///< 修订版
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 笛卡尔数据
 */
/*-------------------------------------------------------------------------*/
struct CartesianData{
	double x;///< x
	double y;///< y
	double z;///< z
	double k;///< 绕x(固定角)
	double p;///< 绕y(固定角)
	double s;///< 绕z(固定角)
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 布尔数据类型
 */
/*-------------------------------------------------------------------------*/
enum BoolType{
	_FALSE,///< 假
	_TRUE///< 真
};

//----------------------------------------------BusConfig-------------------------------------
/*-------------------------------------------------------------------------*/
/**
 * @brief 总线类型
 */
/*-------------------------------------------------------------------------*/
enum BusType {
	_ETHERCAT = 0,///< EtherCAT
	_CANOPEN,///< CanOpen
	_GENERAL///< 通用
};

/*-------------------------------------------------------------------------*/
/**
 * @brief EtherCAT/CanOpen总线PDO映射名称
 */
/*-------------------------------------------------------------------------*/
struct BusPdo{
	const char *device_description; ///<总线配置文件路径及名称
	const char *state; ///<状态字
	const char *control; ///<控制字
	const char *mode; ///<模式字
	const char *ac_mode; ///<实际模式字
	const char *ac_position; ///<实际位置
	const char *ac_velocity; ///<实际速度
	const char *ac_torque; ///<实际力矩
	const char *ac_position2; ///<第 2 路实际位置
	const char *ac_velocity2; ///<第 2 路实际速度
	const char *ac_sensor_torque; ///<实际关节力矩传感器
	const char *ac_sensor_torque2; ///<第二路实际关节力矩传感器
	const char *error_code; ///<错误代码
	const char *following_error; ///<跟踪误差
	const char *position; ///<目标位置
	const char *velocity; ///<目标速度
	const char *torque; ///<目标力矩
	const char *velocity_offset; ///<速度前馈
	const char *torque_offset; ///<力矩前馈
	const char *torque_max_limit; ///<最大力矩限制
	const char *torque_min_limit; ///<最小力矩限制
	const char *usr_di; ///<用户数字量输入
	const char *usr_do; ///<用户数字量输出
	const char *usr_ai; ///<用户模拟量输入
	const char *usr_ao; ///<用户模拟量输出
	const char *inter_di; ///<内部数字量输入
	const char *inter_do; ///<内部数字量输
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 通用总线数据描述
 */
/*-------------------------------------------------------------------------*/
struct GeneralData{
	const char *device_description;///<数据描述
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 总线数据
 */
/*-------------------------------------------------------------------------*/
struct BusData{
	struct BusPdo *bus_pdo;///<EtherCAT/CanOpen pdo 数据
	struct GeneralData *general_data;///<通用数据
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 总线配置数据
 */
/*-------------------------------------------------------------------------*/
typedef struct BusConfig {
	struct ConfigVersion version;///<版本号
	const char *serial;///<机器人序列号
	const char *description;///<该文件的描述
	int communication_cycle;///<通信周期,单位为 ns 纳秒
	enum BusType type;///<通信总线类型,ethercat, canopen, general
	struct BusData * busdata;///<总线数据
}BusConfig;


//----------------------------------------------GlobalConfig-------------------------------------

/*-------------------------------------------------------------------------*/
/**
 * @brief 全局配置数据
 */
/*-------------------------------------------------------------------------*/
typedef struct GlobalConfig {
	struct ConfigVersion version;///<版本号
	const char *serial;///<机器人序列号
	const char *description;///<该文件的描述
	const char *app_path;///<数据存放目录
	enum BoolType simulation;///<是否开启仿真, true 仿真, false 真机
	const char *run_program;///<自启动程序名称
	const char **plugin_list;///<自启动插件名称列表
	unsigned plugin_list_num;///<自启动插件数量
}GlobalConfig;


//----------------------------------------------ExternalDeviceConfig-------------------------------------
/*-------------------------------------------------------------------------*/
/**
 * @brief 外部设备通信类型
 */
/*-------------------------------------------------------------------------*/
enum CommType {
	_TCP = 0,///<tcp
	_UDP,///<udp
	_USB,///<usb
	_SERIAL,///<serial
	_DIO,///<数字io
	_ECAT,///<EtherCAT
	_AIO,///<模拟io
	_MODBUSRTU///<modbus-rtu
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 外部设备参数
 */
/*-------------------------------------------------------------------------*/
struct DeviceArgData{
	const char *id;///<输入参数 1
	const char *sub_id;///<输入参数 2
	const char *cmd;///<命令参数
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 夹爪数据
 */
/*-------------------------------------------------------------------------*/
struct GripData{
	const char *name;///<夹爪名称
	const char *description;///<夹爪信息描述
	const char *serial;///<所支持的夹爪类型
	enum CommType type;///<通信方式
	struct DeviceArgData arg;///<设备参数
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 传感器基础数
 */
/*-------------------------------------------------------------------------*/
struct ForceSensorAttributeData{
	double mass;///<质量
	struct CartesianData measuring_point;///<测量点位置
	struct CartesianData center;///<质心位置
	struct CartesianData offset;///<初始偏移
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 传感器数据
 */
/*-------------------------------------------------------------------------*/
struct ForceSensorData{
	const char *name;///<传感器名称
	const char *description;///<传感器信息描述
	const char *serial;///<所支持的传感器类型
	enum CommType type;///<通信方式
	struct DeviceArgData arg;///<设备参数
	struct ForceSensorAttributeData data;///<传感器基础数
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 外部设备数据
 */
/*-------------------------------------------------------------------------*/
struct ExternalDeviceData{
	struct GripData *grip;///<夹爪数据
	unsigned grip_num;///<夹爪数量
	struct ForceSensorData *force_sensor;///<传感器数据
	unsigned force_sensor_num;///<传感器数量
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 外部设备配置数据
 */
/*-------------------------------------------------------------------------*/
typedef struct ExternalDeviceConfig{
	struct ConfigVersion version;///<版本号
	const char *serial;///<版本号
	const char *description;///<该文件的描述
	struct ExternalDeviceData *devices;///<外部设备数据
}ExternalDeviceConfig;


//----------------------------------------------DeviceConfig-------------------------------------
/*-------------------------------------------------------------------------*/
/**
 * @brief 关节类型
 */
/*-------------------------------------------------------------------------*/
enum JointType {
	_REVOLUTE= 0,///<转动
	_PRISMATIC///<滑动
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 数字io数据
 */
/*-------------------------------------------------------------------------*/
struct DigitalIoData{
	unsigned input_num;///<输入数字io数目
	unsigned output_num;///<输出数字io数目
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 模拟量转换数据
 */
/*-------------------------------------------------------------------------*/
struct AnalogIoCoeffData{
	double a;///<比例
	double b;///<偏移
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 模拟量数据
 */
/*-------------------------------------------------------------------------*/
struct AnalogIoData{
	struct AnalogIoCoeffData *input;///<模拟输入数据
	unsigned input_num;///<模拟输入个数
	struct AnalogIoCoeffData *output;///<模拟输出数据
	unsigned output_num;///<模拟输出个数
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 轴数据
 */
/*-------------------------------------------------------------------------*/
struct AxisData{
	unsigned id;///<驱动从站设备索引
	double initial_offset;///<编码器零点标定初值
	double initial_offset2;///<输出端编码器零点标定初值
	double sensor_offset;///<关节力矩传感器零点标定初值
	double sensor_offset2;///<第二路关节力矩传感器零点标定初值
	enum JointType sigma;///<关节类型
	enum BoolType direction;///<编码器旋转正方向,是否与关节正方向一致，一致为 trure，不一致为 false
	enum BoolType direction2;///<第二路编码器旋转正方向,是否与关节正方向一致，一致为 trure，不一致为 false
	double smin;///<关节最小限位
	double smax;///<关节最大限位
	double vmin;///<最小速度限制
	double vmax;///<最大速度限制
	double amin;///<最小加速度限制
	double amax;///<最大加速度限制
	double jmin;///<最小加加速度限制
	double jmax;///<最大加加速度限制
	double current_coeff;///<电流转化系数
	double torque_coeff;///<等效扭矩常熟转换系数
	double torque_max;///<最大关节力矩限制
	double sensor_coeff;///<关节力矩传感器转换系数
	double sensor_coeff2;///<第二路关节力矩传感器转换系数
	double encoder_ruler;///<编码器单圈数据
	double encoder_ruler2;///<第二路编码器单圈数据
	double gear_ratio;///<关节减速比
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 运动学数据
 */
/*-------------------------------------------------------------------------*/
struct KinematicsData{
	double theta;///<绕z轴转动
	double d;///<沿Z轴移动
	double a;///<沿x轴移动
	double alpha;///<绕x轴转动
	double offset;///<实际零位与模型零位偏差
	enum BoolType mdh;///<dh参数类型,true为修正dh参数，false为标准dh参数
	enum BoolType flip;///<是否修改模型正方向,true修改，false为不修改
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 外部轴基础数据
 */
/*-------------------------------------------------------------------------*/
struct AxisGroupData{
	struct KinematicsData kinematics_parameters;///<外部轴运动学数据
	struct AxisData axis;///<///<外部轴轴数据
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 外部轴组数据
 */
/*-------------------------------------------------------------------------*/
struct ADDITION{
	unsigned link_robot_index;///<外部轴组与机器人的关联关系,第几位置位表示与第几个机器人关联
	struct CartesianData tool;///<运动学附加末端变换
	struct CartesianData base;///<运动学附加基变换
	struct AxisGroupData *axis_group;///<外部轴基础数据
	unsigned dof;///<关节数量
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 多外部轴组数据
 */
/*-------------------------------------------------------------------------*/
struct AdditionGroup{
	struct ADDITION *addition;///<外部轴组数据
	unsigned addition_num;///<外部轴组数目
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 重力加速度数据
 */
/*-------------------------------------------------------------------------*/
struct GravityData{
	double x;///<基坐标系x方向加速度
	double y;///<基坐标系y方向加速度
	double z;///<基坐标系z方向加速度
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 末端约束数据
 */
/*-------------------------------------------------------------------------*/
struct EndLimitData{
	double tran_velocity_max;///<笛卡尔平动速度约束
	double tran_acceleration_max;///<笛卡尔平动加速度约束
	double tran_jerk_max;///<笛卡尔平动加加速度约束
	double rot_velocity_max;///<笛卡尔转动速度约束
	double rot_acceleration_max;///<笛卡尔转动加速度约束
	double rot_jerk_max;///<笛卡尔转动加加速度约束
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 动力学数据
 */
/*-------------------------------------------------------------------------*/
struct DynamicsData{
	double m;///<质量
	double cmx;///<质心x轴
	double cmy;///<质心y轴
	double cmz;///<质心z轴
	double ixx;///<相对于质心的惯性张量(0,0)
	double ixy;///<相对于质心的惯性张量(0,1)
	double ixz;///<相对于质心的惯性张量(0,2)
	double iyy;///<相对于质心的惯性张量(1,1)
	double iyz;///<相对于质心的惯性张量(1,2)
	double izz;///<相对于质心的惯性张量(2,2)
	double fv;///<粘滞摩擦系数
	double fv2;///<预留
	double fv3;///<预留
	double fc0;///<正库伦摩擦系数
	double fc1;///<负库伦摩擦系数
	double jm;///<关节惯性
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 关节连杆数据
 */
/*-------------------------------------------------------------------------*/
struct LINK{
	struct KinematicsData kinematics_parameters;///<运动学数据
	struct DynamicsData dynamics_parameters;///<动力学数据
	struct AxisData axis;///<轴数据
	double* couple;///<耦合系数当前关节与其他关节的耦合系数，当前关节自身的耦合系数为1.0，无耦合可省略，维度要与机器人关节数一致
	int couple_num;///<耦合系数向量维度
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 机器人数据
 */
/*-------------------------------------------------------------------------*/
struct ROBOT{
	const char *model_solve;///<机器人模型求解类型（numerical 是数值解，analytic 是解析解）
	struct CartesianData tool;///<运动学附加末端变换
	struct CartesianData base;///<运动学附加基变换
	struct GravityData gravity;///<重力加速度数据
	struct EndLimitData endlimit;///<末端约束数据
	enum BoolType use_feedforward;///<是否使用动力学前馈
	struct LINK *joint;///<关节连杆数据
	unsigned dof;///<机器人自由度
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 多机器人数据
 */
/*-------------------------------------------------------------------------*/
struct RobotGroup{
	struct ROBOT *robot;///<机器人数据
	unsigned robot_num;///<机器人数目
};

/*-------------------------------------------------------------------------*/
/**
 * @brief 机器人配置数据
 */
/*-------------------------------------------------------------------------*/
typedef struct DeviceConfig{
	struct ConfigVersion version;///<版本号
	const char *serial;///<机器人序列号
	const char *description;///<该文件的描述
	struct DigitalIoData *user_io;///<数字io数据
	struct DigitalIoData *safety_io;///<内部安全数字io数据
	struct AnalogIoData *analog_io;///<模拟量数据
	struct AdditionGroup *addition_group;///<多外部轴组数据
	struct RobotGroup *robot_group;///<多机器人数据
}DeviceConfig;



/*-------------------------------------------------------------------------*/
/**
 * @brief 获取总线配置数据
 * return BusConfig* 配置数据地址
 */
/*-------------------------------------------------------------------------*/
const BusConfig* GetBusConfig();

/*-------------------------------------------------------------------------*/
/**
 * @brief 获取全局配置数据
 * return GlobalConfig* 配置数据地址
 */
/*-------------------------------------------------------------------------*/
const GlobalConfig* GetGlobalConfig();

/*-------------------------------------------------------------------------*/
/**
 * @brief 获取外部设备配置数据
 * return ExternalDeviceConfig* 配置数据地址
 */
/*-------------------------------------------------------------------------*/
const ExternalDeviceConfig* GetExternDeviceConfig();

/*-------------------------------------------------------------------------*/
/**
 * @brief 机器人配置数据
 * return DeviceConfig* 配置数据地址
 */
/*-------------------------------------------------------------------------*/
const DeviceConfig* GetDeviceConfig();

#ifdef __cplusplus
}
}
#endif


#endif /*ROBOTCONFIG_H_*/

