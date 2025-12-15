/**
* @file InputEventInterface.h
*
* @brief  输入外设事件通信接口
* @author hanbing
* @version 12.2.0
* @date 2025-09-06
*
*/

#ifndef INPUTEVENTINTERFACE_H_
#define INPUTEVENTINTERFACE_H_
#include <linux/input.h>
#include <linux/joystick.h>
#include <stdint.h>

#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

/*-------------------------------------------------------------------------*/
/**
 * @brief	输入事件配置数据

	存储配置数据,通过input_event_open()初始化设置
 */
/*-------------------------------------------------------------------------*/
typedef struct InputEvent {
uint8_t isblock; /**< 是否阻塞等待事件输入 */
uint8_t value_range; /**<捕获数据范围，bit0:捕获0值;bit1:捕获检测负值;bit2:捕获检测正值;*/
uint16_t type; /**< 捕获指定的数据类型,捕获所有数据类型:0xFF */
int fd; /**< 设备文件描述符 */
}InputEvent;

/**
* @brief 打开外设通信，并初始化配置数据
*
* @param ie 配置数据
* @param device_name 设备名, 如："/dev/input/js0"
* @param type 指定捕获输入事件类型, 捕获所有数据类型:0xFF
* @param value_range 指定捕获输入数据范围,bit0:捕获0值;bit1:捕获检测负值;bit2:捕获检测正值,如： 0x5为捕获0和正数
* @param isblock 是否阻塞等待事件输入,1:阻塞;0:非阻塞
* @return int 成功返回0，错误返回其他
*/
extern int input_event_open(InputEvent* ie,const char* device_name,uint16_t type,uint8_t value_range,uint8_t isblock);

/**
* @brief 读取外设数据数据
*
* @param ie 配置数据
* @param ev 外设数据
* @return int 成功返回0，错误返回其他
*/
extern int input_event_read(InputEvent* ie,struct input_event* ev);

/**
* @brief 读取js外设数据数据
*
* @param ie 配置数据
* @param ev 外设数据
* @return int 成功返回0，错误返回其他
*/
extern int input_js_event_read(InputEvent* ie,struct js_event* ev);

/**
* @brief 关闭外设通信
*
* @param ie 配置数据
* @return int 成功返回0，错误返回其他
*/
extern int input_event_close(InputEvent* ie);

#ifdef __cplusplus
}
}
#endif

#endif /*INPUTEVENTINTERFACE_H_*/