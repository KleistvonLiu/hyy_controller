/**
 * @file SerialCommunication.h
 * 
 * @brief 串口通信
 * @author HanBing
 * @version 12.0.0
 * @date 2024-02-01
 * 
 */
#ifndef INCLUDE_SERIALCOMMUNICATION_H_
#define INCLUDE_SERIALCOMMUNICATION_H_
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

typedef struct Serial Serial;///<串口数据结构

/**
* @brief 创建串口数据(需要close_serial_ports是否数据资源)
*
* @param device 设备名称, 如:"/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*",etc
* @param baud 波特率,如:9600, 19200, 57600, 115200, etc
* @param parity 校验位,如:'N', 'O', 'E'
* @param data_bit 数据位,如:5,6,7,8 
* @param stop_bit 停止位,如:1,2 
* @param mode 串口模式,0:RS232, 1:RS485 
* @param oflag 串口打开方式,如:O_RDWR , O_NOCTTY , O_NDELAY, O_EXCL 
* @return Serial* 反回串口数据指针，数据创建失败为NULL
*/
extern Serial* new_serial_ports(const char *device, int baud, char parity,
        int data_bit, int stop_bit, int mode, int oflag);

/**
* @brief 打开并连接串口
*
* @param serial 串口数据
* @return int 0：成功,其他：失败
*/
extern int serial_ports_connect(Serial* serial);

/**
* @brief 断开串口并释放数据资源
*
* @param serial_ptr 串口数据指针
*/
extern void close_serial_ports(Serial** serial_ptr);

/**
* @brief 清空串口缓冲区数据
*
* @param serial 串口数据
* @return int 0：成功,其他：失败
*/
extern int flush_serial_ports(Serial* serial);

/**
* @brief 读串口数据
*
* @param serial 串口数据
* @param data 返回的读取数据
* @param length 返回的读取数据字节数
* @return int 返回读取的字节数目，小于0失败
*/
extern int read_serial_ports(Serial* serial, void* data,int length);

/**
* @brief 写串口数据
*
* @param serial 串口数据
* @param data 写入的数据
* @param length 写入的数据字节数
* @return int 返回写入的字节数目，小于0失败
*/
extern int write_serial_ports(Serial* serial, void* data,int length);


#ifdef __cplusplus
}
}
#endif

#endif /* INCLUDE_SERIALCOMMUNICATION_H_ */