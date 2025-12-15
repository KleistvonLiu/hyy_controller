/**
* @file ModbusServer.h
*
* @brief  modubs server 通信接口
* @author hanbing
* @version 12.1.0
* @date 2025-01-6
*
*/

#ifndef INCLUDE_MODBUSSERVER_H_
#define INCLUDE_MODBUSSERVER_H_
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "Base/MetaType.h"

#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

/**
* @brief 创建modbus server(tcp)
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param m_t modbus 类型
* @param server_id id,范围通常为 1 到 247
* @param _update_registers  在client 请求时回调，其中可以用于更新client所读取的寄存器,可以为NULL
* @param _update_device 在client 请求时回调，其中可以使用client所写入的数据更新系统所管理的状态变量,可以为NULL
* @return int 成功返回0，错误返回其他
*/
extern int modbus_tcp_server_fun(char* ip, int port, modbus_type m_t,int server_id, void (*_update_registers)(void* arg),void (*_update_device)(void* arg));

/**
* @brief 创建modbus server(rtu)
*
* @param device 指定串口设备的名称,如/dev/ttyS0 或 /dev/ttyUSB0
* @param baud 波特率
* @param parity 校验位,'N'：无校验;'E'：偶校验;'O'：奇校验
* @param data_bit 数据位,通常为 7 或 8
* @param stop_bit 停止位长度,1 个停止位 或2 个停止位
* @param mode RTU 模式的配置选项  0:MODBUS_RTU_RS232,1:MODBUS_RTU_RS485
* @param server_id id,范围通常为 1 到 247
* @param _update_registers  在client 请求时回调，其中可以用于更新client所读取的寄存器,可以为NULL
* @param _update_device 在client 请求时回调，其中可以使用client所写入的数据更新系统所管理的状态变量,可以为NULL
* @return int 成功返回0，错误返回其他
*/
extern int modbus_rtu_server_fun(const char *device, int baud, char parity,
        int data_bit, int stop_bit,int mode, int server_id, void (*_update_registers)(void* arg),void (*_update_device)(void* arg));

/**
* @brief 关闭modbus server
*/
extern void modbus_server_close();

/**
* @brief 写入功能码05写单个线圈/功能码15写多个线圈
* @param address 寄存器地址
* @param nb 字节个数
* @param in 写入数组
* @return int 成功返回0，错误返回其他
*/
extern int device_to_coils(uint16_t address, int nb, uint8_t *in);

/**
* @brief 写入离散输入（非协议规范，用于更新数据）
* @param address 寄存器地址
* @param nb 字节个数
* @param in 写入数组
* @return int 成功返回0，错误返回其他
*/
extern int device_to_input_bits(uint16_t address, int nb, uint8_t *in);

/**
* @brief 写入功能码06写单个保持寄存器/功能码16写多个保持寄存器
* @param address 寄存器地址
* @param nb 数组个数
* @param in 写入数组
* @return int 成功返回0，错误返回其他
*/
extern int device_to_registers(uint16_t address, int nb, uint16_t *in);

/**
* @brief 写入输入寄存器（非协议规范，用于更新数据）
* @param address 寄存器地址
* @param nb 数组个数
* @param in 写入数组
* @return int 成功返回0，错误返回其他
*/
extern int device_to_input_registers(uint16_t address, int nb, uint16_t *in);

/**
* @brief 功能码01读单个线圈
* @param address 寄存器地址
* @param out 读出的数据
* @return int 成功返回0，错误返回其他
*/
extern int coil_to_device(uint16_t address, uint8_t* out);

/**
* @brief 功能码03读单个保持寄存器
* @param address 寄存器地址
* @param out 读数组
* @return int 成功返回0，错误返回其他
*/
extern int register_to_device(uint16_t address, uint16_t* out);

/**
* @brief 功能码01读多个线圈
* @param address 寄存器地址
* @param nb 字节个数
* @param out 读出的数据
* @return int 成功返回0，错误返回其他
*/
extern int coils_to_device(uint16_t address, int nb, uint8_t* out);

/**
* @brief 功能码03读多个保持寄存器
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读出的数据
* @return int 成功返回0，错误返回其他
*/
extern int registers_to_device(uint16_t address, int nb, uint16_t *out);

/**
* @brief 功能码02读离散输入寄存器
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读出的数据
* @return int 成功返回0，错误返回其他
*/
extern int input_bits_to_device(uint16_t address, int nb, uint8_t *out);

/**
* @brief 功能码04读输入寄存器
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读出的数据
* @return int 成功返回0，错误返回其他
*/
extern int input_registers_to_device(uint16_t address, int nb, uint16_t *out);


#ifdef __cplusplus
}
}
#endif

#endif /* INCLUDE_MODBUSSERVER_H_ */