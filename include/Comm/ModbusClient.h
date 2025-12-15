/**
* @file ModbusClient.h
*
* @brief  modubs client 通信接口
* @author hanbing
* @version 12.1.0
* @date 2025-01-6
*
*/

#ifndef INCLUDE_MODBUSCLIENT_H_
#define INCLUDE_MODBUSCLIENT_H_
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


#ifdef __cplusplus
namespace HYYRobotBase
{
	extern "C" {
#endif

/**
* @brief 创建modbus client(tcp)
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param server_id id,范围通常为 1 到 247
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int createModbusClient_tcp(char* ip, int port,  int server_id, const char* sName);

/**
* @brief 创建modbus client(tcp)
*
* @param ip ip地址(NULL为本机ip)
* @param port 端口号
* @param server_id id,范围通常为 1 到 247
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int createModbusClient_tcppi(char* ip, int port,  int server_id,const char* sName);

/**
* @brief 创建modbus server(rtu)
*
* @param device 指定串口设备的名称,如/dev/ttyS0 或 /dev/ttyUSB0
* @param baud 波特率
* @param parity 校验位,'N'：无校验;'E'：偶校验;'O'：奇校验
* @param data_bit 数据位,通常为 7 或 8
* @param stop_bit 停止位长度,1 个停止位 或2 个停止位
* @param rtu_mode RTU 模式的配置选项 
* @param server_id id,范围通常为 1 到 247
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int createModbusClient_rtu(const char *device, int baud, const char parity,
        int data_bit, int stop_bit,int rtu_mode, int server_id,const char* sName);

/**
* @brief 关闭modbus 
*
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int closeModbus(const char* sName);

/**
* @brief 功能码05写单个线圈/功能码15写多个线圈
*
* @param address 寄存器地址
* @param nb 数组个数
* @param in 写入数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int write_coils(int address, int nb, uint8_t *in, const char* sName);

/**
* @brief 功能码01读线圈
*
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int read_coils(int address, int nb, uint8_t *out, const char* sName);

/**
* @brief 功能码02读离散输入
*
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int read_discretes_input(int address, int nb, uint8_t *out, const char* sName);

/**
* @brief 功能码06写单个保持寄存器/功能码16写多个保持寄存器
*
* @param address 寄存器地址
* @param nb 数组个数
* @param in 写入数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int write_registers(int address, int nb, uint16_t *in, const char* sName);

/**
* @brief 功能码03读保持寄存器
*
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int read_registers(int address, int nb, uint16_t *out, const char* sName);

/**
* @brief 功能码04读输入寄存器
*
* @param address 寄存器地址
* @param nb 数组个数
* @param out 读数组
* @param sName server名称
* @return int 成功返回0，错误返回其他
*/
extern int read_input_registers(int address, int nb, uint16_t *out, const char* sName);

/**
* @brief 获取本地ip
*
* @param str_ip 返回ip
* @return int 成功返回0，错误返回其他
*/
extern int get_local_ip_using_ifconf(char *str_ip);

/**
* @brief 从4字节数获取浮点数（ABCD 顺序）,即:不进行任何转换
*
* @param src 数据(2维uint16_t数组)
* @return float 返回浮点数
*/
extern float modbus_get_float_abcd(const uint16_t *src);

/**
* @brief 从4字节数获取浮点数（DCBA 顺序）
*
* @param src 数据(2维uint16_t数组)
* @return float 返回浮点数
*/
extern float modbus_get_float_dcba(const uint16_t *src);

/**
* @brief 从4字节数获取浮点数（BADC 顺序）
*
* @param src 数据(2维uint16_t数组)
* @return float 返回浮点数
*/
extern float modbus_get_float_badc(const uint16_t *src);

/**
* @brief 从4字节数获取浮点数（CDAB 顺序）
*
* @param src 数据(2维uint16_t数组)
* @return float 返回浮点数
*/
extern float modbus_get_float_cdab(const uint16_t *src);

/**
* @brief 从浮点数获取4字节数（ABCD 顺序）,即:不进行任何转换
*
* @param f 浮点数
* @param dest 数据(2维uint16_t数组)
*/
extern void modbus_set_float_abcd(float f, uint16_t *dest);

/* Set a float to 4 bytes for Modbus with byte and word swap conversion (DCBA) */
/**
* @brief 从浮点数获取4字节数（DCBA 顺序）
*
* @param f 浮点数
* @param dest 数据(2维uint16_t数组)
*/
extern void modbus_set_float_dcba(float f, uint16_t *dest);

/**
* @brief 从浮点数获取4字节数（BADC 顺序）
*
* @param f 浮点数
* @param dest 数据(2维uint16_t数组)
*/
extern void modbus_set_float_badc(float f, uint16_t *dest);

/**
* @brief 从浮点数获取4字节数（CDAB 顺序）
*
* @param f 浮点数
* @param dest 数据(2维uint16_t数组)
*/
extern void modbus_set_float_cdab(float f, uint16_t *dest);

#ifdef __cplusplus
}
}
#endif

#endif /* INCLUDE_MODBUSCLIENT_H_ */