/**
 * @file dh_device.h
 * @brief  Device opration class
 * @author Shelling Ding (Renjie.ding@DH-Robotics.com)
 * @version 1.0
 * @date 2020-11-17
 * 
 * @copyright Copyright (c) 2020  Shenzhen DH-Robotics Inc.
 * 
 * 
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-11-17 <td>1.0     <td>Shelling Ding     <td>
 * </table>
 */
#ifndef __dh_device__
#define __dh_device__

#include <cstddef>
#include <cstdint>
#include <string>

// connect uart 
int serial_connect(std::string portname, int Baudrate );

int tcp_connect(std::string portname);

int connect_device(std::string portname, int Baudrate);

//disconnect
void disconnect_device(int fd);

//write data 
int device_wrire(int fd, char *data, int len);

//read data
int device_read(int fd, char *data, int data_len);

//write all bytes or fail
bool device_write_all(int fd, const uint8_t* data, size_t len);




#endif //__dh_device__
