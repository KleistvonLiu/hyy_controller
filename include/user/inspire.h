#ifndef INSPIRE_H_
#define INSPIRE_H_
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <bitset>
int init_serial(const std::string& port, int baudrate = B115200);

void write6(int fd, const std::string& reg_name, const std::vector<int>& val, const std::string& id_value);


#endif /*INSPIRE_H_*/