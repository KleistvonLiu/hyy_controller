/**
 * @file PluginTemplate.cpp
 *
 * @brief 控制系统插件,具体形式不局限于该模板形式，仅插件入口函数必须
 * @author hanbing
 * @version 12.2.0
 * @date 2024-04-18
 */
#include "HYYRobotInterface.h"
#include "PluginTemplate.h"
using namespace HYYRobotBase;//仅c++需要
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <iostream>
//
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 插件入库函数,用于实现插件初始化(被控制系初始函数调用),该函数要求非阻塞，如阻塞需要开线程运行
 */
extern void PluginMain();
#ifdef __cplusplus
}
#endif

void PluginMain()
{
    std::cout<<"PluginMain, start ..."<<std::endl;

    PluginClassDemo* PD=GetPluginInstantiation();
    PD->Init();
}







//c++
static PluginClassDemo _PluginData; 
PluginClassDemo* GetPluginInstantiation()
{
    return &_PluginData;
}

void PluginClassDemo::Init()
{
    std::cout<<"class init"<<std::endl;
}

void PluginClassDemo::UserClass()
{
 std::cout<<"class use"<<std::endl;
}


