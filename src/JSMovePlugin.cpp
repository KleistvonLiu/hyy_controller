/**
 * @file SUPCONPlugin.cpp
 *
 * @brief 中控信息事件功能插件
 * @author hanbing
 * @version 12.2.0
 * @date 2025-09-07
 */
#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <atomic>
#include <vector>
#include "user/inspire.h"
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

static std::thread t;
static std::thread tshow;
static std::atomic<bool> isshow;
static std::atomic<bool> isstop;
static int hand_left;
static int hand_right;
static std::string HANDID="01";

void PluginMain()
{
    ROBOT_INFO("PluginMain:JSMovePlugin init\n");
    // hand_left = init_serial("/dev/ttyS0");
    // if (hand_left<0)
    // {
    //     ROBOT_INFO("PluginMain:init_serial failure\n");
    //     return;
    // }
    // hand_right = init_serial("/dev/ttyS1");
    // if (hand_right<0)
    // {
    //     ROBOT_INFO("PluginMain:init_serial failure\n");
    //     return;
    // }
    // std::vector<int> values = {0, 0, 0, 0, 0, 1000};
    // write6(hand_left, "angleSet", values, HANDID);
    // write6(hand_right, "angleSet", values, HANDID);

    sleep(2);

    t=std::thread([](){ 
        InputEvent ie;
        if (0!=input_event_open(&ie,"/dev/input/js0",0x1,0x4,1))
        {
            ROBOT_ERROR("input_event_open failure\n");
            return;
        }
        struct js_event js;

        isshow.store(false);
        isstop.store(false);
        while (true)
        {
            if (0==input_js_event_read(&ie,&js))
            {
                // printf("%u,%u\n",js.type,js.number);
                if (1==js.type && 0==js.number)
                {
                    ROBOT_INFO("DevicePower()\n");
                    ClearDeviceError();
                    DevicePower();
                    setMoveThread(1);

                } else if (1==js.type && 1==js.number)
                {
                    ROBOT_INFO("DevicePoweroff()\n");
                    DevicePoweroff();
                    setMoveThread(0);
                    // {
                    //     std::vector<int> values = {0, 0, 0, 0, 0, 1000};
                    //     write6(hand_left, "angleSet", values, HANDID);
                    //     write6(hand_right, "angleSet", values, HANDID);
                    // }
                    
                } else if (1==js.type && 2==js.number)
                {
                    ROBOT_INFO("home\n");
                    DeviceStop();
                    isstop.store(true);
                    while (isshow.load())
                    {
                        usleep(100000);
                    }
                    DeviceStopRecover();
                    if (0==DevicePowerState())
                    {
                        DevicePoweroff();
                        ClearDeviceError();
                        DevicePower();
                    }
                    setMoveThread(1);
                    IMPORTJOINT(R0_jhome);
                    IMPORTJOINT(R1_jhome);
                    IMPORTJOINT(A0_jhome);
                    //IMPORTJOINT(A1_jhome);
                    IMPORTSPEED(R0_v50);  
                    MultiMoveA(R0_jhome,R0_v50,NULL3,0);
                    MultiMoveA(R1_jhome,R0_v50,NULL3,1);
                    MultiMoveAdd(A0_jhome, R0_v50, NULL3, 0);

                    //MultiMoveAdd(A1_jhome, v50, NULL3, 1);
                    // {
                    //     std::vector<int> values ={0, 0, 0, 0, 0, 1000};
                    //     write6(hand_left, "angleSet", values, HANDID);
                    //     write6(hand_right, "angleSet", values, HANDID);
                    // }
                    
                } else if (1==js.type && 3==js.number)
                {
                    ROBOT_INFO("init\n");
                    DeviceStop();
                    isstop.store(true);
                    while (isshow.load())
                    {
                        usleep(100000);
                    }
                    DeviceStopRecover();
                    if (0==DevicePowerState())
                    {
                        DevicePoweroff();
                        ClearDeviceError();
                        DevicePower();
                    }
                    setMoveThread(1);
                    IMPORTJOINT(R0_jinit);
                    IMPORTJOINT(R1_jinit);
                    IMPORTJOINT(A0_jinit);
                    // IMPORTJOINT(A1_jinit);
                    IMPORTSPEED(R0_v50);
                    MultiMoveA(R0_jinit,R0_v50,NULL3,0);
                    MultiMoveA(R1_jinit,R0_v50,NULL3,1);
                    MultiMoveAdd(A0_jinit, R0_v50, NULL3, 0);
                    //MultiMoveAdd(A1_jinit, v50, NULL3, 1);
                    usleep(10000);
                    MoveSyncStart();
                    WaitDeviceMoveFinish();
                    // {
                    //     std::vector<int> values ={1000, 1000, 1000, 1000, 1000, 1000};
                    //     write6(hand_left, "angleSet", values, HANDID);
                    //     write6(hand_right, "angleSet", values, HANDID);
                    // }
                    
                } else if (1==js.type && 7==js.number)
                {
                    ROBOT_INFO("move\n");
                    DeviceStop();
                    isstop.store(true);
                    while (isshow.load())
                    {
                        usleep(100000);
                    }
                    DeviceStopRecover();
                    if (0==DevicePowerState())
                    {
                        DevicePoweroff();
                        ClearDeviceError();
                        DevicePower();
                    }

                    tshow=std::thread([](){
                        isstop.store(false);
                        isshow.store(true);
                        IMPORTJOINT(R0_j1);
                        IMPORTJOINT(R1_j1);
                        IMPORTJOINT(A0_j1);
                        //IMPORTJOINT(A1_j1);
                        IMPORTJOINT(R0_j2);
                        IMPORTJOINT(R1_j2);
                        IMPORTJOINT(A0_j2);
                        //IMPORTJOINT(A1_j2);
                        IMPORTJOINT(R0_j3);
                        IMPORTJOINT(R1_j3);
                        IMPORTJOINT(A0_j3);
                        //IMPORTJOINT(A1_j3);
                        IMPORTJOINT(R0_j4);
                        IMPORTJOINT(R1_j4);
                        IMPORTJOINT(A0_j4);
                        //IMPORTJOINT(A1_j4);
                        IMPORTJOINT(R0_jinit);
                        IMPORTJOINT(R1_jinit);
                        IMPORTJOINT(A0_jinit);
                        // IMPORTJOINT(A1_jinit);
                        IMPORTJOINT(R0_jhome);
                        IMPORTJOINT(R1_jhome);
                        IMPORTJOINT(A0_jhome);
                        //IMPORTJOINT(A1_jhome);
                        IMPORTSPEED(R0_v50);
                        IMPORTSPEED(R0_v100);
                        IMPORTSPEED(R0_v200);
                        setMoveThread1(1,10e9);
                        MultiMoveA(R0_jinit,R0_v50,NULL3,0);
                        MultiMoveA(R1_jinit,R0_v50,NULL3,1);
                        MultiMoveAdd(A0_jinit, R0_v50, NULL3, 0);
                        usleep(10000);
                        MoveSyncStart();
                        WaitDeviceMoveFinish();
                        //MultiMoveAdd(A1_jinit, v50, NULL3, 1);

                        int count =0;
                        while (robot_ok()&&!isstop.load())
                        {
                            MultiMoveA(R0_jinit,R0_v200,NULL3,0);
                            MultiMoveA(R1_jinit,R0_v200,NULL3,1);
                            MultiMoveAdd(A0_jhome,R0_v200,NULL3,0);//抬头
                            usleep(10000);
                            MoveSyncStart();
                            WaitDeviceMoveFinish();
                            count=0;
                            // {
                            //     std::vector<int> values = {1000, 1000, 1000, 1000, 1000, 1000};
                            //     write6(hand_left, "angleSet", values, HANDID);
                            //     write6(hand_right, "angleSet", values, HANDID);
                            // }
                            
                            while(count<3)//双手挥手
                            {
                                MultiMoveA(R0_j1,R0_v100,NULL3,0);
                                MultiMoveA(R1_j1,R0_v100,NULL3,1);
                                usleep(10000);
                                MoveSyncStart();
                                WaitDeviceMoveFinish();
                                MultiMoveA(R0_j2,R0_v100,NULL3,0);
                                MultiMoveA(R1_j2,R0_v100,NULL3,1);
                                usleep(10000);
                                MoveSyncStart();
                                WaitDeviceMoveFinish();
                                count++;
                            }
                            // {
                            //     std::vector<int> values1={0, 0, 0, 0, 0, 0};
                            //     std::vector<int> values2={0, 0, 0, 0, 1000, 1000};
                            //     write6(hand_left, "angleSet", values1, HANDID);
                            //     write6(hand_right, "angleSet", values2, HANDID);
                            // }
                            MultiMoveA(R0_j3,R0_v200,NULL3,0);//竖拇指
                            MultiMoveA(R1_j3,R0_v200,NULL3,1);
                            MoveSyncStart();
                            WaitDeviceMoveFinish();
                            Rsleep(2000);

                            // {
                            //     std::vector<int> values={1000, 1000, 1000, 1000, 1000, 1000};
                            //     write6(hand_left, "angleSet", values, HANDID);
                            //     write6(hand_right, "angleSet", values, HANDID);
                            // } 
                            MultiMoveA(R0_j4,R0_v200,NULL3,0);//请进
                            MultiMoveA(R1_j4,R0_v200,NULL3,1);
                            MultiMoveAdd(A0_jinit,R0_v200,NULL3,0);
                            MoveSyncStart();
                            WaitDeviceMoveFinish();
                            Rsleep(2000);
                        }
                        isshow.store(false);

                    });
                    if (tshow.joinable()) {
                        tshow.detach(); // 分离线程
                    }
                }
            }
        }

    });

    if (t.joinable()) {
        t.detach(); // 分离线程
    }

}
