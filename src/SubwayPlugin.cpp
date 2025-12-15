/**
 * @file SubwayPlugin.cpp
 *
 * @brief 地铁项目服务功能插件
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

int subway_deal(int fd)
{
    int flag=0;
    double joint[10];
    uint8_t buf[1024];
    memset(buf,0,sizeof(buf));
    int ret=read(fd,buf,1024);
    if ( ret<= 0)
    {
        if (0==ret)
        {
            return ret;
        }
        else if (errno==EINTR||errno==EWOULDBLOCK||errno==EAGAIN)
        {
            return 1;
        }
        else if (-1==ret&&errno==ECONNRESET)
        {
            return 0;
        }
        ROBOT_ERROR("command_action: Server Recieve Data Failed! ret=%d,errno=%d(%s)\n",ret,errno,strerror(errno));
        return ret;
    }
    if (IsProtocolRight(buf,ret))
    {
        int cmd=0;
        GetProtocolDataInt(buf,"cmd",&cmd);
        switch (cmd)
        {
            case 0:
                ClearDeviceError();
                ret=DevicePower();
                setMoveThread(1);
                break;
            case 1:
                DevicePoweroff();
                setMoveThread(0);
                ret=0;
                break;
            case 2:
                {
                    double joint[10];
                    double vel=0.1;
                    GetProtocolDataDouble(buf,"vel",&vel);
                    GetProtocolDataDouble(buf,"joint1",&(joint[0]));
                    GetProtocolDataDouble(buf,"joint2",&(joint[1]));
                    GetProtocolDataDouble(buf,"joint3",&(joint[2]));
                    GetProtocolDataDouble(buf,"joint4",&(joint[3]));
                    GetProtocolDataDouble(buf,"joint5",&(joint[4]));
                    GetProtocolDataDouble(buf,"joint6",&(joint[5]));
                    GetProtocolDataDouble(buf,"joint7",&(joint[6]));
                
                    SETJOINT7(target,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]);
                    SETSPEED(sp,vel,vel);
                    ret=HYYRobotBase::MoveA(target,sp,NULL3);
                    if (0==ret)
                    {
                        HYYRobotBase::robpose pospose;
                        HYYRobotBase::GetCurrentTargetCartesian(NULL,NULL,&pospose,0);
                        double x,y,z,k,p,s;
                        GetProtocolDataDouble(buf,"x",&x);
                        GetProtocolDataDouble(buf,"y",&y);
                        GetProtocolDataDouble(buf,"z",&z);
                        GetProtocolDataDouble(buf,"k",&k);
                        GetProtocolDataDouble(buf,"p",&p);
                        GetProtocolDataDouble(buf,"s",&s);
                        pospose=Offs(&pospose,x,y,z,k,p,s);
                        ret=HYYRobotBase::MoveL(&pospose,sp,NULL3);
                    }
                }
                break;
            case 3:
                {
                    double vel=0.1;
                    GetProtocolDataDouble(buf,"vel",&vel);
                    GetProtocolDataDouble(buf,"joint1",&(joint[0]));
                    GetProtocolDataDouble(buf,"joint2",&(joint[1]));
                    GetProtocolDataDouble(buf,"joint3",&(joint[2]));
                    GetProtocolDataDouble(buf,"joint4",&(joint[3]));
                    GetProtocolDataDouble(buf,"joint5",&(joint[4]));
                    GetProtocolDataDouble(buf,"joint6",&(joint[5]));
                    GetProtocolDataDouble(buf,"joint7",&(joint[6]));
                    SETJOINT7(target,joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]);
                    SETSPEED(sp,vel,vel);
                    ret=HYYRobotBase::MoveA(target,sp,NULL3);
                }
                break;
            case 4:
                ret=HYYRobotBase::get_robot_move_state(0);
                break;
            case 5:
                GetCurrentJoint(joint, 0);
                ret=0;
                flag=1;
                break;
        }

    }
    else
    {
        ret=-1000;
    }

    memset(buf,0,sizeof(buf));            
    uint16_t offset=0;
    offset = SetProtocolDataInt(buf,offset, "value", ret);
    if(1==flag)
    {
        offset = SetProtocolDataDouble(buf,offset, "joint1", joint[0]);
        offset = SetProtocolDataDouble(buf,offset, "joint2", joint[1]);
        offset = SetProtocolDataDouble(buf,offset, "joint3", joint[2]);
        offset = SetProtocolDataDouble(buf,offset, "joint4", joint[3]);
        offset = SetProtocolDataDouble(buf,offset, "joint5", joint[4]);
        offset = SetProtocolDataDouble(buf,offset, "joint6", joint[5]);
        offset = SetProtocolDataDouble(buf,offset, "joint7", joint[6]);
    }

    ret=write(fd,buf,offset);
    if ( ret<= 0)
    {
        if (0==ret)
        {
            return ret;
        }
        else if (errno==EINTR||errno==EWOULDBLOCK||errno==EAGAIN)
        {
            return 1;
        }
        else if (-1==ret&&errno==ECONNRESET)
        {
            return 0;
        }
        ROBOT_ERROR("command_action: Server Recieve Data Failed! ret=%d,errno=%d(%s)\n",ret,errno,strerror(errno));
        return ret;
    }
    return ret;
}

void PluginMain()
{
    ROBOT_INFO("PluginMain:SubwayPlugin init\n");

    t=std::thread([](){
    
    TCPConcurrentServer(NULL, 6969, subway_deal);
     
    });

    if (t.joinable()) {
        t.detach(); // 分离线程
    }

}
