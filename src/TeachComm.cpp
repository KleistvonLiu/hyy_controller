
#include<stdio.h>  
#include<stdlib.h>  
#include<string.h>  
#include<sys/socket.h>  
#include<netinet/in.h>  
#include<arpa/inet.h>  
#include<netdb.h>  
#include<errno.h>  
#include<sys/types.h>  
#include<fcntl.h>//for open
#include<unistd.h>//for close
#include<math.h>
//#include "communication_protocol.h"
#include "Comm/Communication.h"
#include <thread>
using namespace HYYRobotBase;//仅c++需要
static bool out_flag=false;
void feadback_deal(int sock)
{
    int ret=0;
    uint8_t buf[1024];
    double joint[10];
    double cartesian[10];
    int tmp=0;
    char stmp[1024];
    while (!out_flag)
    {
        //接收反馈
        ret = recv(sock, buf, sizeof(buf), 0);   
        if ( ret<= 0)
        {
            if (0==ret)
            {
                break;
            }
            else if (errno==EINTR||errno==EWOULDBLOCK||errno==EAGAIN)
            {
                continue;
            }
            break;
        }
        if (!IsProtocolRight(buf,ret))
        {
            printf("IsProtocolRight error\n");
            continue;
        }
        //接收固定反馈
        printf("===============================================\n");
        //机器人0关节实际位置
        GetProtocolDataDouble(buf,"Robot0:Joint1",&(joint[0]));
        GetProtocolDataDouble(buf,"Robot0:Joint2",&(joint[1]));
        GetProtocolDataDouble(buf,"Robot0:Joint3",&(joint[2]));
        GetProtocolDataDouble(buf,"Robot0:Joint4",&(joint[3]));
        GetProtocolDataDouble(buf,"Robot0:Joint5",&(joint[4]));
        GetProtocolDataDouble(buf,"Robot0:Joint6",&(joint[5]));
        printf("Robot0:Joint:%f,%f,%f,%f,%f,%f\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
        //机器人0关节实际位置
        GetProtocolDataDouble(buf,"Robot0:X",&(cartesian[0]));
        GetProtocolDataDouble(buf,"Robot0:Y",&(cartesian[1]));
        GetProtocolDataDouble(buf,"Robot0:Z",&(cartesian[2]));
        GetProtocolDataDouble(buf,"Robot0:K",&(cartesian[3]));
        GetProtocolDataDouble(buf,"Robot0:P",&(cartesian[4]));
        GetProtocolDataDouble(buf,"Robot0:S",&(cartesian[5]));
        printf("Robot0:Cartesian:%f,%f,%f,%f,%f,%f\n",cartesian[0],cartesian[1],cartesian[2],cartesian[3],cartesian[4],cartesian[5]);
        GetProtocolDataInt(buf,"Robot0:MoveState",&(tmp));
        printf("Robot0:MoveState:%d\n",tmp);

        //接收命令反馈
        memset(stmp,0,sizeof(stmp));
        GetProtocolDataString(buf,"FeedbackCommand",stmp);
        if (0!=strcmp("null",stmp))
        {
            if (0==strcmp("set_robot_index",stmp))
            {
                if (0==GetProtocolDataInt(buf,"set_robot_index",&(tmp)))
                {
                    printf("set_robot_index:%d\n",tmp);
                }
            }else if (0==strcmp("set_robot_teach_coordinate",stmp))
            {
                if (0==GetProtocolDataInt(buf,"set_robot_teach_coordinate",&(tmp)))
                {
                    printf("set_robot_teach_coordinate:%d\n",tmp);
                }
            }else if (0==strcmp("get_robot_num",stmp))
            {
                if (0==GetProtocolDataInt(buf,"get_robot_num",&(tmp)))
                {
                    printf("get_robot_num:%d\n",tmp);
                }
            }else if (0==strcmp("robot_teach_enable",stmp))
            {
                if (0==GetProtocolDataInt(buf,"robot_teach_enable",&(tmp)))
                {
                    printf("robot_teach_enable:%d\n",tmp);
                }
            }else if (0==strcmp("robot_teach_move",stmp))
            {
                if (0==GetProtocolDataInt(buf,"robot_teach_move",&(tmp)))
                {
                    printf("robot_teach_move:%d\n",tmp);
                }
            }else if (0==strcmp("robot_teach_stop",stmp))
            {
                if (0==GetProtocolDataInt(buf,"robot_teach_stop",&(tmp)))
                {
                    printf("robot_teach_stop:%d\n",tmp);
                }
            }else if (0==strcmp("robot_teach_disenable",stmp))
            {
                if (0==GetProtocolDataInt(buf,"robot_teach_disenable",&(tmp)))
                {
                    printf("robot_teach_disenable:%d\n",tmp);
                }
            }else if (0==strcmp("get_robot_data_path",stmp))
            {
                memset(stmp,0,sizeof(stmp));
                if (0==GetProtocolDataString(buf,"get_robot_data_path",stmp))
                {
                    printf("get_robot_data_path:%s\n",stmp);
                }
            }else if (0==strcmp("start_robot_c_project",stmp))
            {
                if (0==GetProtocolDataInt(buf,"start_robot_c_project",&(tmp)))
                {
                    printf("start_robot_c_project:%d\n",tmp);
                }
            }else if (0==strcmp("suspend_robot_project",stmp))
            {
                if (0==GetProtocolDataInt(buf,"suspend_robot_project",&(tmp)))
                {
                    printf("suspend_robot_project:%d\n",tmp);
                }
            }else if (0==strcmp("continue_robot_project",stmp))
            {
                if (0==GetProtocolDataInt(buf,"continue_robot_project",&(tmp)))
                {
                    printf("continue_robot_project:%d\n",tmp);
                }
            }else if (0==strcmp("close_robot_c_project",stmp))
            {
                if (0==GetProtocolDataInt(buf,"close_robot_c_project",&(tmp)))
                {
                    printf("close_robot_c_project:%d\n",tmp);
                }
            }
        }
    }


}

int TeachComm()
{
    //建立命令socket client
    int sock6667 = socket(PF_INET, SOCK_STREAM, 0);
    if(sock6667 == -1)
    {
        printf("sock6667 failure\n");
        return -1;
    }
    struct sockaddr_in srv_addr6667;
    srv_addr6667.sin_family = AF_INET;
    srv_addr6667.sin_addr.s_addr = inet_addr("192.168.137.99");
    srv_addr6667.sin_port = htons(6667);
    if(-1 == connect(sock6667, (struct sockaddr*)&srv_addr6667, sizeof(struct sockaddr)))
    {
        printf("sock6667 connect failure\n");
        return -2;
    }
    //建立反馈socket client
    int sock6668 = socket(PF_INET, SOCK_STREAM, 0);
    if(sock6668 == -1)
    {
        printf("sock6668 failure\n");
        return -1;
    }
    struct sockaddr_in srv_addr6668;
    srv_addr6668.sin_family = AF_INET;
    srv_addr6668.sin_addr.s_addr = inet_addr("192.168.137.99");
    srv_addr6668.sin_port = htons(6668);
    if(-1 == connect(sock6668, (struct sockaddr*)&srv_addr6668, sizeof(struct sockaddr)))
    {
        printf("sock6668 connect failure\n");
        return -2;
    }
    sleep(3);
    printf("start\n");
    //处理反馈
    std::thread fb(&feadback_deal, sock6668);

    uint8_t buf[1024];
    uint16_t offset=0;
    int count=0;
    bool set_flag=false;
    //下发命令
    while(true)
    {
        sleep(1);
        //模拟触发命令
        if (0==count)//设置要操作机器人的索引
        {
            printf("设置要操作机器人的索引\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","set_robot_index");
			offset=SetProtocolDataInt(buf,offset,"robot_index",0);
            set_flag=true;
        }else if (5==count)//设置当前操作机器人的坐标系
        {
            printf("设置当前操作机器人的坐标系\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","set_robot_teach_coordinate");
			offset=SetProtocolDataInt(buf,offset,"frame",1);
            set_flag=true;
        }else if (10==count)//获取机器人数目
        {
            printf("获取机器人数目\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","get_robot_num");
            set_flag=true;
        }else if (15==count)//设置当前操作机器人的最大示教速度
        {
            printf("设置当前操作机器人的最大示教速度\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","set_robot_teach_velocity");
			offset=SetProtocolDataDouble(buf,offset,"vel_percent",0.2);
            set_flag=true;
        }else if (20==count)//使能当前操作机器人
        {
            printf("使能当前操作机器人\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","robot_teach_enable");
            set_flag=true;
        }else if (22==count)//示教当前操作机器人，运动相对坐标系包括：关节坐标系，基座坐标系，工件坐标系，工具坐标系，由当前设置的坐标系决定。
        {
            printf("示教当前操作机器人\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","robot_teach_move");
            offset=SetProtocolDataInt(buf,offset,"axis_index",2);
            offset=SetProtocolDataInt(buf,offset,"dir",-1);
            set_flag=true;
        }else if (25==count)//停止当前操作机器人的示教移动
        {
            printf("停止当前操作机器人的示教移动\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","robot_teach_stop");
            set_flag=true;
        }else if (27==count)//非使能当前操作机器人
        {
            printf("非使能当前操作机器人\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","robot_teach_disenable");
            set_flag=true;
        }else if (30==count)//获取机器人系统使用的数据所在路径
        {
            printf("获取机器人系统使用的数据所在路径\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","get_robot_data_path");
            set_flag=true;
        }else if (32==count)//启动机器人C项目
        {
            printf("启动机器人C项目\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","start_robot_c_project");
            offset=SetProtocolDataString(buf,offset,"project_name","gohome");
            set_flag=true;
        }else if (34==count)//暂停机器人c/lua项目
        {
            printf("暂停机器人c/lua项目\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","suspend_robot_project");
            set_flag=true;
        }else if (37==count)//暂停后继续运行机器人c/lua项目
        {
            printf("暂停后继续运行机器人c/lua项目\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","continue_robot_project");
            set_flag=true;
        }
        else if (40==count)//关闭机器人c项目
        {
            printf("关闭机器人c项目\n");
            offset=0;
            memset(buf,0,sizeof(buf));
			offset=SetProtocolDataString(buf,offset,"function","close_robot_c_project");
            set_flag=true;
        }else if (count >50)
        {
            break;
        }

        count++;
        if (set_flag)//有触发命令时发送
        {
            set_flag=false;
            int ret = send(sock6667, buf, offset, 0);
            if ( ret<= 0)
            {
                if (0==ret)
                {
                    break;
                }
                else if (errno==EINTR||errno==EWOULDBLOCK||errno==EAGAIN)
                {
                    continue;
                }
                break;
            }
        }

    }

    //关闭连接
    shutdown(sock6667,SHUT_RDWR);
    close(sock6667);
    out_flag=true;
    if(fb.joinable()) 
    {
        fb.join();
    }
    shutdown(sock6668,SHUT_RDWR);
    close(sock6668);
    sleep(3);
    return 0;
}
