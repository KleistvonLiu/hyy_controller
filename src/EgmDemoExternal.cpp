#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要

/*
//轨迹修正程序(在控制器外运行)
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
#include "communication_protocol.h"
int main()
{
	int sockfd;  
	struct sockaddr_in adr_srvr;  
	struct sockaddr_in adr_clnt;  
	adr_srvr.sin_family=AF_INET;  
  	adr_srvr.sin_port=htons(6680);  
  	adr_srvr.sin_addr.s_addr=htonl(INADDR_ANY); 
  	bzero(&(adr_srvr.sin_zero),8);  
  	sockfd=socket(AF_INET,SOCK_DGRAM,0);  
  	if(sockfd==-1)
	{  
    	printf("socket error!");  
    	return 0;
  	}  
	bind(sockfd,(struct sockaddr *)&adr_srvr,sizeof(adr_srvr));  
	double t=0;
    double target[6]={0,0,0,0,0,0};
	uint8_t buf[1024];
	uint64_t ipoc=0;
	double cartesian_start[6];
	double joint[6];
	//得到开始引导位置
	int adr_clnt_len=sizeof(struct sockaddr);
	int len=recvfrom(sockfd, buf, 1024, 0, (struct sockaddr *)&adr_clnt, &adr_clnt_len);
	if (!IsProtocolRight(buf,len))
	{
		printf("recv failure\n");
		return 0;
	}
	GetProtocolData(buf,"IPOC",&ipoc);
	GetProtocolDataDouble(buf,"CartesianX",&(cartesian_start[0]));
	GetProtocolDataDouble(buf,"CartesianY",&(cartesian_start[1]));
	GetProtocolDataDouble(buf,"CartesianZ",&(cartesian_start[2]));
	GetProtocolDataDouble(buf,"CartesianK",&(cartesian_start[3]));
	GetProtocolDataDouble(buf,"CartesianP",&(cartesian_start[4]));
	GetProtocolDataDouble(buf,"CartesianS",&(cartesian_start[5]));
	uint8_t buf_out[1024];
	uint16_t offset=0;
	while (1)
	{
		//接收数据（兼定时功能）
		len=recvfrom(sockfd, buf, 1024, 0, (struct sockaddr *)&adr_clnt, &adr_clnt_len);
		if (len<=0)
		{
			printf("recv failure\n");
			return 0;
		}
		GetProtocolData(buf,"IPOC",&ipoc);
		//获取关节实时位置
		GetProtocolDataDouble(buf,"Joint0",&(joint[0]));
		GetProtocolDataDouble(buf,"Joint1",&(joint[1]));
		GetProtocolDataDouble(buf,"Joint2",&(joint[2]));
		GetProtocolDataDouble(buf,"Joint3",&(joint[3]));
		GetProtocolDataDouble(buf,"Joint4",&(joint[4]));
		GetProtocolDataDouble(buf,"Joint5",&(joint[5]));

		if (t<=5)
		{
			//计算引导的绝对位置
		    target[4]=0.1*cos(2*3.14*0.2*t)-0.1;
			//协议打包
			offset=0;
			//offset=SetProtocolDataInt(buf_out,offset,"IsAbsoluteGuide",1);
			offset=SetProtocolData(buf_out,offset,"IPOC",ipoc);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetX",target[0]);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetY",target[1]);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetZ",target[2]);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetK",target[3]);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetP",target[4]);
			offset=SetProtocolDataDouble(buf_out,offset,"CartesianTargetS",target[5]);
			//发送引导数据
			sendto(sockfd,buf_out,offset,0,(struct sockaddr *)&adr_clnt,sizeof(adr_clnt));  
		    t+=0.004;
		}

	}
    return 0;
}
*/
int EgmDemoExternal()
{
    //创建外部修正数据区
	int ret=EgmCreate(2,4,NULL, NULL,0);
	if (0!=ret)
    {
        printf("EGMCreate:%d\n",ret);
        return 0;
    }
    //开始修正
	ret=EgmStart("192.168.8.129",6680,1, 0);
    if (0!=ret)
    {
	    printf("EGMStart:%d\n",ret);
        return 0;
    }
	Rsleep(1000);
    //移动机器人
	robpose pospose;
	GetCurrentTargetCartesian(NULL,NULL, &pospose, 0);
    robpose pospose1=Offs(&pospose,0.1,0,0.05,0.02,0.01,-0.03);
	IMPORTSPEED(v10);
	while (robot_ok())
	{
	    MoveL(&pospose1,v10,NULL,NULL,NULL);
	    MoveL(&pospose,v10,NULL,NULL,NULL);
	}
	EgmStop(0);
	EgmDelete(0);
	return 0;
}