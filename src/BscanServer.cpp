#include "user/BscanServer.h"
#include "math.h"
#include <sys/socket.h>
#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdlib.h>        // for exit
#include <string.h>        // for bzero
#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include "sys/wait.h"
#include <sys/epoll.h>
#include <arpa/inet.h>
#include <errno.h>
#define R_PI 3.1415926535898
namespace bscan_server
{
void BscanServer::StartBscanServer(int cycle_times,HYYRobotBase::tool* tool,HYYRobotBase::wobj* wobj)
{
    printf("StartBscanServer start\n");
    HYYRobotBase::init_tool(&tool_, NULL, 1, NULL);
    if (NULL!=tool)
    {
        tool_=*tool;
    }
    HYYRobotBase::init_wobj(&wobj_, NULL, NULL, NULL, 0, 0, 0);
    if (NULL!=wobj)
    {
        wobj_=*wobj;
    }
    cycle_times_=cycle_times;

    printf("StartBscanServer start socket concurrent server\n");
	//创建server
	if (0!=tcp_concurrent_server(NULL, 8888))
	{
		printf("StartBscanServer: tcp_concurrent_server failure\n");
	}

	if (thread_.joinable())
	{
		thread_.join();
	}
	printf("StartBscanServer stop\n");
}

int BscanServer::cmd_server(int fd)
{
	int ret=0;
    uint8_t buf[1024];
    while (true)
    {
        memset(buf,0,sizeof(buf));
		ret=recv(fd,(void*)buf,1024,0);
        if (0==ret)
        {
            break;
        }
        if (ret>0)
        {
            data_lock_.lock();
            //判断数据格式是否正确
            if (HYYRobotBase::IsProtocolRight(buf,ret))
            {
                //解析数据
                int func=0;
                HYYRobotBase::GetProtocolDataInt(buf,"function",&func);
                std::vector<double> tmp;
                int cmd=0;
                int err=0;
                double v=0;
                switch(func)
                {
                case 1://压入数据
                    tmp.clear();
                    err=HYYRobotBase::GetProtocolDataDouble(buf,"x",&v);
                    tmp.push_back(v);
                    err+=HYYRobotBase::GetProtocolDataDouble(buf,"y",&v);
                    tmp.push_back(v);
                    err+=HYYRobotBase::GetProtocolDataDouble(buf,"z",&v);
                    tmp.push_back(v);
                    err+=HYYRobotBase::GetProtocolDataDouble(buf,"rx",&v);
                    tmp.push_back(v);
                    err+=HYYRobotBase::GetProtocolDataDouble(buf,"ry",&v);
                    tmp.push_back(v);
                    err+=HYYRobotBase::GetProtocolDataDouble(buf,"rz",&v);
                    tmp.push_back(v);
                    if (0==err)
                    {
                        server_data_.push_back(tmp);
                        printf("function:%d;data_size:%ld\n",func,server_data_.size());
                    }
                    else
                    {
                        input_feedback=-1;
                    }
                    break;
                case 2://机器人控制
                    err=HYYRobotBase::GetProtocolDataInt(buf,"cmd",&cmd);
                    printf("function:%d;cmd:%d\n",func,cmd);
                    switch(cmd)
                    {
                    case _start:
                        if (_stop==server_state_)
                        {
                            server_state_=_start;
							if (thread_.joinable())
							{
								thread_.join();
							}
                            thread_=std::thread(&BscanServer::state_server, this);
                            usleep(2000);
                        }
                        else
                        {
                            input_feedback=-5;
                        }
                        break;
                    case _stop:
                        stop_server();
                        break;
                    case _pause:
                        server_state_=_pause;
                        pause_server();
                        break;
                    case _continue:
                        server_state_=_continue;
                        continue_server();
                        break;
                    default:
                        input_feedback=-4;
                        break;
                    }
                    break;
                case 3://清理缓存区
                    server_data_.clear();
                    printf("function:%d;data_size:%ld\n",func,server_data_.size());
                    break;
                default:
                    input_feedback=-3;
                }
            }
            else
            {
                input_feedback=-2;
            }
            data_lock_.unlock();
        }
    }
	return ret;
}

void BscanServer::feedback_server(void)
{
	int ret=0;
	int fd=feedbacd_fd_;
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer,0,cycle_times_);//定时周期为1倍总线周期
    uint8_t buf[1024];
    uint16_t offset=0;
    server_state_=_stop;
    uint8_t server_state_1=_stop;
    //数据反馈
    while (true)
    {
        userTimer(&timer);
        data_lock_.lock();
        if ((server_state_1!=server_state_)&&(_stop==server_state_))
        {
            if (thread_.joinable())
            {
                thread_.join();
            }
            server_state_1=server_state_;
        }
        memset(buf,0,sizeof(buf));
        offset=0;
        double joint[10];
        HYYRobotBase::GetCurrentJoint(joint, 0);
        double cartesian[6];
        HYYRobotBase::GetCurrentCartesian(&tool_,&wobj_, (HYYRobotBase::robpose*)cartesian, 0);
        double R[3][3];
        rpy2tr(&(cartesian[3]), R, 2);
        tr2rpy(R, &(cartesian[3]), 3);
        offset = HYYRobotBase::SetProtocolDataInt(buf,offset, "RobotState", HYYRobotBase::get_robot_move_state(0));
        offset = HYYRobotBase::SetProtocolDataInt(buf,offset, "InputFeedback", input_feedback);
        offset = HYYRobotBase::SetProtocolDataInt(buf,offset, "ServerState", server_state_);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "x", cartesian[0]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "y", cartesian[1]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "z", cartesian[2]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "rx", cartesian[3]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "ry", cartesian[4]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "rz", cartesian[5]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint1", joint[0]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint2", joint[1]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint3", joint[2]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint4", joint[3]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint5", joint[4]);
        offset = HYYRobotBase::SetProtocolDataDouble(buf,offset, "joint6", joint[5]);

		input_feedback=0;
        data_lock_.unlock();
         //发送数据到client
		ret = send(fd,buf, offset,MSG_NOSIGNAL);
        if (ret<0)
        {
            printf("StartBscanServer: send failure\n");
			return;
        }
        if (0==ret)
        {
            break;
        }
    }
	return;
}

void BscanServer::state_server()
{
        //
        SETSPEED(sp,0.1,0.01);
        HYYRobotBase::robpose rpose[100];
        int num=server_data_.size();
        int i=0;
        double R[3][3];
        for (i=0;i<num;i++)
        {
            rpy2tr(&(server_data_[i][3]), R, 3);
		    tr2rpy(R, &(server_data_[i][3]), 2);
            rpose[i].xyz[0]=server_data_[i][0];
            rpose[i].xyz[1]=server_data_[i][1];
            rpose[i].xyz[2]=server_data_[i][2];
            rpose[i].kps[0]=server_data_[i][3];
            rpose[i].kps[1]=server_data_[i][4];
            rpose[i].kps[2]=server_data_[i][5];
        }

		// IMPORTJOINT(jbscan1);
        // MoveA(jbscan1,sp,NULL,NULL,NULL);
		IMPORTJOINT(jbscan);
        MoveA(jbscan,sp,NULL,NULL,NULL);
		// IMPORTJOINT(jbscan2);
        // MoveA(jbscan2,sp,NULL,NULL,NULL);
		MoveL(&(rpose[0]),sp,NULL,&tool_, &wobj_);
		MoveL(&(rpose[1]),sp,NULL,&tool_, &wobj_);
	    //初始化力控制
	    int ret=HYYRobotBase::SFCInit("SFC", 0, 1, NULL, &wobj_, 0);
	    if (0!=ret)
        {
            server_state_=_stop;
            return;
        }
        //设置力控参数
        double M[6]={10,10,2,10,10,10};
        double B[6]={1000,1000,200,1000,1000,1000};
        double K[6]={5000,5000,0,10000,10000,10000};
	    HYYRobotBase::SFCSetAdmittanceCtrlParam("SFC", M, B, K);
        //设置期望力（非零力控，零导纳控制）
	    double target_force[6]={0,0,-2,0,0,0};
	    HYYRobotBase::SFCSetTargetForce("SFC", target_force);
	    //开启力控制
	    ret=HYYRobotBase::SFCStart("SFC");
        if (0!=ret)
        {
            server_state_=_stop;
            return;
        }
		HYYRobotBase::TorqueSensorOpenBias(HYYRobotBase::GetTorqueSensorName(0,NULL));
		MoveL(&(rpose[2]),sp,NULL,&tool_, &wobj_);
        if (num>1)
        {
            MoveS(&(rpose[3]), num-4, sp, NULL, &tool_, &wobj_);
        }  
        HYYRobotBase::Rsleep(200);
        //关闭力控
	    ret=HYYRobotBase::SFCEnd("SFC");
		MoveL(&(rpose[num-1]),sp,NULL,&tool_, &wobj_);
		MoveA(jbscan,sp,NULL,NULL,NULL);
        server_state_=_stop;
}
void BscanServer::stop_server()
{
    HYYRobotBase::DeviceStopRun();
}
void BscanServer::pause_server()
{

}
void BscanServer::continue_server()
{

}

void BscanServer::tr2rpy(double R[3][3], double* rpy, int flag)
{
	double eps = 0.000001;
	double s = 0.0;
	double c = 0.0;
	double cp = 0.0;
	//double Q = 0.0;
	double m=0.0;

	//matlab 四元数
	double qs=0;
	double kx=0;
	double ky=0;
	double kz=0;
	double kx1=0;
	double ky1=0;
	double kz1=0;
	int add=0;
	double nm=0;
	switch (flag)
	{
	case 0://XYZ order
		if (fabs(R[2][2])<eps&&fabs(R[1][2])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] =R_PI/2;// atan2(R[0][2], R[2][2]);//pitch
			rpy[2] = atan2(R[1][0], R[1][1]);//yaw is sum of roll+yaw

		}
		else
		{
			rpy[0] = atan2(-R[1][2], R[2][2]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(R[0][2], c*R[2][2] - s*R[1][2]);//pitch
			rpy[2] = atan2(-R[0][1], R[0][0]);//yaw

		}
		break;
	case 1://ZYX order
		if (fabs(R[0][0])<eps&&fabs(R[1][0])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] = atan2(-R[2][0], R[0][0]);//pitch
			rpy[2] = atan2(-R[1][2], R[1][1]);//yaw is sum of roll+yaw
		}
		else
		{
			rpy[0] = atan2(R[1][0], R[0][0]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(-R[2][0], c*R[0][0] + s*R[1][0]);//pitch
			rpy[2] = atan2(s*R[0][2] - c*R[1][2], c*R[1][1] - s*R[0][1]);//yaw

		}
		break;
	case 2://XYZ HLHN J.CRAIG method
		rpy[1] = atan2(-R[2][0],sqrt(R[0][0]*R[0][0]+R[1][0]*R[1][0])); //pitch
		if (fabs(fabs(rpy[1]) - R_PI / 2.0) < eps)
		{
			if (rpy[1] > 0) //pi/2
			{
				rpy[1] = R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = atan2(R[0][1],R[1][1]);
			}
			else//-pi/2
			{
				rpy[1] = -R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = -atan2(R[0][1], R[1][1]);
			}
		}
		else
		{
			cp = cos(rpy[1]);
			rpy[2] = atan2(R[1][0]/cp,R[0][0]/cp);
			rpy[0] = atan2(R[2][1]/cp, R[2][2]/cp);
		}
		break;
	case 3://orocos
		cp=(R[0][0]+R[1][1]+R[2][2]-1)/2.0;
		if (cp>1-(eps*(1e-6)))
		{
        	rpy[0] = 0;
        	rpy[1] = 0;
        	rpy[2] = 0;
		}else if (cp<-1+(eps*(1e-6)))
		{
			rpy[0] = sqrt( (R[0][0]+1.0)/2);
			rpy[1] = sqrt( (R[1][1]+1.0)/2);
			rpy[2] = sqrt( (R[2][2]+1.0)/2);
			if ( R[0][2]< 0) rpy[0] =-rpy[0] ;
			if ( R[2][1]< 0) rpy[1] =-rpy[1] ;
			if ( rpy[0]*rpy[1]*R[0][1] < 0) rpy[0]=-rpy[0];  // this last line can be necessary when z is 0
			// z always >= 0
			// if z equal to zero
			rpy[0] = rpy[0]*R_PI;
			rpy[1] = rpy[1]*R_PI;
			rpy[2] = rpy[2]*R_PI;
		}
		else
		{
//			Q=acos(cp);
//
//	    	rpy[0] = Q*0.5*(R[2][1]-R[1][2])/sin(Q);
//	    	rpy[1] = Q*0.5*(R[0][2]-R[2][0])/sin(Q);
//	    	rpy[2] = Q*0.5*(R[1][0]-R[0][1])/sin(Q);


			double angle;
			double mod_axis;
			double axisx, axisy, axisz;
			axisx = R[2][1]-R[1][2];
			axisy = R[0][2]-R[2][0];
			axisz = R[1][0]-R[0][1];
			mod_axis = sqrt(axisx*axisx+axisy*axisy+axisz*axisz);
			angle = atan2(mod_axis/2,cp);
			rpy[0] = angle*(R[2][1]-R[1][2])/mod_axis;
			rpy[1] = angle*(R[0][2]-R[2][0])/mod_axis;
			rpy[2] = angle*(R[1][0]-R[0][1])/mod_axis;
		}


		break;
	case 4://单位四元数
			m=0.5*sqrt(1+R[0][0]+R[1][1]+R[2][2]);
			if (fabs(m)<eps)
			{
				break;
			}
			rpy[3]=m;
			rpy[0]=(R[2][1]-R[1][2])/(4*rpy[3]);
			rpy[1]=(R[0][2]-R[2][0])/(4*rpy[3]);
			rpy[2]=(R[1][0]-R[0][1])/(4*rpy[3]);
			break;
	case 5: //matlab 机器人工具箱
		 qs = sqrt((R[0][0]+R[1][1]+R[2][2])+1)/2.0;
		 kx = R[2][1] - R[1][2];   // Oz - Ay
		 ky =R[0][2] - R[2][0];   //Ax - Nz
		 kz = R[1][0] - R[0][1];   // Ny - Ox

		 if ((R[0][0]>=R[1][1]) && (R[0][0]>= R[2][2]) )
		 {
				kx1 = R[0][0]- R[1][1] - R[2][2] + 1; // Nx - Oy - Az + 1
				ky1 = R[1][0] + R[0][1];          // Ny + Ox
				kz1 = R[2][0]+ R[0][2];          // Nz + Ax
				if (kx >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		 else if  (R[1][1]>= R[2][2])
		 {
		        kx1 = R[1][0]+ R[0][1];          //Ny + Ox
		        ky1 = R[1][1]- R[0][0]- R[2][2] + 1; // Oy - Nx - Az + 1
		        kz1 = R[2][1] + R[1][2];          // Oz + Ay
				if (ky >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		else
		{
				kx1 = R[2][0] + R[0][2];          // Nz + Ax
				ky1 = R[2][1]+ R[1][2];          //Oz + Ay
				kz1 = R[2][2] - R[0][0] - R[1][1]+ 1; // Az - Nx - Oy + 1
				if (kz >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		}
		 if( add)
		 {
				 kx = kx + kx1;
				 ky = ky + ky1;
				 kz = kz + kz1;
		 }
		 else
		 {
				 kx = kx - kx1;
				 ky = ky - ky1;
				 kz = kz - kz1;
		 }
		 nm=sqrt(kx*kx+ky*ky+kz*kz);
		 if (nm<0.000001)
		 {
			 	 rpy[3]=1;rpy[0]=0;rpy[1]=0;rpy[2]=0;

		 }
		 else
		 {
			 	 s=sqrt(1-qs*qs)/nm;
			 	rpy[3]=qs;rpy[0]=s*kx;rpy[1]=s*ky;rpy[2]=s*kz;

		 }
		break;
	default:
		break;
	}

}

void BscanServer::rpy2tr(double* rpy, double R[3][3], int flag)
{
	double sx = 0;
	double cx = 0;
	double sy = 0;
	double cy = 0;
	double sz = 0;
	double cz = 0;

	double Q = 0;
	double k[3];
	double v=0;

	/*
	Rx = [
	1   0    0
	0   cx  -sx
	0   sx   cxtcp_concurrent_server(const char* ip, int port)
	0   1   0
	-sy  0   cy
	];
	Rz = [
	cz  -sz  0
	sz   cz  0
	0    0   1
	];
	*/
	switch (flag)
	{
	case 0://XYZ order
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rx*Ry*Rz
		R[0][0] = cy*cz; R[0][1] = -cy*sz; R[0][2] = sy;
		R[1][0] = cx*sz + cz*sx*sy; R[1][1] = cx*cz - sx*sy*sz; R[1][2] = -cy*sx;
		R[2][0] = sx*sz - cx*cz*sy; R[2][1] = cz*sx + cx*sy*sz; R[2][2] = cx*cy;
		break;
	case 1://ZYX order
		sx = sin(rpy[2]);
		cx = cos(rpy[2]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[0]);
		cz = cos(rpy[0]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 2://XYZ HLHN J.CRAIG method
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 3:
		Q=sqrt(rpy[0]*rpy[0]+rpy[1]*rpy[1]+rpy[2]*rpy[2]);
		if (Q<1e-10)
		{
	        R[0][0] = 1;	   R[0][1] = 0;	 R[0][2] = 0;
	        R[1][0] = 0;    R[1][1] = 1;   R[1][2] = 0;
	        R[2][0] = 0;	   R[2][1] = 0;   R[2][2] = 1;
        	break;
		}
        k[0]=rpy[0]/Q;
        k[1]=rpy[1]/Q;
        k[2]=rpy[2]/Q;
        sx=sin(Q);cx=cos(Q);v=1-cx;
        R[0][0] = k[0]*k[0]*v + cx;			R[0][1] = k[0]*k[1]*v - k[2]*sx;	R[0][2] = k[0]*k[2]*v + k[1]*sx;
        R[1][0] = k[0]*k[1]*v + k[2]*sx;	R[1][1] = k[1]*k[1]*v + cx;			R[1][2] = k[1]*k[2]*v - k[0]*sx;
        R[2][0] = k[0]*k[2]*v - k[1]*sx;	R[2][1] = k[1]*k[2]*v + k[0]*sx;	R[2][2] = k[2]*k[2]*v + cx;
		break;
	case 4://单位四元数
			R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
			break;
	case 5: //matlab 机器人工具箱
		    R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
		break;
	default:
		break;
	}

}

int BscanServer::tcp_concurrent_server(const char* ip, int port)
{
	struct sockaddr_in server_addr;
	bzero(&server_addr,sizeof(server_addr));
	server_addr.sin_family = PF_INET;
	if (NULL==ip)
	{
		server_addr.sin_addr.s_addr = htons(INADDR_ANY);
	}
	else
	{
		server_addr.sin_addr.s_addr = inet_addr(ip);
	}
	server_addr.sin_port = htons(port);

	int server_socket = socket(PF_INET,SOCK_STREAM,0);
	if( server_socket < 0)
	{
	    printf("TCPConcurrentServer: socket failure!\n");
	    return -1;
	}
    //设置为非阻塞接收
	int res = fcntl(server_socket , F_GETFL , 0);
    fcntl(server_socket , F_SETFL , res | O_NONBLOCK);

	if( bind(server_socket,(struct sockaddr*)&server_addr,sizeof(server_addr)))
	{
	    printf("TCPConcurrentServer: bind failure!\n");
	    close(server_socket);
	    return -2;
	}

	if ( listen(server_socket, 5) )
	{
	    printf("TCPConcurrentServer: listen failure!\n");
	    close(server_socket);
	    return -3;
	}

	/* 创建epoll对象 */
	int epoll_fd = epoll_create(1024);
	if (epoll_fd<0)
	{
		printf("TCPConcurrentServer: epoll_create failure!\n");
		return -4;
	}

	//准备一个事件结构体
	struct epoll_event event = {0};
	event.events = EPOLLIN | EPOLLET;
	event.data.fd = server_socket;   //data是一个共用体，除了fd还可以返回其他数据

	//ctl是监控listenfd是否有event被触发
	//如果发生了就把event通过wait带出。
	//所以，如果event里不标明fd，我们将来获取就不知道哪个fd
	epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_socket, &event);
	struct epoll_event revents[MAXFD];
	int nready=0;
	int i = 0;
	std::thread thread_[MAXFD];
	while(1)
	{
		//wait返回等待的event发生的数目
		//并把相应的event放到event类型的数组中
		nready = epoll_wait(epoll_fd, revents, MAXFD, -1);
		for(i=0;i<nready; i++)
		{
			//wait通过在events中设置相应的位来表示相应事件的发生
			//如果输入可用，那么下面的这个结果应该为真
			if((revents[i].events & EPOLLIN)==EPOLLIN)
			{
				//如果是listenfd有数据输入
				if(revents[i].data.fd == server_socket)
				{
					int sockfd = accept(server_socket, NULL, NULL);
					if(sockfd<0)
					{
						printf("TCPConcurrentServer: accept failure!\n");
						continue;
					}
					struct epoll_event event = {0};
					event.events = EPOLLIN | EPOLLET;
					event.data.fd = sockfd;
					epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sockfd, &event);

					feedbacd_fd_=sockfd;
					if (thread_[i].joinable())
					{
						thread_[i].join();
					}
					thread_[i]=std::thread(&BscanServer::feedback_server, this);

					printf("TCPConcurrentServer: client connect (%d)\n",sockfd);
				}
				else
				{
					int ret = cmd_server(revents[i].data.fd);
					if(0 >= ret)
					{
						//客户端链接已经断开
						if (thread_[i].joinable())
						{
							thread_[i].join();
						}
						printf("TCPConcurrentServer: client disconnect (%d)\n",revents[i].data.fd);
						close(revents[i].data.fd);
						epoll_ctl(epoll_fd, EPOLL_CTL_DEL, revents[i].data.fd, &revents[i]);
					}
				}
			}

		}
	}

	close(server_socket);
	close(epoll_fd);
	return 0;
}



}