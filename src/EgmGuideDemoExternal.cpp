#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要

/*
//轨迹引导程序(在控制器外运行)
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

#define R_PI 3.1415926535898
void tr2rpy(double R[3][3], double* rpy, int flag)
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

void rpy2tr(double* rpy, double R[3][3], int flag)
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

void Rmulti(double R0[3][3],double R1[3][3],double Rres[3][3])
{
	int i=0;
	int j=0;
	int k=0;
	for (i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Rres[i][j]=0;
			for (k=0;k<3;k++)
			{
				Rres[i][j]=Rres[i][j]+R0[i][k]*R1[k][j];
			}
		}
	}
}

void RMultVec(double(*R)[3], double* v, double * vres)
{
	vres[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
	vres[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
	vres[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

void Offs(const double* rpose, double x, double y, double z, double k, double p, double s,double* result)
{
	result[0]=rpose[0]+x;
	result[1]=rpose[1]+y;
	result[2]=rpose[2]+z;
	double kps[3]={rpose[3],rpose[4],rpose[5]};
	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3]={k,p,s};
	rpy2tr(kps,  R0, 2);
	rpy2tr(rpy,  R1, 2);
	Rmulti(R1,R0,Rres);
	tr2rpy(Rres, &(result[3]), 2);
}

void OffsRel(const double* rpose, double x, double y, double z, double k, double p, double s,double* result)
{
	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3]={k,p,s};
	double kps[3]={rpose[3],rpose[4],rpose[5]};
	rpy2tr(kps,  R0, 2);
	rpy2tr(rpy,  R1, 2);
	Rmulti(R0,R1,Rres);
	tr2rpy(Rres, &(result[3]), 2);

	double _pos[3];
	double pos[3]={x,y,z};
	RMultVec(R0, pos, _pos);
	result[0]=rpose[0]+_pos[0];
	result[1]=rpose[1]+_pos[1];
	result[2]=rpose[2]+_pos[2];
}

//笛卡尔引导
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
	double joint_start[10];
	double joint[10];
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
	GetProtocolDataDouble(buf,"Joint0",&(joint_start[0]));
	GetProtocolDataDouble(buf,"Joint1",&(joint_start[1]));
	GetProtocolDataDouble(buf,"Joint2",&(joint_start[2]));
	GetProtocolDataDouble(buf,"Joint3",&(joint_start[3]));
	GetProtocolDataDouble(buf,"Joint4",&(joint_start[4]));
	GetProtocolDataDouble(buf,"Joint5",&(joint_start[5]));

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
			//未叠加初值，不能设置IsAbsoluteGuide
		    //target[2]=0.1*cos(2*3.14*0.2*t)-0.1;
			//叠加初值，设置IsAbsoluteGuide
			OffsRel(cartesian_start, 0, 0, 0,0.1*cos(2*3.14*0.2*t)-0.1, 0, 0,target);
			//协议打包
			offset=0;
			offset=SetProtocolDataInt(buf_out,offset,"IsAbsoluteGuide",1);
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

//关节引导
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
	double joint_start[10];
	double joint[10];
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
	GetProtocolDataDouble(buf,"Joint0",&(joint_start[0]));
	GetProtocolDataDouble(buf,"Joint1",&(joint_start[1]));
	GetProtocolDataDouble(buf,"Joint2",&(joint_start[2]));
	GetProtocolDataDouble(buf,"Joint3",&(joint_start[3]));
	GetProtocolDataDouble(buf,"Joint4",&(joint_start[4]));
	GetProtocolDataDouble(buf,"Joint5",&(joint_start[5]));

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
			//未叠加初值，不能设置IsAbsoluteGuide
		    //target[2]=0.1*cos(2*3.14*0.2*t)-0.1;

			//叠加初值，设置IsAbsoluteGuide
			memcpy(target,joint_start,6*sizeof(double));
			target[2]=joint_start[2]+0.1*cos(2*3.14*0.2*t)-0.1;
			//协议打包
			offset=0;
			offset=SetProtocolDataInt(buf_out,offset,"IsAbsoluteGuide",1);
			offset=SetProtocolData(buf_out,offset,"IPOC",ipoc);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget0",target[0]);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget1",target[1]);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget2",target[2]);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget3",target[3]);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget4",target[4]);
			offset=SetProtocolDataDouble(buf_out,offset,"JointTarget5",target[5]);
			//发送引导数据
			sendto(sockfd,buf_out,offset,0,(struct sockaddr *)&adr_clnt,sizeof(adr_clnt));  
		    t+=0.004;
		}

	}
    return 0;
}
*/

int EgmGuideDemoExternal()
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
	//启用外部引导
	EgmGuideMove(0);
	Rsleep(100000000);



	EgmStop(0);
	EgmDelete(0);
	return 0;
}