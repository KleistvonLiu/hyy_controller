#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
//数据点处理子函数
robpose* mid_ropose(robpose* c,robpose* m,robpose* ret, double offset)
{
	if (c->xyz[0]>m->xyz[0])
	{
		ret->xyz[0]=c->xyz[0]-offset;
	}
	else
	{
		ret->xyz[0]=c->xyz[0]+offset;
	}
	if (c->xyz[1]>m->xyz[1])
	{
		ret->xyz[1]=c->xyz[1]-offset;
	}
	else
	{
		ret->xyz[1]=c->xyz[1]+offset;
	}
	ret->xyz[2]=c->xyz[2];
	ret->kps[0]=c->kps[0];
	ret->kps[1]=c->kps[1];
	ret->kps[2]=c->kps[2];
	return ret;
}
int MultiMoveDemo()
{
    //复杂轨迹关键点初始化
	robpose mid;
	double un=0.001;
	double z=0.010;
	double y=-0.005;
	double o=0.0176776695296637;
	double k=3.14159265358;
	double p=0;
	double s=0;
	SETPOSE(j0,-5*un,y,4*z,k,p,s);
	SETPOSE(j1,-5*un,y,z,k,p,s);
	SETPOSE(j2,20*un,y,z,k,p,s);
	SETPOSE(j3,45*un,y,z,k,p,s);
	SETPOSE(j4,70*un,y,z,k,p,s);
	SETPOSE(j5,95*un,y,z,k,p,s);
	SETPOSE(j6,120*un,y,z,k,p,s);
	SETPOSE(j7,145*un,y,z,k,p,s);
	SETPOSE(j8,170*un,y,z,k,p,s);
	SETPOSE(j9,195*un,y,z,k,p,s);	
	y=20*un;
	SETPOSE(j10,-5*un,y,z,k,p,s);
	SETPOSE(j11,20*un,y,z,k,p,s);
	SETPOSE(j12,45*un,y,z,k,p,s);
	SETPOSE(j13,70*un,y,z,k,p,s);
	SETPOSE(j14,95*un,y,z,k,p,s);
	SETPOSE(j15,120*un,y,z,k,p,s);
	SETPOSE(j16,145*un,y,z,k,p,s);
	SETPOSE(j17,170*un,y,z,k,p,s);
	SETPOSE(j18,195*un,y,z,k,p,s);
	y=45*un;
	SETPOSE(j19,-5*un,y,z,k,p,s);
	SETPOSE(j20,20*un,y,z,k,p,s);
	SETPOSE(j21,45*un,y,z,k,p,s);
	SETPOSE(j22,70*un,y,z,k,p,s);
	SETPOSE(j23,95*un,y,z,k,p,s);
	SETPOSE(j24,120*un,y,z,k,p,s);
	SETPOSE(j25,145*un,y,z,k,p,s);
	SETPOSE(j26,170*un,y,z,k,p,s);
	SETPOSE(j27,195*un,y,z,k,p,s);
	y=70*un;
	SETPOSE(j28,-5*un,y,z,k,p,s);
	SETPOSE(j29,20*un,y,z,k,p,s);
	SETPOSE(j30,45*un,y,z,k,p,s);
	SETPOSE(j31,70*un,y,z,k,p,s);
	SETPOSE(j32,95*un,y,z,k,p,s);
	SETPOSE(j33,120*un,y,z,k,p,s);
	SETPOSE(j34,145*un,y,z,k,p,s);
	SETPOSE(j35,170*un,y,z,k,p,s);
	SETPOSE(j36,195*un,y,z,k,p,s);
	y=95*un;
	SETPOSE(j37,-5*un,y,z,k,p,s);
	SETPOSE(j38,20*un,y,z,k,p,s);
	SETPOSE(j39,45*un,y,z,k,p,s);
	SETPOSE(j40,70*un,y,z,k,p,s);
	SETPOSE(j41,95*un,y,z,k,p,s);
	SETPOSE(j42,120*un,y,z,k,p,s);
	SETPOSE(j43,145*un,y,z,k,p,s);
	SETPOSE(j44,170*un,y,z,k,p,s);
	SETPOSE(j45,195*un,y,z,k,p,s);
	y=120*un;
	SETPOSE(j46,-5*un,y,z,k,p,s);
	SETPOSE(j47,20*un,y,z,k,p,s);
	SETPOSE(j48,45*un,y,z,k,p,s);
	SETPOSE(j49,70*un,y,z,k,p,s);
	SETPOSE(j50,95*un,y,z,k,p,s);
	SETPOSE(j51,120*un,y,z,k,p,s);
	SETPOSE(j52,145*un,y,z,k,p,s);
	SETPOSE(j53,170*un,y,z,k,p,s);
	SETPOSE(j54,195*un,y,z,k,p,s);
	y=145*un;
	SETPOSE(j55,-5*un,y,z,k,p,s);
	SETPOSE(j56,20*un,y,z,k,p,s);
	SETPOSE(j57,45*un,y,z,k,p,s);
	SETPOSE(j58,70*un,y,z,k,p,s);
	SETPOSE(j59,95*un,y,z,k,p,s);
	SETPOSE(j60,120*un,y,z,k,p,s);
	SETPOSE(j61,145*un,y,z,k,p,s);
	SETPOSE(j62,170*un,y,z,k,p,s);
	SETPOSE(j63,195*un,y,z,k,p,s);
	y=170*un;
	SETPOSE(j64,-5*un,y,z,k,p,s);
	SETPOSE(j65,20*un,y,z,k,p,s);
	SETPOSE(j66,45*un,y,z,k,p,s);
	SETPOSE(j67,70*un,y,z,k,p,s);
	SETPOSE(j68,95*un,y,z,k,p,s);
	SETPOSE(j69,120*un,y,z,k,p,s);
	SETPOSE(j70,145*un,y,z,k,p,s);
	SETPOSE(j71,170*un,y,z,k,p,s);
	SETPOSE(j72,195*un,y,z,k,p,s);
	y=195*un;
	SETPOSE(j73,-5*un,y,z,k,p,s);
	SETPOSE(j74,20*un,y,z,k,p,s);
	SETPOSE(j75,45*un,y,z,k,p,s);
	SETPOSE(j76,70*un,y,z,k,p,s);
	SETPOSE(j77,95*un,y,z,k,p,s);
	SETPOSE(j78,120*un,y,z,k,p,s);
	SETPOSE(j79,145*un,y,z,k,p,s);
	SETPOSE(j80,170*un,y,z,k,p,s);
	SETPOSE(j81,195*un,y,z,k,p,s);
	SETSPEED(sp,0.05,0.50);
	SETSPEED(sp1,0.05,0.3);//0.3
	SETSPEED(sp2,0.05,0.5);//0.5
	SETZONESIZE(zone,0.01);
	IMPORTTOOL(toolgg);
	IMPORTWOBJ(wobjgg);
    //初始化移动到工作位姿
	MoveJ(j0,sp,NULL,toolgg,wobjgg);printf("j0\n");
	MoveL(j1,sp,NULL,toolgg,wobjgg);printf("j1\n");
	//开始加工路径（由直线和圆弧组成且指令间用转弯区平滑过渡）
	MoveL(j8,sp,zone,toolgg,wobjgg);
	MoveC(j26,j18,sp,zone,toolgg,wobjgg);
	MoveL(j20,sp,zone,toolgg,wobjgg);
	MoveC(j38,j28,sp,zone,toolgg,wobjgg);
	MoveL(j44,sp,zone,toolgg,wobjgg);
	MoveC(j62,j54,sp,zone,toolgg,wobjgg);
	MoveL(j56,sp,zone,toolgg,wobjgg);
	MoveC(j74,j64,sp,zone,toolgg,wobjgg);
	MoveL(j80,sp,zone,toolgg,wobjgg);
	MoveC(j72,mid_ropose(j71,j81,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j18,sp,zone,toolgg,wobjgg);
	MoveC(j16,j8,sp,zone,toolgg,wobjgg);
	MoveL(j70,sp,zone,toolgg,wobjgg);
	MoveC(j68,j78,sp,zone,toolgg,wobjgg);
	MoveL(j14,sp,zone,toolgg,wobjgg);
	MoveC(j12,j4,sp,zone,toolgg,wobjgg);
	MoveL(j66,sp,zone,toolgg,wobjgg);
	MoveC(j64,j74,sp,zone,toolgg,wobjgg);
	MoveL(j10,sp,zone,toolgg,wobjgg);
	MoveC(j12,j2,sp,zone,toolgg,wobjgg);
	MoveC(j14,j22,sp,zone,toolgg,wobjgg);
	MoveC(j16,j6,sp,zone,toolgg,wobjgg);
	MoveC(j18,j26,sp,zone,toolgg,wobjgg);
	MoveC(j26,j8,sp,zone,toolgg,wobjgg);
	MoveC(j34,j44,sp,zone,toolgg,wobjgg);
	MoveC(j32,j24,sp,zone,toolgg,wobjgg);
	MoveC(j30,j40,sp,zone,toolgg,wobjgg);
	MoveC(j38,j20,sp,zone,toolgg,wobjgg);
	MoveC(j46,j56,sp,zone,toolgg,wobjgg);
	MoveC(j48,j38,sp,zone,toolgg,wobjgg);
	MoveC(j50,j58,sp,zone,toolgg,wobjgg);
	MoveC(j52,j42,sp,zone,toolgg,wobjgg);
	MoveC(j54,j62,sp,zone,toolgg,wobjgg);
	MoveC(j62,j44,sp,zone,toolgg,wobjgg);
	MoveC(j70,j80,sp,zone,toolgg,wobjgg);
	MoveC(j68,j60,sp,zone,toolgg,wobjgg);
	MoveC(j66,j76,sp,zone,toolgg,wobjgg);
	MoveC(j74,j56,sp,zone,toolgg,wobjgg);
	MoveC(j56,j66,sp,zone,toolgg,wobjgg);
	MoveC(j46,mid_ropose(j47,j55,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j10,sp,zone,toolgg,wobjgg);
	MoveC(j2,mid_ropose(j11,j1,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j4,sp,zone,toolgg,wobjgg);
	MoveC(j14,mid_ropose(j13,j5,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j32,sp,zone,toolgg,wobjgg);
	MoveC(j40,mid_ropose(j31,j41,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j38,sp,zone,toolgg,wobjgg);
	MoveC(j28,mid_ropose(j29,j37,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j10,sp,zone,toolgg,wobjgg);
	MoveC(j2,mid_ropose(j11,j1,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j6,sp,zone,toolgg,wobjgg);
	MoveC(j16,mid_ropose(j15,j7,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j52,sp,zone,toolgg,wobjgg);
	MoveC(j60,mid_ropose(j51,j61,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j56,sp,zone,toolgg,wobjgg);
	MoveC(j46,mid_ropose(j47,j55,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j10,sp,zone,toolgg,wobjgg);
	MoveC(j2,mid_ropose(j11,j1,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j8,sp,zone,toolgg,wobjgg);
	MoveC(j18,mid_ropose(j17,j9,&mid, o),sp,zone,toolgg,wobjgg);
	MoveL(j72,sp2,zone,toolgg,wobjgg);
	MoveC(j80,mid_ropose(j71,j81,&mid, o),sp2,zone,toolgg,wobjgg);
	MoveL(j74,sp1,zone,toolgg,wobjgg);
	MoveC(j64,mid_ropose(j65,j73,&mid, o),sp1,zone,toolgg,wobjgg);
	MoveL(j1,sp1,NULL,toolgg,wobjgg);//最后一条指令不可使用转弯区
	Rsleep(2000);
    return 0;
}