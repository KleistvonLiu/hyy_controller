#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int EGMDemo()
{
    IMPORTJOINT(jegm);
    IMPORTSPEED(v10);
    IMPORTTOOL(tool15);
    IMPORTWOBJ(wobj0);
	int ret=0;
	//设置底面,允许进入该区，但会有反作用力阻止继续深入
 	double centre1[3]={-10000, -10000,0};
 	double length[3]={10000,-10000,0};
 	double width[3]={-10000,10000,0};
 	double high[3]={-10000, -10000,0.080};
 	ret=ret|AddSpatialConstraintCuboid(centre1, length, width, high, 
                                      _constraint_slowallow, NULL, NULL, "cuboid1");
	//设置底面,不允许进入该区，但会停止移动并报警
	double high1[3]={-10000, -10000,0.050};
 	ret=ret|AddSpatialConstraintCuboid(centre1, length, width, high1, 
                                      _constraint_noallow, NULL, NULL, "cuboid2");
	OffSpatialConstraint();//关闭笛卡尔空间约束检测，允许强行恢复运动
	RESTART:
	MoveA(jegm,v10,NULL,NULL,NULL);
	OnSpatialConstraint();//开启笛卡尔空间约束检测
    //创建引导数据资源
	EGMCreate_c("egm", 0, 0, 1, 4, tool15, wobj0);	
	EGMRunJoint_c("egm", 1);//开启引导
	EGMRelease_c("egm");//释放引导数据资源
    //判断是否由错误引起的退出引导服务
	if (!robot_move_ok())
	{
		clear_robot_move_error(0);//强行清楚错误，允许恢复运动
		OffSpatialConstraint();//关闭笛卡尔空间约束检测，允许恢复运动
		Rsleep(1000);
		goto RESTART;
	}
    //删除所有保护区域
	DeleteAllSpatialConstraint();
    return 0;
}