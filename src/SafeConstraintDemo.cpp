#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SafeConstraintDemo()
{
	IMPORTSPEED(v100);
	IMPORTZONE(z0);
	IMPORTTOOL(tool0);
	IMPORTWOBJ(wobj0);
	int ret=0;
    //设置球体区域，性质：不允许区域，但允许缓慢移动
	double centre[3]={0.438,0,0.365};
	ret=AddSpatialConstraintSphere(centre, 0.1, _constraint_slowallow, 
                                   tool0, wobj0, "sphere1");
    SetSpatialConstraintSlowCoeff(0.2, "sphere1");//设置降速百分比
    //设置立方体，性质：不允许区域，但允许缓慢移动
 	double centre1[3]={0.400, 0.100,0.400};
 	double length[3]={0.400,0.100,0.300};
 	double width[3]={0.400,-0.100,0.400};
 	double high[3]={0.700,0.100,0.400};
 	ret=AddSpatialConstraintCuboid(centre1, length, width, high, 
                                   _constraint_slowallow, tool0, wobj0, "cuboid1");
    SetSpatialConstraintSlowCoeff(0.2, "cuboid1");//设置降速百分比
    //设置圆柱体性质：不允许区域
 	double centreb[3]={0.438,0,0};
 	double centreh[3]={0.438,0,0.400};
 	ret=AddSpatialConstraintCylinder(centreb, centreh, 0, 0.1, 
                                     _constraint_noallow, NULL, NULL, "cylinder");
 	printf("AddSpatialConstraintCylinder:%d\n",ret);
    //移动机器人,进入_constraint_slowallow区，机器人会降速; 进入_constraint_noallow区机器人会停止移动，并置为错误状态
	robpose rpose;
	init_robpose(&rpose, centre, NULL);
	ret=MoveL(&rpose,v100,NULL,tool0,wobj0);
	printf("MoveL:%d\n",ret);
	ret=DeleteAllSpatialConstraint();//删除所有区域
    printf("ret=%d\n",ret);
    return 0;
}