#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int DataStructDemo()
{
    //从控制系统中导入已有数据到数据变量(数据点必须在系统中存在)
    IMPORTPOSE(p2);
	IMPORTPOSE(p3);
	IMPORTJOINT(j1);
	IMPORTSPEED(v100);
	IMPORTZONE(z0);
	IMPORTTOOL(tool0);
	IMPORTWOBJ(wobj0);
    printf("p2:x:%f,y:%f,z:%f,k:%f,p:%f,s:%f\n",
    p2->xyz[0],p2->xyz[1],p2->xyz[2],p2->kps[0],p2->kps[1],p2->kps[2]);
    double p2xyz[3];
    double p2rpy[3];
    parse_robpose(p2, p2xyz, p2rpy);
    printf("p2:x:%f,y:%f,z:%f,k:%f,p:%f,s:%f\n",
    p2xyz[0],p2xyz[1],p2xyz[2],p2rpy[0],p2rpy[1],p2rpy[2]);
    //设置数据变量
    SETJOINT6(rb,0.1,0.5,-1,0.8,-0.3,-0.8);
	SETSPEEDTIME(rbv,5);
    printf("rbv(tcp):%f,%f,%d\n",rbv->tcp,rbv->orl,rbv->tcp_flag);
    printf("rbv(joint):%f,%f,%f,%f,%f,%f,%d\n",
    rbv->per[0],rbv->per[1],rbv->per[2],rbv->per[3],rbv->per[4],rbv->per[5],rbv->per_flag);
    //从控制系统中导入已有数据到数据变量(类似IMPORT***类接口功能)
    robjoint j1_;
    getrobjoint ("j1", &j1_);
    printf("j1:%f,%f,%f,%f,%f,%f,%d\n",
    j1_.angle[0],j1_.angle[1],j1_.angle[2],j1_.angle[3],j1_.angle[4],j1_.angle[5],j1_.dof);
    //写变量数据到系统
    robpose p5;
    double xyz[3]={0,0,0};
    double rpy[3]={0,0,0};
    init_robpose (&p5, xyz, rpy);
    writerobpose("p5", &p5);
    return 0;
}