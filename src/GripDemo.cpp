#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int GripDemo()
{
	int ret=0;
    //创建夹爪(名称在配置文件中已指定)
	ret=CreateGrip("grip");
    printf("CreateGrip,ret=%d\n",ret);
	Rsleep(1000);//1s
	ret=ControlGrip("grip", 0);//打开夹爪
    printf("ControlGrip,ret=%d\n",ret);
	Rsleep(2000);//2s
	ret=ControlGrip("grip", 1);//关闭夹爪
    printf("ControlGrip,ret=%d\n",ret);
	Rsleep(5000);
    return 0;
}