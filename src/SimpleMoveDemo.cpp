#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int SimpleMoveDemo()
{
    //机器人回零位
    SETJOINT6(home,0,0,0,0,0,0);
    IMPORTSPEED(v100);
    MoveA(home,v100,NULL3);
    //运动到指定关节位置
    IMPORTJOINT(j2);
    MoveA(j2,v100,NULL3);
    //在当前笛卡尔位置上沿工件坐标系y轴正向移动0.01m(绝对移动)
    IMPORTTOOL(tool0);
    IMPORTWOBJ(wobj0);
    robpose tp;
    GetCurrentCartesian (tool0, wobj0, &tp, 0);
    tp=Offs(&tp, 0, 0.01, 0, 0, 0, 0);
    MoveL(&tp,v100,NULL,tool0,wobj0);
    //在当前笛卡尔位置上沿工具坐标系y轴正向移动0.01m(相对移动)
    GetCurrentCartesian (tool0, wobj0, &tp, 0);
    tp=OffsRel(&tp, 0, 0.01, 0, 0, 0, 0);
    MoveL(&tp,v100,NULL,tool0,wobj0);
    return 0;
}