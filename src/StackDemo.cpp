#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int StackDemo()
{
	IMPORTSTACK(stack1,1);//导入码垛工艺
	StackTechnologySetCurrentWorkpieceNum(stack1, 1);//设置当前码垛工件id
	IMPORTPOSE(enterp);//导入入口点位姿
	IMPORTPOSE(auxiliaryp);//导入辅助点位姿
	IMPORTPOSE(workp);//导入放置位姿
	IMPORTPOSE(getworkp);//导入拾取工件位姿
	IMPORTSPEED(v500);//导入速度数据
	IMPORTZONE(z0);//导入转弯区数据
	IMPORTTOOL(tool0);//导入工具
	IMPORTWOBJ(wobj0);//导入工件
	IMPORTJOINT(jinit);//导入初始化点
	//开始码垛
	while(StackTechnologyState(stack1)&&robot_ok())
	{
		//移动到初始化点
		MoveA(jinit,v500,z0,tool0,wobj0);
		//移动到拾取点上方
		robpose getworkp_up=Offs(getworkp,0,0,0.100,0,0,0);		
		MoveL(&getworkp_up,v500, z0, tool0, wobj0);	
		//移动到拾取点
		MoveL(getworkp,v500, z0, tool0, wobj0);
		//抓取工件(io控制型工具)
		SetDo(1,1);	
		//移动到拾取点上方		
		MoveL(&getworkp_up,v500, z0, tool0, wobj0);
		//移动到入口点
		StackTechnologyMoveEnterPosition(stack1, enterp, v500, z0, tool0, wobj0 );
		//移动到辅助点
		StackTechMoveAuxiliaryPosition(stack1, auxiliaryp, v500, z0, tool0, wobj0 );
		//移动到放置点
		StackTechMoveWorkpiecePosition(stack1, workp, v500, NULL, tool0, wobj0 );
		//放置工件(io控制型工具)
		SetDo(1,0);
		//移动到辅助点
		StackTechMoveAuxiliaryPosition(stack1, auxiliaryp, v500, z0, tool0, wobj0 );
		//移动到入口点
		StackTechnologyMoveEnterPosition(stack1, enterp, v500, NULL, tool0, wobj0 );
	} 
    return 0;
}