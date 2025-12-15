/*
 * main.cpp
 *
 *  Created on: 2022-9-20
 *      Author: HanBing
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "HYYRobotInterface.h"
#include "user/BscanServer.h"
extern int ServoDemo();
int main(int argc, char *argv[])
{	
	//------------------------initialize----------------------------------
	int err=0;
	HYYRobotBase::command_arg arg;
	err=HYYRobotBase::commandLineParser(argc, argv,&arg);
	if (0!=err)
	{
		return -1;
	}
	err=HYYRobotBase::system_initialize(&arg);
	if (0!=err)
	{
		return err;
	}
	//-----------------------user designation codes---------------
	ServoDemo();
	// HYYRobotBase::DevicePower();
	// bscan_server::BscanServer bs;
	// IMPORTTOOL(tool10);
	// bs.StartBscanServer(100,tool10,NULL);

	// HYYRobotBase::DevicePoweroff();


	//------------------------wait----------------------------------
	pause();
	return 0;
}
