#include "HYYRobotInterface.h"

using namespace HYYRobotBase;//仅c++需要

    int AxisPower(const char* name,int axis_ID)
    {
        if ((axis_ID<=0)||(axis_ID>get_group_dof(name)))
        {
            return -1;
        }

        int _position=get_axis_position(name,axis_ID);
        set_axis_position(name,_position,axis_ID);//positon set before power

        unsigned short stat;
        int count=0;
        int time=1000;//1ms
        int time_over=500000;//us=0.5s,单位us,允许的最大时间
        while(1)
        {
           usleep(time);

           if (count>=(time_over/time))
           {
               set_axis_control(name,0,axis_ID);
              return -2;
           }
           count++;
           stat=get_axis_status(name,axis_ID);
			
           if ((stat&0x4F)==0x00)
           {
               continue;
           }else if ((stat&0x4F)==0x40)
           {
               set_axis_control(name,6,axis_ID);
           }else if ((stat&0x6F)==0x21)
           {
               set_axis_control(name,7,axis_ID);
           }else if ((stat&0x6F)==0x23)
           {
               set_axis_control(name,15,axis_ID);
           }else if ((stat&0x6F)==0x27)
           {
               break;
           }else if ((stat&0x6F)==0x07)
           {
               set_axis_control(name,15,axis_ID);
           }else if ((stat&0x4F)==0x0F)
           {
               set_axis_control(name,0,axis_ID);
               return -3;

           }else if ((stat&0x4F)==0x08)
           {
               set_axis_control(name,0,axis_ID);
               return -4;
           }
           else
           {
               set_axis_control(name,0,axis_ID);
           }

        }
        return 0;
    }

    int AxisPoweroff(const char* name,int axis_ID)
    {

        if ((axis_ID<=0)||(axis_ID>get_group_dof(name)))
        {
            return -1;
        }
        set_axis_control(name,0,axis_ID);
        return 0;
    }