#include "user/robot_control.h"
namespace piano_server
{

bool RobotControl::StartControl(double feedback_rate)
{
    int err=0;
    HYYRobotBase::command_arg arg;
    std::string arg_c="--path "+config_path_+" "+"--iscopy true";
    err=HYYRobotBase::commandLineParser1(arg_c.c_str(), &arg);
    if (0!=err)
    {
        printf("config file path is error");
        return false;
    }
    err=HYYRobotBase::system_initialize(&arg);
    if (0!=err)
    {
        printf("StartContorl: system_initialize failure");
        return false;
    }

    SETTOOL_FRAME(tool_left,1,0,0,0,0,0,0);
    SETTOOL_FRAME(tool_right,1,0,0,0,0,0,0);
    SETWOBJ_FRAME(wobj_left,0,0,0,0,0,0,0);
    SUPWOBJ_AFRAME(wobj_left,0,0,0,0,0,0);
    SETWOBJ_FRAME(wobj_right,0,0,0,0,0,0,0);
    SUPWOBJ_AFRAME(wobj_right,0,0,0,0,0,0);



    ltool=*tool_left;
    lwobj=*wobj_left;
    rtool=*tool_right;
    rwobj=*wobj_right;
 
    message_server_.StartMessageServer(feedback_rate,tool_left,tool_right,wobj_left,wobj_right);

    HYYRobotBase::DevicePower();

    err=HYYRobotBase::StartAdditionServer(tool_left, wobj_left,LEFT_ARM_INDEX);
	if (0!=err)
	{
		printf("StartAdditionServer left arm failure");
        return false;
	}

    err=HYYRobotBase::StartAdditionServer(tool_right, wobj_right,RIGHT_ARM_INDEX);
	if (0!=err)
	{
		printf("StartAdditionServer right arm failure");
        return false;
	}

    HYYRobotBase::setMoveThread1(1,9e9);

    HYYRobotBase::initUserTimer(&timer_, 0, 1);
    time_=0;
    dt=HYYRobotBase::get_control_cycle(HYYRobotBase::get_deviceName(0,NULL))*1e9;//ns
    return true;
}

MessageServer* RobotControl::GetMessageServer()
{
    return &message_server_;
}

void RobotControl::CloseControl()
{

    HYYRobotBase::StopAdditionServer(LEFT_ARM_INDEX);

    HYYRobotBase::StopAdditionServer(RIGHT_ARM_INDEX);

    HYYRobotBase::DevicePoweroff();

}

void RobotControl::LoopControl()
{
    RESTART:
    std::vector<robot_server_data> data;
    bool is_get_data=true;

    while (((!message_server_.HasMessageData())||(_start!=message_server_.GetControlMode())||(!HYYRobotBase::robot_move_ok())))
    {
        usleep(100000);
    }
    message_server_.GetMessageData(data);
    is_get_data=false;
    sync_time(data[0].time_ns);

    printf("Start LoopControl\n");
    while(HYYRobotBase::robot_ok())
    {
        robot_timer();

        if (is_get_data)
        {
            if (!message_server_.GetMessageData(data))
            {
                continue;
            }
            is_get_data=false;
        }

        if (is_run(data))
        {
            is_get_data=true;
            move(data);
        }

        if (_stop==message_server_.GetControlMode())
        {
            message_server_.ClearMessageData();
            printf("Stop LoopControl\n");
            goto RESTART;
        }
        if (_pause==message_server_.GetControlMode())
        {
            printf("Pause LoopControl\n");
            while (HYYRobotBase::robot_ok())
            {
                if (_continue==message_server_.GetControlMode())
                {
                    printf("Continue LoopControl");
                    break;
                }
                if (_stop==message_server_.GetControlMode())
                {
                    printf("Stop LoopControl");
                    message_server_.ClearMessageData();
                    goto RESTART;
                }
                usleep(100000);
            }
        }

    }
    printf("Stop LoopControl\n");
}

uint64_t RobotControl::robot_timer()
{
    userTimer(&timer_);
    time_+=dt;
    return time_;
}

void RobotControl::sync_time(uint64_t time)
{
    time_=time;
}

bool RobotControl::move(std::vector<robot_server_data> &data)
{
    int ret=0;
    for(std::vector<robot_server_data>::iterator it = data.begin(); it != data.end();it++)
    {
        SETSPEEDTIME(speed,it->run_time);
        switch (it->type)
        {
        case _arm_left_l:
            printf("type:_arm_left_l; time_ns:%ld; run_time:%f; left_cartesian_num:%u\n",it->time_ns,it->run_time,it->left_cartesian_num);
            if (1==it->left_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->left_cartesian[0], speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->left_cartesian[0], it->left_cartesian_num, speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            break;
        case _arm_right_l:
            printf("type:_arm_right_l; time_ns:%ld; run_time:%f; right_cartesian_num:%u\n",it->time_ns,it->run_time,it->right_cartesian_num);
            if (1==it->right_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->right_cartesian[0], speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->right_cartesian[0], it->right_cartesian_num, speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            break;
        case _arm_both_l:
            printf("type:_arm_both_l; time_ns:%ld; run_time:%f; left_cartesian_num:%u; right_cartesian_num%u\n",it->time_ns,it->run_time,it->left_cartesian_num,it->right_cartesian_num);
            if (1==it->left_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->left_cartesian[0], speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->left_cartesian[0], it->left_cartesian_num, speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            if (0!=ret)
            {
                break;
            }
            if (1==it->right_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->right_cartesian[0], speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->right_cartesian[0], it->right_cartesian_num, speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            break;
        case _arm_left_j:
            {
                printf("type:_arm_left_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT7(ljoint,it->left_joint[0],it->left_joint[1],it->left_joint[2],it->left_joint[3],it->left_joint[4],it->left_joint[5],it->left_joint[6]);
                ret=MultiMoveA(ljoint, speed, NULL, NULL, NULL, LEFT_ARM_INDEX);
            }
            break;    
        case _arm_right_j:
            {
                printf("type:_arm_right_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT7(rjoint,it->right_joint[0],it->right_joint[1],it->right_joint[2],it->right_joint[3],it->right_joint[4],it->right_joint[5],it->right_joint[6]);
                ret=MultiMoveA(rjoint, speed, NULL, NULL, NULL, RIGHT_ARM_INDEX);
            }
            break;
        case _arm_both_j:
            {
                printf("type:_arm_both_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT7(ljoint,it->left_joint[0],it->left_joint[1],it->left_joint[2],it->left_joint[3],it->left_joint[4],it->left_joint[5],it->left_joint[6]);
                ret=MultiMoveA(ljoint, speed, NULL, NULL, NULL, LEFT_ARM_INDEX);
                SETJOINT7(rjoint,it->right_joint[0],it->right_joint[1],it->right_joint[2],it->right_joint[3],it->right_joint[4],it->right_joint[5],it->right_joint[6]);
                ret=MultiMoveA(rjoint, speed, NULL, NULL, NULL, RIGHT_ARM_INDEX);
            }
            break;
        case _foot_j:
            {
                printf("type:_foot_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT1(fjoint,it->foot_joint[0]);
                ret=MultiMoveAdd(fjoint, speed, NULL, NULL, NULL, FOOT_INDEX);
            }
            break;
        case _head_j:
            {
                printf("type:_head_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT2(hjoint,it->head_joint[0],it->head_joint[1]);
                ret=MultiMoveAdd(hjoint, speed, NULL, NULL, NULL, HEAD_INDEX);
            }
            break;
        case _waist_j:
            {
                printf("type:_waist_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT2(wjoint,it->waist_joint[0],it->waist_joint[1]);
                ret=MultiMoveAdd(wjoint, speed, NULL, NULL, NULL, WAIST_INDEX);
            }

            break;
        case _robot_l:
            printf("type:_robot_l; time_ns:%ld; run_time:%f;left_cartesian_num:%u; right_cartesian_num:%u,\n",it->time_ns,it->run_time,it->left_cartesian_num,it->right_cartesian_num);
            if (1==it->left_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->left_cartesian[0], speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->left_cartesian[0], it->left_cartesian_num, speed, NULL, &ltool, &lwobj, LEFT_ARM_INDEX);
            }
            if (0!=ret)
            {
                break;
            }
            if (1==it->right_cartesian_num)
            {
                ret=MultiMoveL((HYYRobotBase::robpose*)it->right_cartesian[0], speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            else
            {
                ret=MultiMoveS((HYYRobotBase::robpose*)it->right_cartesian[0], it->right_cartesian_num, speed, NULL, &rtool, &rwobj, RIGHT_ARM_INDEX);
            }
            if (0!=ret)
            {
                break;
            }
            {
                SETJOINT1(fjoint,it->foot_joint[0]);
                ret=MultiMoveAdd(fjoint, speed, NULL, NULL, NULL, FOOT_INDEX);
                if (0!=ret)
                {
                    break;
                }
                SETJOINT2(hjoint,it->head_joint[0],it->head_joint[1]);
                ret=MultiMoveAdd(hjoint, speed, NULL, NULL, NULL, HEAD_INDEX);
                if (0!=ret)
                {
                    break;
                }
                SETJOINT2(wjoint,it->waist_joint[0],it->waist_joint[1]);
                ret=MultiMoveAdd(wjoint, speed, NULL, NULL, NULL, WAIST_INDEX);
            }

            break;
        case _robot_j:
            {
                printf("type:_robot_j; time_ns:%ld; run_time:%f;\n",it->time_ns,it->run_time);
                SETJOINT7(ljoint,it->left_joint[0],it->left_joint[1],it->left_joint[2],it->left_joint[3],it->left_joint[4],it->left_joint[5],it->left_joint[6]);
                ret=MultiMoveA(ljoint, speed, NULL, NULL, NULL, LEFT_ARM_INDEX);
                SETJOINT7(rjoint,it->right_joint[0],it->right_joint[1],it->right_joint[2],it->right_joint[3],it->right_joint[4],it->right_joint[5],it->right_joint[6]);
                ret=MultiMoveA(rjoint, speed, NULL, NULL, NULL, RIGHT_ARM_INDEX);
                if (0!=ret)
                {
                    break;
                }
                SETJOINT1(fjoint,it->foot_joint[0]);
                ret=MultiMoveAdd(fjoint, speed, NULL, NULL, NULL, FOOT_INDEX);
                if (0!=ret)
                {
                    break;
                }
                SETJOINT2(hjoint,it->head_joint[0],it->head_joint[1]);
                ret=MultiMoveAdd(hjoint, speed, NULL, NULL, NULL, HEAD_INDEX);
                if (0!=ret)
                {
                    break;
                }
                SETJOINT2(wjoint,it->waist_joint[0],it->waist_joint[1]);
                ret=MultiMoveAdd(wjoint, speed, NULL, NULL, NULL, WAIST_INDEX);
            }
            break;
        default:
            break;
        }
        
        if (0!=ret)
        {
            printf("Move failure,ret=%d\n",ret);
            return false;
        }
    }
    printf("MoveSyncStart\n");
    HYYRobotBase::MoveSyncStart();
    return true;

}

bool RobotControl::is_run(std::vector<robot_server_data> &data)
{
    if (time_>=data[0].time_ns)
    {
        for(std::vector<robot_server_data>::iterator it = data.begin(); it != data.end();it++)
        {
            switch (it->type)
            {
            case _arm_left_l:
            case _arm_left_j:
                if (0!=HYYRobotBase::get_robot_move_state(LEFT_ARM_INDEX))
                {
                    return false;
                }
                break;
            case _arm_right_j:
            case _arm_right_l:
                if (0!=HYYRobotBase::get_robot_move_state(RIGHT_ARM_INDEX))
                {
                    return false;
                }
                break;
            case _arm_both_j:
            case _arm_both_l:
                if ((0!=HYYRobotBase::get_robot_move_state(LEFT_ARM_INDEX))||
                    (0!=HYYRobotBase::get_robot_move_state(RIGHT_ARM_INDEX)))
                {
                    return false;
                }
                break;
            case _foot_j:
                if (0!=HYYRobotBase::get_addition_move_state(FOOT_INDEX))
                {
                    return false;
                }
                break;
            case _head_j:
                if (0!=HYYRobotBase::get_addition_move_state(HEAD_INDEX))
                {
                    return false;
                }
                break;
            case _waist_j:
                if (0!=HYYRobotBase::get_addition_move_state(WAIST_INDEX))
                {
                    return false;
                }
                break;
            case _robot_j:
            case _robot_l:
                if ((0!=HYYRobotBase::get_robot_move_state(LEFT_ARM_INDEX))||
                    (0!=HYYRobotBase::get_robot_move_state(RIGHT_ARM_INDEX))||
                    (0!=HYYRobotBase::get_addition_move_state(FOOT_INDEX))||
                    (0!=HYYRobotBase::get_addition_move_state(HEAD_INDEX))||
                    (0!=HYYRobotBase::get_addition_move_state(WAIST_INDEX)))
                {
                    return false;
                }
                break;
            default:
                    return false;
                break;
            }
        }
        return true;

    }

    return false;
}


bool MessageServer::StartMessageServer( double feedback_rate,
HYYRobotBase::tool* tool_left,HYYRobotBase::tool* tool_right,
HYYRobotBase::wobj* wobj_left,HYYRobotBase::wobj* wobj_right)
{
    feedback_rate_=feedback_rate;
    tool_left_=*tool_left;
    tool_right_=*tool_right;
    wobj_left_=*wobj_left;
    wobj_right_=*wobj_right;
    feedbace_thread_=std::thread(&MessageServer::feedbace_server, this);
    control_mode_=_stop;
    return true;
}

void MessageServer::CloseMessageServer()
{
    if (feedbace_thread_.joinable())
    {
        feedbace_thread_.join();
    }
}

void MessageServer::feedbace_server()
{

    const char* left_arm=HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0,NULL), LEFT_ARM_INDEX);
    const char* right_arm=HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0,NULL), RIGHT_ARM_INDEX);
    const char* head=HYYRobotBase::get_name_additionaxis_device(HYYRobotBase::get_deviceName(0,NULL), HEAD_INDEX);
    const char* waist=HYYRobotBase::get_name_additionaxis_device(HYYRobotBase::get_deviceName(0,NULL), WAIST_INDEX);
    const char* foot=HYYRobotBase::get_name_additionaxis_device(HYYRobotBase::get_deviceName(0,NULL), FOOT_INDEX);

    double left_cartesian[6];
    double right_cartesian[6];

    double left_joint[7];
    double right_joint[7];
    double head_joint[2];
    double waist_joint[2];
    double foot_joint[1];

    double left_velocity[7];
    double right_velocity[7];
    double head_velocity[2];
    double waist_velocity[2];
    double foot_velocity[1];

    double left_current[7];
    double right_current[7];
    double head_current[2];
    double waist_current[2];
    double foot_current[1];

    double left_torque[7];
    double right_torque[7];
    double head_torque[2];
    double waist_torque[2];
    double foot_torque[1];
    HYYRobotBase::RTimer timer_;
    HYYRobotBase::initUserTimer(&timer_, 1, 1);
    while (HYYRobotBase::robot_ok())
    {
        userTimer(&timer_);
        HYYRobotBase::GetCurrentCartesian(&tool_left_,&wobj_left_, (HYYRobotBase::robpose*)left_cartesian, LEFT_ARM_INDEX);
        HYYRobotBase::GetCurrentCartesian(&tool_right_,&wobj_right_, (HYYRobotBase::robpose*)right_cartesian, RIGHT_ARM_INDEX);

        HYYRobotBase::GetGroupPosition(left_arm, left_joint);
        HYYRobotBase::GetGroupPosition(right_arm, right_joint);
        HYYRobotBase::GetGroupPosition(head, head_joint);
        HYYRobotBase::GetGroupPosition(waist, waist_joint);
        HYYRobotBase::GetGroupPosition(foot, foot_joint);

        HYYRobotBase::GetGroupVelocity(left_arm, left_velocity);
        HYYRobotBase::GetGroupVelocity(right_arm, right_velocity);
        HYYRobotBase::GetGroupVelocity(head, head_velocity);
        HYYRobotBase::GetGroupVelocity(waist, waist_velocity);
        HYYRobotBase::GetGroupVelocity(foot, foot_velocity);

        HYYRobotBase::GetGroupTorque(left_arm, left_torque);
        HYYRobotBase::GetGroupTorque(right_arm, right_torque);
        HYYRobotBase::GetGroupTorque(head, head_torque);
        HYYRobotBase::GetGroupTorque(waist, waist_torque);
        HYYRobotBase::GetGroupTorque(foot, foot_torque);

        HYYRobotBase::GetGroupTorque(left_arm, left_current);
        HYYRobotBase::GetGroupTorque(right_arm, right_current);
        HYYRobotBase::GetGroupTorque(head, head_current);
        HYYRobotBase::GetGroupTorque(waist, waist_current);
        HYYRobotBase::GetGroupTorque(foot, foot_current);
        //printf("%f,%f,%f,%f,%f\n",left_joint[0],right_joint[0],head_joint[0],waist_joint[0],foot_joint[0]);
    }


}

void MessageServer::SetMessageData(robot_server_data &data)
{
    data_lock_.lock();
    server_data_.push(data);
    data_lock_.unlock();
}

bool MessageServer::GetMessageData(std::vector<robot_server_data> &data)
{
    data_lock_.lock();
    if (server_data_.empty())
    {
        data_lock_.unlock();
        return false;
    }
    data.clear();
    robot_server_data _data=server_data_.front();
    data.push_back(_data);
    server_data_.pop();
    while (!server_data_.empty())
    {
        _data=server_data_.front();
        if (data[0].time_ns!=_data.time_ns)
        {
            break;
        }
        data.push_back(_data);
        server_data_.pop();
    }
    data_lock_.unlock();
    return true;
}

bool MessageServer::ClearMessageData()
{
    data_lock_.lock();
    while (!server_data_.empty()) 
    {
        server_data_.pop();
    }
    data_lock_.unlock();
    return true;
}

bool MessageServer::HasMessageData()
{
    //data_lock_.lock();
    bool tmp=!server_data_.empty();
    //data_lock_.unlock();
    return tmp;
}

uint8_t MessageServer::GetControlMode()
{
    data_lock_.lock();
    uint8_t tmp=control_mode_;
    data_lock_.unlock();
    return tmp;
}

void MessageServer::SetControlMode(uint8_t mode)
{
    data_lock_.lock();
    control_mode_=mode;
    data_lock_.unlock();
}

}