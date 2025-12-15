
#include "user/robot_control.h"

int piano_main()
{

    //std::string path="/home/hanbing/work/program/robot/config/robot/zhijiang/robot_config";
    std::string path="/home/robot/Work/system/robot_config";
    piano_server::RobotControl robot(path);

    if (!robot.StartControl())
    {
        printf("StartControl failure");
        return 0;
    }

    piano_server::MessageServer* server=robot.GetMessageServer();

    piano_server::robot_server_data data;
    double T=1.5;
    double tns=1e+9;
    //l 1,r 1
    printf("l 1,r 1\n");
    data.type=piano_server::_arm_left_j;
    data.time_ns=0;
    data.run_time=5;
    data.left_joint[0]=-0.574859;data.left_joint[1]=-0.854140;data.left_joint[2]=1.071294;
    data.left_joint[3]=-0.384550;data.left_joint[4]=-1.779418;data.left_joint[5]=0.350323;data.left_joint[6]=-0.471987;
    server->SetMessageData(data);
    data.type=piano_server::_arm_right_j;
    data.time_ns=0;
    data.run_time=5;
    data.right_joint[0]=0.078521;data.right_joint[1]=-0.930263;data.right_joint[2]=-0.717807;
    data.right_joint[3]=-0.578982;data.right_joint[4]=1.981040;data.right_joint[5]=0.471028;data.right_joint[6]=0.195487;
    server->SetMessageData(data);

    //l 2,r 2
    printf("l 2,r 2\n");
    data.type=piano_server::_arm_left_l;
    data.time_ns=6*tns;
    data.run_time=T;
    data.left_cartesian[0][0]=0.626603;data.left_cartesian[0][1]=0.234808;data.left_cartesian[0][2]=0.112869;
    data.left_cartesian[0][3]=0.016240;data.left_cartesian[0][4]=-0.182864;data.left_cartesian[0][5]=0.116378;
    data.left_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_arm_right_l;
    data.time_ns=8*tns;
    data.run_time=T;
    data.right_cartesian[0][0]=0.659967;data.right_cartesian[0][1]=-0.137137;data.right_cartesian[0][2]=0.117075;
    data.right_cartesian[0][3]=-0.011483;data.right_cartesian[0][4]=0.015480;data.right_cartesian[0][5]=0.004637;
    data.right_cartesian_num=1;
    server->SetMessageData(data);
    usleep(100000);

    //l 1, r 1
    printf("l 1, r 1\n");
    data.type=piano_server::_arm_left_l;
    data.time_ns=14*tns;
    data.run_time=T;
    data.left_cartesian[0][0]=0.626565;data.left_cartesian[0][1]=0.604541;data.left_cartesian[0][2]=0.112843;
    data.left_cartesian[0][3]=0.016323;data.left_cartesian[0][4]=-0.182799;data.left_cartesian[0][5]=0.116398;
    data.left_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_arm_right_l;
    data.time_ns=14*tns;
    data.run_time=T;
    data.right_cartesian[0][0]=0.629918;data.right_cartesian[0][1]=-0.605226;data.right_cartesian[0][2]=0.116984;
    data.right_cartesian[0][3]=-0.011640;data.right_cartesian[0][4]=0.015714;data.right_cartesian[0][5]=0.004719;
    data.right_cartesian_num=1;
    server->SetMessageData(data);
    usleep(100000);

    // h
    printf("h\n");
    data.type=piano_server::_head_j;
    data.time_ns=18*tns;
    data.run_time=T;
    data.head_joint[0]=0;data.head_joint[1]=0;
    server->SetMessageData(data);
    usleep(100000);

    //l 2, r 2, h
    printf("l 2, r 2, h\n");
    data.type=piano_server::_arm_left_l;
    data.time_ns=22*tns;
    data.run_time=T;
    data.left_cartesian[0][0]=0.626603;data.left_cartesian[0][1]=0.234808;data.left_cartesian[0][2]=0.112869;
    data.left_cartesian[0][3]=0.016240;data.left_cartesian[0][4]=-0.182864;data.left_cartesian[0][5]=0.116378;
    data.left_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_arm_right_l;
    data.time_ns=22*tns;
    data.run_time=T;
    data.right_cartesian[0][0]=0.659967;data.right_cartesian[0][1]=-0.137137;data.right_cartesian[0][2]=0.117075;
    data.right_cartesian[0][3]=-0.011483;data.right_cartesian[0][4]=0.015480;data.right_cartesian[0][5]=0.004637;
    data.right_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_head_j;
    data.time_ns=22*tns;
    data.run_time=T;
    data.head_joint[0]=0;data.head_joint[1]=0;
    server->SetMessageData(data);
    usleep(100000);

    //l 1, r 1, w
    printf("l 1, r 1, w\n");
    data.type=piano_server::_arm_left_l;
    data.time_ns=28*tns;
    data.run_time=T;
    data.left_cartesian[0][0]=0.626565;data.left_cartesian[0][1]=0.604541;data.left_cartesian[0][2]=0.112843;
    data.left_cartesian[0][3]=0.016323;data.left_cartesian[0][4]=-0.182799;data.left_cartesian[0][5]=0.116398;
    data.left_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_arm_right_l;
    data.time_ns=28*tns;
    data.run_time=T;
    data.right_cartesian[0][0]=0.629918;data.right_cartesian[0][1]=-0.605226;data.right_cartesian[0][2]=0.116984;
    data.right_cartesian[0][3]=-0.011640;data.right_cartesian[0][4]=0.015714;data.right_cartesian[0][5]=0.004719;
    data.right_cartesian_num=1;
    server->SetMessageData(data);
    data.type=piano_server::_waist_j;
    data.time_ns=28*tns;
    data.run_time=T;
    data.waist_joint[0]=-0.1;data.waist_joint[1]=-0.05;
    server->SetMessageData(data);
    usleep(100000);

    // //l 2,r 2, w, h, f
    // printf("l 2,r 2, w, h, f\n");
    // data.type=piano_server::_arm_left_l;
    // data.time_ns=33*tns;
    // data.run_time=T;
    // data.left_cartesian[0][0]=0.626603;data.left_cartesian[0][1]=0.234808;data.left_cartesian[0][2]=0.112869;
    // data.left_cartesian[0][3]=0.016240;data.left_cartesian[0][4]=-0.182864;data.left_cartesian[0][5]=0.116378;
    // data.left_cartesian_num=1;
    // server->SetMessageData(data);
    // data.type=piano_server::_arm_right_l;
    // data.time_ns=33*tns;
    // data.run_time=T;
    // data.right_cartesian[0][0]=0.659967;data.right_cartesian[0][1]=-0.137137;data.right_cartesian[0][2]=0.117075;
    // data.right_cartesian[0][3]=-0.011483;data.right_cartesian[0][4]=0.015480;data.right_cartesian[0][5]=0.004637;
    // data.right_cartesian_num=1;
    // server->SetMessageData(data);
    // data.type=piano_server::_waist_j;
    // data.time_ns=33*tns;
    // data.run_time=T;
    // data.waist_joint[0]=0;data.waist_joint[1]=0;
    // server->SetMessageData(data);
    // data.type=piano_server::_head_j;
    // data.time_ns=33*tns;
    // data.run_time=T;
    // data.head_joint[0]=0.1;data.head_joint[1]=0.2;
    // server->SetMessageData(data);
    // data.type=piano_server::_foot_j;
    // data.time_ns=33*tns;
    // data.run_time=T;
    // data.foot_joint[0]=0.1;
    // server->SetMessageData(data);

    usleep(100000);
    printf("mode\n");
    server->SetControlMode(1);



    robot.LoopControl();
    robot.CloseControl();

    return 0;
}
