#include "HYYRobotInterface.h"
using namespace HYYRobotBase;//仅c++需要
int DynModelDemo()
{
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);
    int dof=get_group_dof(robot_name);//获取机器人关节数目
    //设置动力学输入
    R_DYNAMICS rdyn;
    init_R_DYNAMICS2(&rdyn, NULL, NULL, NULL, dof);
    IMPORTPAYLOAD(payload0);//导入负载
    set_R_DYNAMICS_payload(&rdyn, payload0);//设置工具负载
    double q[10]={0.1,0.1,0.1,0.1,0.1,0.1,0,0,0,0};
    double qd[10]={0.1,0.1,0.1,0.1,0.1,0.1,0,0,0,0};
    double qdd[10]={0.1,0.1,0.1,0.1,0.1,0.1,0,0,0,0};
    set_joint_state_to_R_DYNAMICS(&rdyn, q, qd, qdd);
    Dyn_Inverse(robot_name, &rdyn);//逆动力学求解
    double torque[10];
    get_torque_from_R_DYNAMICS(&rdyn, torque);//对应rdyn.torque
    printf("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f\n",
    torque[0],torque[1],torque[2],torque[3],torque[4],torque[5]);
    //重力项求解
    R_DYNAMICS rdyn_grav;
    init_R_DYNAMICS2(&rdyn_grav, NULL, NULL, NULL, dof);
    set_R_DYNAMICS_GRAVITY(&rdyn_grav, q, dof);
    Dyn_Gravity(robot_name, &rdyn_grav);//逆动力学重力项求解
    printf("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f\n"
    ,rdyn_grav.gravity.D[0],rdyn_grav.gravity.D[1],rdyn_grav.gravity.D[2],
    rdyn_grav.gravity.D[3],rdyn_grav.gravity.D[4],rdyn_grav.gravity.D[5]);
    return 0;
}