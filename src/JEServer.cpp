#include "HYYRobotInterface.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp"
#include <vector>
#include <atomic>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 插件入库函数,用于实现插件初始化(被控制系初始函数调用),该函数要求非阻塞，如阻塞需要开线程运行
 */
extern void PluginMain();
#ifdef __cplusplus
}
#endif

#define CYCLIE 10

static zmq::context_t context(1);
static zmq::socket_t publisher(context, zmq::socket_type::pub);
static zmq::socket_t subscriber(context, zmq::socket_type::sub);
std::thread* pub_th=nullptr;
std::thread* sub_th=nullptr;
static void publisher_loop()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer,0,CYCLIE);//10ms
    nlohmann::ordered_json data;
    std::cout << "JEServer: start publisher loop" << std::endl;
    while (true)
    {
        HYYRobotBase::userTimer(&timer);
        int rn=HYYRobotBase::robot_getNUM();
        for (int i=0;i<rn;i++)
        {
            data[std::string("Robot")+std::to_string(i)]["MoveState"]=HYYRobotBase::get_robot_move_state(i);
            data[std::string("Robot")+std::to_string(i)]["PowerState"]=HYYRobotBase::GetRobotPowerState(i);
            int dof=HYYRobotBase::robot_getDOF(i);
            std::vector<double> joint(dof);
            HYYRobotBase::GetCurrentJoint(joint.data(), 0);
            data[std::string("Robot")+std::to_string(i)]["Joint"]=joint;
            std::vector<double> Cartesian(6);
            HYYRobotBase::GetCurrentCartesian(NULL,NULL,(HYYRobotBase::robpose*)Cartesian.data(), 0);
            data[std::string("Robot")+std::to_string(i)]["Cartesian"]=Cartesian;
        }
        publisher.send(zmq::buffer("State " + data.dump()));
    }
}

static void subscriber_loop()
{
    std::cout << "JEServer: start subscriber loop" << std::endl;
    while(true)
    {
        zmq::message_t msg;
        subscriber.recv(msg);
        std::string cmd(static_cast<char*>(msg.data()), msg.size());
        auto pos = cmd.find(' ');
        std::string topic = cmd.substr(0, pos);
        nlohmann::ordered_json cmd_json = nlohmann::json::parse(cmd.substr(pos + 1));
        // std::cout<<cmd_json.dump(4)<<std::endl;
        // std::cout<<topic<<std::endl;
        int rn=HYYRobotBase::robot_getNUM();
        if ("Switch"==topic)
        {
            if (cmd_json.contains("Switch"))
            {
                HYYRobotBase::ClearRobotError();
                if (cmd_json["Switch"].get<bool>())
                {

                    for (int i=0;i<rn;i++)
                    {
                        // HYYRobotBase::RobotStopRecover(i);
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                        HYYRobotBase::RobotPower(i);
                        HYYRobotBase::ServoStart(0.00001,0.5, i);
                    }
                }
                else
                {
                    for (int i=0;i<rn;i++)
                    {
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                    } 
                }
            } 
	}else if("Joint"==topic)
	{
	    // std::cout << "JEServer: control with joint" << std::endl;
	    for (int i = 0; i < rn; i++)
	    {
	        if (cmd_json.contains(std::string("Robot") + std::to_string(i)))
	        {
	            // std::cout << "JEServer: control with joint here, Robot" << i << std::endl;

	            // 原有逻辑
	            double time = cmd_json[std::string("Robot") + std::to_string(i)]["time"].get<double>();
	            std::vector<double> joint =
	                cmd_json[std::string("Robot") + std::to_string(i)]["joint"].get<std::vector<double>>();

	            // ======= 新增调试输出：打印 data（joint 向量的内容） =======
	            // std::cout << "[DEBUG] Robot" << i << " time = " << time << std::endl;
	            // std::cout << "[DEBUG] Robot" << i << " joint.size() = " << joint.size() << std::endl;
	            // std::cout << "[DEBUG] Robot" << i << " joint data = [";
	            // for (size_t j = 0; j < joint.size(); ++j)
	            // {
	                // std::cout << joint[j];
	                // if (j + 1 < joint.size())
	                    // std::cout << ", ";
	            // }
	            // std::cout << "]" << std::endl;

	            // 如果你想看指针地址（意义不大，一般不用）
	            // std::cout << "[DEBUG] joint.data() ptr = " << static_cast<const void*>(joint.data()) << std::endl;
	            // ================================================

	            HYYRobotBase::robjoint jt;
	            HYYRobotBase::init_robjoint(&jt, joint.data(), HYYRobotBase::robot_getDOF(i));
	            HYYRobotBase::ServoJoint(&jt, time, i);
	        }
	    }
        }else if ("Cartesian"==topic)
        {
            for (int i=0;i<rn;i++)
            {
                if (cmd_json.contains(std::string("Robot")+std::to_string(i)))
                {
                    double time=cmd_json[std::string("Robot")+std::to_string(i)]["time"].get<double>();
                    std::vector<double> cartesian=cmd_json[std::string("Robot")+std::to_string(i)]["cartesian"].get<std::vector<double>>();
                    HYYRobotBase::robpose pt;
                    HYYRobotBase::init_robpose(&pt,cartesian.data(),cartesian.data()+3);
                    HYYRobotBase::ServoCartesian(&pt,time,NULL,NULL,i);

                    // SETSPEED(sp,0.05,0.50);
                    // HYYRobotBase::MoveJ(&pt,sp,NULL,NULL,NULL);
                }
            }
        }
    }
}

void PluginMain()
{
    publisher.set(zmq::sockopt::sndhwm, 0);  // 0 表示无限小队列，但行为是：不能缓存
    publisher.set(zmq::sockopt::immediate, 1);  // SUB 未连接时直接丢弃
    publisher.bind("tcp://*:8000");
    pub_th=new std::thread(publisher_loop);
    pub_th->detach();
    subscriber.connect("tcp://192.168.0.35:8001");
    subscriber.set(zmq::sockopt::subscribe, "");
    sub_th=new std::thread(subscriber_loop);
    sub_th->detach();
}
