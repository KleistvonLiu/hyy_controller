#include "HYYRobotInterface.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <fstream>   // ① 增加这一行
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
static std::thread* pub_th=nullptr;
static std::thread* sub_th=nullptr;
static std::atomic<bool> is_stop(true);
static std::thread* stop_th=nullptr;

static void save_data()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer,1,1);
    double data[14];
    while (!is_stop.load())
    {
        HYYRobotBase::userTimer(&timer);
        memset(data,0,sizeof(data));
        HYYRobotBase::GetCurrentJoint(data, 0);
        HYYRobotBase::GetCurrentLastTargetJoint(&(data[7]), 0);
        HYYRobotBase::RSaveDataFast1("jeserver",1, 100, 14, data );
    }
}

static void publisher_loop()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer,0,CYCLIE);//10ms
    nlohmann::ordered_json data;
    printf("start publisher_loop\n");
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
            HYYRobotBase::GetCurrentLastTargetJoint(joint.data(), 0);
            data[std::string("Robot")+std::to_string(i)]["TargetJoint"]=joint;
            std::vector<double> Cartesian(6);
            HYYRobotBase::GetCurrentCartesian(NULL,NULL,(HYYRobotBase::robpose*)Cartesian.data(), 0);
            data[std::string("Robot")+std::to_string(i)]["Cartesian"]=Cartesian;
            HYYRobotBase::GetCurrentLastTargetCartesian(NULL,NULL,(HYYRobotBase::robpose*)Cartesian.data(), 0);
            data[std::string("Robot")+std::to_string(i)]["TargetCartesian"]=Cartesian;
        }
        publisher.send(zmq::buffer("State " + data.dump()));
    }
}

static void subscriber_loop()
{
    // 2 增加落盘
    // static std::ofstream cart_log("cartesian_log.csv", std::ios::out | std::ios::app);
    printf("start subscriber_loop\n");
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
                        is_stop.store(true);
                        HYYRobotBase::RobotStopRecover(i);
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                        usleep(100000);
                        HYYRobotBase::RobotPower(i);
                        HYYRobotBase::ServoStart(1,0.0001, i);
                        is_stop.store(false);
                        stop_th=new std::thread(save_data);
                        stop_th->detach();
                    }
                }
                else
                {
                    for (int i=0;i<rn;i++)
                    {
                        is_stop.store(true);
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                    } 
                }
            } 
        }else if("Joint"==topic)
        {
            for (int i=0;i<rn;i++)
            {
                if (cmd_json.contains(std::string("Robot")+std::to_string(i)))
                {
                    double time=cmd_json[std::string("Robot")+std::to_string(i)]["time"].get<double>();
                    //printf("===%f\n",time);
                    std::vector<double> joint=cmd_json[std::string("Robot")+std::to_string(i)]["joint"].get<std::vector<double>>();
                    HYYRobotBase::robjoint jt;
                    HYYRobotBase::init_robjoint(&jt,joint.data(),HYYRobotBase::robot_getDOF(i));
                    HYYRobotBase::ServoJoint(&jt,time,i);
                }
            }
        }
        else if ("Cartesian"==topic)
        {
            for (int i=0;i<rn;i++)
            {
                const std::string rk = std::string("Robot")+std::to_string(i);
                if (cmd_json.contains(rk))
                {
                    double time=cmd_json[rk]["time"].get<double>();
                    std::vector<double> cartesian=cmd_json[rk]["cartesian"].get<std::vector<double>>();

                    // // ③ 增加：落盘（CSV：robot_id,time,c0,c1,...）
                    // cart_log << i << "," << time;
                    // for (double v : cartesian) cart_log << "," << v;
                    // cart_log << "\n";
                    // cart_log.flush();  // 最简单：每条都刷盘

                    HYYRobotBase::robpose pt;
                    HYYRobotBase::init_robpose(&pt,cartesian.data(),cartesian.data()+3);
                    HYYRobotBase::ServoCartesian(&pt,time,NULL,NULL,i);
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
