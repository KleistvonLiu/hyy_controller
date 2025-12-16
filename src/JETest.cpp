#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp"
#include <vector>
#include <fstream>
#include <iomanip>
#include <cmath>

static inline double now_sec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void RobotSwitch(zmq::socket_t &pub,bool value)
{
    nlohmann::ordered_json cmd_switch;
    cmd_switch["Switch"] = value;
    pub.send(zmq::buffer("Switch " + cmd_switch.dump()));
}

nlohmann::ordered_json GetRobotState(zmq::socket_t &sub)
{
    nlohmann::ordered_json state_json;
    zmq::message_t msg;
    sub.recv(msg);
    std::string state(static_cast<char*>(msg.data()), msg.size());
    auto pos = state.find(' ');
    std::string topic = state.substr(0, pos);
    if ("State" == topic)
    {
        state_json = nlohmann::json::parse(state.substr(pos + 1));
    }
    return state_json;
}

void ClearHistoricalData(zmq::socket_t &sub)
{
    zmq::message_t msg;
    while (sub.recv(msg, zmq::recv_flags::dontwait)) {
        usleep(200);
    }
}

void SetRobotJoint(zmq::socket_t &pub,std::vector<double> &joint0,std::vector<double> &joint1,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"] = time;
    data["Robot0"]["joint"] = joint0;
    data["Robot1"]["time"] = time;
    data["Robot1"]["joint"] = joint1;
    pub.send(zmq::buffer("Joint " + data.dump()));
}

void SetRobotJoint(zmq::socket_t &pub,std::vector<double> &joint,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"] = time;
    data["Robot0"]["joint"] = joint;
    pub.send(zmq::buffer("Joint " + data.dump()));
}

void SetRobotCartesian(zmq::socket_t &pub,std::vector<double> &cartesian,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"] = time;
    data["Robot0"]["cartesian"] = cartesian;
    pub.send(zmq::buffer("Cartesian " + data.dump()));
}

int main(int argc, char *argv[])
{
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    publisher.set(zmq::sockopt::sndhwm, 0);     // 0 表示无限小队列，但行为是：不能缓存
    publisher.set(zmq::sockopt::immediate, 1);  // SUB 未连接时直接丢弃
    publisher.bind("tcp://*:8001");

    subscriber.connect("tcp://192.168.0.99:8000");
    subscriber.set(zmq::sockopt::subscribe, "");

    printf("RobotSwitch\n");
    sleep(1);
    RobotSwitch(publisher, true);
    sleep(1);

    printf("GetRobotState\n");
    auto state = GetRobotState(subscriber);
    printf("=====\n");

    // 初始关节和位姿
    std::vector<double> robot0_joint =
        state["Robot0"]["Joint"].get<std::vector<double>>();
    std::vector<double> roobt0_cartesian =
        state["Robot0"]["Cartesian"].get<std::vector<double>>();

    std::vector<double> target0_joint = robot0_joint;

    double time = 0.0;
    double dt   = 0.01;
    double A    = 1.0;
    double f    = 0.1;

    printf("loop\n");
    ClearHistoricalData(subscriber);
    // ========== 打开关节 CSV 文件并写表头 ==========
    std::ofstream csv_joint_compare("joint_target_state_log.csv");
    if (!csv_joint_compare.is_open())
    {
        std::cerr << "Failed to open csv_joint_compare.csv for writing!" << std::endl;
        return 1;
    }
    csv_joint_compare << std::fixed << std::setprecision(6);
    csv_joint_compare << "time,target_joint,state_joint\n";

    // ========== 打开关节 CSV 文件并写表头 ==========
    std::ofstream csv_joint("joint_log.csv");
    if (!csv_joint.is_open())
    {
        std::cerr << "Failed to open joint_log.csv for writing!" << std::endl;
        return 1;
    }
    csv_joint << std::fixed << std::setprecision(6);
    csv_joint << "time,j0,j1,j2,j3,j4,j5,j6\n";

    // ========== 打开末端位姿 CSV 文件并写表头 ==========
    std::ofstream csv_cart("cartesian_log.csv");
    if (!csv_cart.is_open())
    {
        std::cerr << "Failed to open cartesian_log.csv for writing!" << std::endl;
        return 1;
    }
    csv_cart << std::fixed << std::setprecision(6);
    // 假设 Cartesian 向量为 [x, y, z, rx, ry, rz]
    csv_cart << "time,x,y,z,rx,ry,rz\n";

    // ========= 读取当前关节姿态，并先用 SetRobotJoint 到达初始关节位置 =========
    // 关节id 对应的限位rad：
    // joint 0
    int joint_idx = 0;
    double alpha = 0.88;
    double joint_max = +1.57 * alpha;
    double joint_min = -1.57 * alpha;
    // joint 1
    // double alpha = 1.0;
    // int joint_idx = 1;
    // double joint_max = 3.14 * alpha;
    // double joint_min = 0 * alpha;
    // joint 2
    // double alpha = 1.0;
    // int joint_idx = 2;
    // double joint_max = 2.90 * alpha;
    // double joint_min = -2.90 * alpha;
    // joint 3
    // double alpha = 1.0;
    // int joint_idx = 3;
    // double joint_max = 1.70 * alpha;
    // double joint_min = -2.1 * alpha;
    // joint 4
    // double alpha = 1.0;
    // int joint_idx = 4;
    // double joint_max = 2.90 * alpha;
    // double joint_min = -2.90 * alpha;
    // joint 5
    // double alpha = 1.0;
    // int joint_idx = 5;
    // double joint_max = 1.4 * alpha;
    // double joint_min = -1.5 * alpha;
    // joint 6
    // double alpha = 1.0;
    // int joint_idx = 6;
    // double joint_max = 2.90 * alpha;
    // double joint_min = -2.90 * alpha;

    double joint_mid = (joint_max + joint_min)/2;
    
    target0_joint[joint_idx] = joint_mid;
    A = joint_max - target0_joint[joint_idx];

    std::cout << "get initial joint posi: "
                << robot0_joint[0] << ","
                << robot0_joint[1] << ","
                << robot0_joint[2] << ","
                << robot0_joint[3] << ","
                << robot0_joint[4] << ","
                << robot0_joint[5] << ","
                << robot0_joint[6] << "\n";
    std::cout << "get target joint posi: "
                << target0_joint[0] << ","
                << target0_joint[1] << ","
                << target0_joint[2] << ","
                << target0_joint[3] << ","
                << target0_joint[4] << ","
                << target0_joint[5] << ","
                << target0_joint[6] << "\n";

    time = 0.1;  // 关节插补时间（单位秒，可按需要调整）
    std::cout << "Move to initial joint pose with time = "
              << time << " s" << std::endl;
    int kInitialSteps = 1000;
    int idx = 0;
    while (idx++ < kInitialSteps) {
        auto state = GetRobotState(subscriber); // 阻塞等待接收
        SetRobotJoint(publisher, target0_joint, time);
        time+=dt;
    }
    // SetRobotJoint(publisher, target0_joint, time);
    // time +=11.0;
    // double t0 = now_sec();
    // sleep(10);
    // double t1 = now_sec();

    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "before: " << t0 << "\n";
    // std::cout << "after : " << t1 << "\n";
    // std::cout << "delta : " << (t1 - t0) << " sec\n";

    double init_time = time;
    // ========= 开启一个余弦循环 =========
    while (true)
    {
        time += dt;
        // ========== 获取机器人状态 ==========
        auto state = GetRobotState(subscriber); // 阻塞等待接收

        // 实际关节 6 位置
        std::vector<double> state_joint_vec =
            state["Robot0"]["Joint"].get<std::vector<double>>();

        // 实际末端位姿
        std::vector<double> state_cart_vec =
            state["Robot0"]["Cartesian"].get<std::vector<double>>();

        // ========== 发送控制指令 ==========
        double state_joint = state_joint_vec[joint_idx];
        // // 目标关节 6 位置（正弦激励）
        target0_joint[joint_idx] = A * std::sin(M_PI * 2 * f * (time - init_time)) + joint_mid;

        SetRobotJoint(publisher, target0_joint, time);

        // std::cout << "JETest: state_joint6: " << state_joint6
        //           << " published_joint6: " << target0_joint[6] << std::endl;

        // ========== 写入关节 CSV ==========
        csv_joint_compare << time << "," << target0_joint[joint_idx] << "," << state_joint << "\n";

        // ========== 写入末端位姿 CSV ==========
        if (state_cart_vec.size() >= 6)
        {
            csv_cart << time << ","
                     << state_cart_vec[0] << ","
                     << state_cart_vec[1] << ","
                     << state_cart_vec[2] << ","
                     << state_cart_vec[3] << ","
                     << state_cart_vec[4] << ","
                     << state_cart_vec[5] << "\n";
        }
        else
        {
            // 尺寸不对时给个提示
            std::cerr << "Cartesian vector size < 6, size = "
                      << state_cart_vec.size() << std::endl;
        }
        std::cout << "get cart state: "
                     << state_cart_vec[0] << ","
                     << state_cart_vec[1] << ","
                     << state_cart_vec[2] << ","
                     << state_cart_vec[3] << ","
                     << state_cart_vec[4] << ","
                     << state_cart_vec[5] << "\n";
        if (state_joint_vec.size() >= 6)
        {
            csv_joint << time << ","
                     << state_joint_vec[0] << ","
                     << state_joint_vec[1] << ","
                     << state_joint_vec[2] << ","
                     << state_joint_vec[3] << ","
                     << state_joint_vec[4] << ","
                     << state_joint_vec[5] << ","
                     << state_joint_vec[6] << "\n";
        }
        else
        {
            // 尺寸不对时给个提示
            std::cerr << "joint vector size < 7, size = "
                      << state_joint_vec.size() << std::endl;
        }
        std::cout << "get joint posi: "
                     << state_joint_vec[0] << ","
                     << state_joint_vec[1] << ","
                     << state_joint_vec[2] << ","
                     << state_joint_vec[3] << ","
                     << state_joint_vec[4] << ","
                     << state_joint_vec[5] << ","
                     << state_joint_vec[6] << "\n";
    }

    printf("RobotSwitch\n");
    RobotSwitch(publisher, false);

    return 0;
}
