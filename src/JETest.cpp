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
    // RobotSwitch(publisher, true);
    sleep(1);

    printf("GetRobotState\n");
    auto state = GetRobotState(subscriber);
    printf("=====\n");

    // 初始关节和位姿
    std::vector<double> roobt0_joint =
        state["Robot0"]["Joint"].get<std::vector<double>>();
    std::vector<double> roobt0_cartesian =
        state["Robot0"]["Cartesian"].get<std::vector<double>>();

    std::vector<double> targt0_joint = roobt0_joint;

    double time = 0.0;
    double dt   = 0.01;
    double A    = 1.0;
    double f    = 0.1;

    printf("loop\n");
    ClearHistoricalData(subscriber);

    // ========== 打开关节 CSV 文件并写表头 ==========
    std::ofstream csv_joint("joint_log.csv");
    if (!csv_joint.is_open())
    {
        std::cerr << "Failed to open joint_log.csv for writing!" << std::endl;
        return 1;
    }
    csv_joint << std::fixed << std::setprecision(6);
    csv_joint << "time,target_joint6,state_joint6\n";

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

    while (1)
    {
        auto state = GetRobotState(subscriber); // 阻塞等待接收

        // // 实际关节 6 位置
        // std::vector<double> state_joint_vec =
        //     state["Robot0"]["Joint"].get<std::vector<double>>();
        // double state_joint6 = state_joint_vec[6];

        // 实际末端位姿
        std::vector<double> state_cart_vec =
            state["Robot0"]["Cartesian"].get<std::vector<double>>();

        // // 目标关节 6 位置（正弦激励）
        // targt0_joint[6] = A * std::cos(3.14 * 2 * f * time) - A + roobt0_joint[6];

        // SetRobotJoint(publisher, targt0_joint, time);

        // std::cout << "JETest: state_joint6: " << state_joint6
        //           << " published_joint6: " << targt0_joint[6] << std::endl;

        // ========== 写入关节 CSV ==========
        // csv_joint << time << "," << targt0_joint[6] << "," << state_joint6 << "\n";

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
        std::cout << state_cart_vec[0] << ","
                     << state_cart_vec[1] << ","
                     << state_cart_vec[2] << ","
                     << state_cart_vec[3] << ","
                     << state_cart_vec[4] << ","
                     << state_cart_vec[5] << "\n";
        time += dt;
    }

    printf("RobotSwitch\n");
    RobotSwitch(publisher, false);

    return 0;
}
