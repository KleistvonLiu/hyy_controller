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
#include <fstream>   // 读 CSV
#include <sstream>   // 拆分每一行

// ===================== 通用工具函数 =====================

void RobotSwitch(zmq::socket_t &pub, bool value)
{
    nlohmann::ordered_json cmd_switch;
    cmd_switch["Switch"] = value;
    pub.send(zmq::buffer("Switch " + cmd_switch.dump()));
}

nlohmann::ordered_json GetRobotState(zmq::socket_t &sub)
{
    nlohmann::ordered_json state_json;
    zmq::message_t msg;
    sub.recv(msg);  // 阻塞等待一帧

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
    while (sub.recv(msg, zmq::recv_flags::dontwait))
    {
        usleep(200);
    }
}

void SetRobotCartesian(zmq::socket_t &pub, std::vector<double> &cartesian, double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"] = time;
    data["Robot0"]["cartesian"] = cartesian;
    pub.send(zmq::buffer("Cartesian " + data.dump()));
}

// ========= 新增：关节控制，先到达初始关节位姿 =========

void SetRobotJoint(zmq::socket_t &pub, std::vector<double> &joint, double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"]  = time;
    data["Robot0"]["joint"] = joint;
    pub.send(zmq::buffer("Joint " + data.dump()));
}

// ===================== 从 CSV 读取 waypoints =====================

bool LoadJointsFromCsv(const std::string &csv_path,
                          std::vector<std::vector<double>> &waypoints)
{
    std::ifstream ifs(csv_path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open csv file: " << csv_path << std::endl;
        return false;
    }

    std::string line;

    // 读取并丢弃第一行表头
    if (!std::getline(ifs, line))
    {
        std::cerr << "Empty csv file: " << csv_path << std::endl;
        return false;
    }

    while (std::getline(ifs, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<double> fields;

        // 按逗号拆分一行
        while (std::getline(ss, field, ','))
        {
            try
            {
                fields.push_back(std::stod(field));
            }
            catch (const std::exception &)
            {
                // 转换失败的字段直接丢弃整行
                fields.clear();
                break;
            }
        }

        // 我们期望一行有 7 个数：time, x, y, z, rx, ry, rz
        if (fields.size() < 7)
            continue;

        // 构造 [x,y,z,rx,ry,rz]
        std::vector<double> pose(7);
        pose[0] = fields[1]; // x
        pose[1] = fields[2]; // y
        pose[2] = fields[3]; // z
        pose[3] = fields[4]; // rx
        pose[4] = fields[5]; // ry
        pose[5] = fields[6]; // rz
        pose[6] = fields[7]; // joint7

        waypoints.push_back(std::move(pose));
    }

    if (waypoints.empty())
    {
        std::cerr << "No valid waypoints loaded from " << csv_path << std::endl;
        return false;
    }

    std::cout << "Loaded " << waypoints.size()
              << " waypoints from " << csv_path << std::endl;
    return true;
}


// ===================== 从 CSV 读取 waypoints =====================

bool LoadWaypointsFromCsv(const std::string &csv_path,
                          std::vector<std::vector<double>> &waypoints)
{
    std::ifstream ifs(csv_path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open csv file: " << csv_path << std::endl;
        return false;
    }

    std::string line;

    // 读取并丢弃第一行表头
    if (!std::getline(ifs, line))
    {
        std::cerr << "Empty csv file: " << csv_path << std::endl;
        return false;
    }

    while (std::getline(ifs, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<double> fields;

        // 按逗号拆分一行
        while (std::getline(ss, field, ','))
        {
            try
            {
                fields.push_back(std::stod(field));
            }
            catch (const std::exception &)
            {
                // 转换失败的字段直接丢弃整行
                fields.clear();
                break;
            }
        }

        // 我们期望一行有 7 个数：time, x, y, z, rx, ry, rz
        if (fields.size() < 7)
            continue;

        // 构造 [x,y,z,rx,ry,rz]
        std::vector<double> pose(6);
        pose[0] = fields[1]; // x
        pose[1] = fields[2]; // y
        pose[2] = fields[3]; // z
        pose[3] = fields[4]; // rx
        pose[4] = fields[5]; // ry
        pose[5] = fields[6]; // rz

        waypoints.push_back(std::move(pose));
    }

    if (waypoints.empty())
    {
        std::cerr << "No valid waypoints loaded from " << csv_path << std::endl;
        return false;
    }

    std::cout << "Loaded " << waypoints.size()
              << " waypoints from " << csv_path << std::endl;
    return true;
}

// ===================== 主函数：发送 CSV 末端位姿轨迹 =====================

int main(int argc, char *argv[])
{
    // 1. ZeroMQ 初始化
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    // PUB：给控制器发送指令
    publisher.set(zmq::sockopt::sndhwm, 0);      // 不缓存队列
    publisher.set(zmq::sockopt::immediate, 1);   // SUB 未连接时丢弃
    publisher.bind("tcp://*:8001");              // 与插件端 subscriber.connect("tcp://192.168.0.35:8001") 对应

    // SUB：从控制器接收状态
    subscriber.connect("tcp://192.168.0.99:8000");
    subscriber.set(zmq::sockopt::subscribe, "");

    printf("RobotSwitch ON\n");
    sleep(1);
    RobotSwitch(publisher, true);  // 上电 + 使能
    sleep(1);

    // 2. 读取当前状态
    printf("GetRobotState\n");
    auto state = GetRobotState(subscriber);
    printf("=====\n");

    // 当前末端位姿
    std::vector<double> robot0_cartesian =
        state["Robot0"]["Cartesian"].get<std::vector<double>>();
    std::cout << "Current Cartesian (Robot0): [";
    for (size_t i = 0; i < robot0_cartesian.size(); ++i)
    {
        std::cout << robot0_cartesian[i];
        if (i + 1 < robot0_cartesian.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // ========= 新增：读取当前关节姿态，并先用 SetRobotJoint 到达初始关节位置 =========
    std::vector<double> robot0_joint =
        state["Robot0"]["Joint"].get<std::vector<double>>();
    std::vector<double> targt0_joint = {0.0001, 0.0000, -1.5619, -0.7539, 0.0001, -0.6506, -0.0003};

    double joint_move_time = 2.0;  // 关节插补时间（单位秒，可按需要调整）
    std::cout << "Move to initial joint pose with time = "
              << joint_move_time << " s" << std::endl;

    SetRobotJoint(publisher, targt0_joint, joint_move_time);
    // 简单等待关节运动完成（更严谨可以轮询状态判断）
    usleep(static_cast<useconds_t>(joint_move_time * 1e6));

    // 3. 从 CSV 预设一组末端位姿
    std::string csv_path = (argc > 1) ? argv[1] : "cartesian_log.csv";
    std::string joint_csv_path = (argc > 1) ? argv[1] : "joint_log.csv";

    std::vector<std::vector<double>> waypoints;
    // if (!LoadWaypointsFromCsv(csv_path, waypoints))
    if (!LoadJointsFromCsv(joint_csv_path, waypoints))
    {
        return 1;
    }

    double move_time = 4.0; // 传给 ServoCartesian 的 time 参数
    double dt        = 0.01;

    printf("Start Cartesian waypoint motion\n");
    ClearHistoricalData(subscriber);

    // 4. 依次发送每一个预设末端位姿
    for (size_t idx = 0; idx < waypoints.size(); ++idx)
    {
        auto state = GetRobotState(subscriber); // 阻塞等待接收
        
        auto &target = waypoints[waypoints.size()- 1 - idx];
        std::cout << "Send waypoint " << idx << ": [";
        for (size_t i = 0; i < target.size(); ++i)
        {
            std::cout << target[i];
            if (i + 1 < target.size()) std::cout << ", ";
        }
        std::cout << "], time = " << move_time << "s" << std::endl;

        // SetRobotCartesian(publisher, target, move_time);
        SetRobotJoint(publisher, target, move_time);
        move_time += dt;  // 如果希望每个点的 time 相同，可以去掉这一行

        // 以 ~100Hz 的节奏播放轨迹
        usleep(static_cast<useconds_t>(dt * 1e6));
    }

    // 5. 结束后可以再读一次状态（可选）
    auto final_state = GetRobotState(subscriber);
    auto final_cartesian =
        final_state["Robot0"]["Cartesian"].get<std::vector<double>>();
    std::cout << "Final Cartesian (Robot0): [";
    for (size_t i = 0; i < final_cartesian.size(); ++i)
    {
        std::cout << final_cartesian[i];
        if (i + 1 < final_cartesian.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 6. 关闭伺服/下电
    printf("RobotSwitch OFF\n");
    RobotSwitch(publisher, false);

    return 0;
}
