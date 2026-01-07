#include "HYYRobotInterface.h"
#include "device_interface.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <fstream>   // ① 增加这一行
#include <thread>
#include "nlohmann/json.hpp"
#include <vector>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
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

// End effector serial config (adjust port if needed)
static const char* kEndEffectorPort = "/dev/ttyS1";
static constexpr speed_t kEndEffectorBaud = B115200;
static int end_effector_fd = -1;

static int init_serial(const char* port, speed_t baudrate)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        std::cerr << "open serial failed: " << port << " err=" << strerror(errno) << "\n";
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "tcgetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "tcsetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    return fd;
}

static uint8_t checksum_sum(const std::vector<uint8_t>& payload)
{
    uint32_t sum = 0;
    for (uint8_t b : payload)
    {
        sum += b;
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

static bool write_all(int fd, const uint8_t* data, size_t len)
{
    size_t off = 0;
    while (off < len)
    {
        ssize_t n = ::write(fd, data + off, len - off);
        if (n < 0)
        {
            if (errno == EINTR) continue;
            return false;
        }
        off += (size_t)n;
    }
    return true;
}

static bool send_end_effector_frame(const std::vector<uint8_t>& payload)
{
    if (end_effector_fd < 0)
    {
        std::cerr << "end effector serial not ready, skip command\n";
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 3);
    frame.push_back(0x55);
    frame.push_back(0xAA);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(checksum_sum(payload));

    if (!write_all(end_effector_fd, frame.data(), frame.size())) {
        std::cerr << "end effector write failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

static void end_effector_stop_all()
{
    // 55 AA 05 06 01 02 03 04 05 06 20
    send_end_effector_frame({0x05, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06});
}

static void end_effector_stop_motor(uint8_t motor_id)
{
    // 55 AA 05 01 <ID> <checksum>
    send_end_effector_frame({0x05, 0x01, motor_id});
}

static void end_effector_two_finger(uint8_t percent)
{
    std::cout << "send percent: 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(percent)
            << std::dec << std::endl;
    // 55 AA 09 <percent> 00 <checksum>
    send_end_effector_frame({0x09, percent, 0x00});
}

static void end_effector_three_finger(uint8_t percent)
{
    std::cout << "send percent: 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(percent)
            << std::dec << std::endl;
    // 55 AA 0A <percent> 00 <checksum>
    send_end_effector_frame({0x0A, percent, 0x00});
}

static void end_effector_set_positions(uint8_t m1, uint8_t m2, uint8_t m3,
                                       uint8_t m4, uint8_t m5, uint16_t m6)
{
    // 55 AA 03 06 01 02 03 04 05 06 m1 m2 m3 m4 m5 m6_hi m6_lo <checksum>
    std::vector<uint8_t> payload = {
        0x03, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        m1, m2, m3, m4, m5,
        static_cast<uint8_t>((m6 >> 8) & 0xFF),
        static_cast<uint8_t>(m6 & 0xFF)
    };
    send_end_effector_frame(payload);
}

static void handle_end_effector_preset(int preset)
{
    // std::cout << "handle_ee_preset \n";
    switch (preset)
    {
        case 0:
            end_effector_stop_all();
            break;
        case 1:
            end_effector_stop_motor(0x01);
            break;
        case 2:
            end_effector_stop_motor(0x02);
            break;
        case 3:
            end_effector_stop_motor(0x03);
            break;
        case 4:
            end_effector_stop_motor(0x04);
            break;
        case 5:
            end_effector_stop_motor(0x05);
            break;
        case 6:
            end_effector_stop_motor(0x06);
            break;
        case 10:
            end_effector_two_finger(0x00);
            break;
        case 11:
            end_effector_two_finger(0x47);
            break;
        case 20:
            end_effector_three_finger(0x00);
            break;
        case 21:
            end_effector_three_finger(0x58);
            break;
        default:
            std::cerr << "unknown end effector preset: " << preset << "\n";
            break;
    }
}

static void handle_end_effector_position(double position)
{
    // std::cout << "handle_ee_position \n";

    int pos = static_cast<int>(std::lround(position));
    if (pos < 0) pos = 0;
    if (pos > 2000) pos = 2000;

    // motor6 uses 0~2000, other motors keep at 0 by default
    end_effector_set_positions(0, 0, 0, 0, 0, static_cast<uint16_t>(pos));
}

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
    HYYRobotBase::initUserTimer(&timer, 0, CYCLIE); // 10ms
    nlohmann::ordered_json data;

    // 参照示例：拿到 device_name，后续按 i 拿 robot_name（子设备名）
    const char* device_name = HYYRobotBase::get_deviceName(0, nullptr);

    printf("start publisher_loop\n");
    while (true)
    {
        HYYRobotBase::userTimer(&timer);

        int rn = HYYRobotBase::robot_getNUM();
        for (int i = 0; i < rn; i++)
        {
            const std::string rk = std::string("Robot") + std::to_string(i);

            data[rk]["MoveState"]  = HYYRobotBase::get_robot_move_state(i);
            data[rk]["PowerState"] = HYYRobotBase::GetRobotPowerState(i);

            int dof = HYYRobotBase::robot_getDOF(i);

            // -------- Joint position / target --------
            std::vector<double> joint(dof);
            std::vector<double> target_joint(dof);

            // 修正：使用 i（对应每台机器人）
            HYYRobotBase::GetCurrentJoint(joint.data(), i);
            data[rk]["Joint"] = joint;

            HYYRobotBase::GetCurrentLastTargetJoint(target_joint.data(), i);
            data[rk]["TargetJoint"] = target_joint;

            // -------- Joint velocity / torque (NEW) --------
            // 参照示例：获取该 robot 的子设备名
            const char* robot_name = HYYRobotBase::get_name_robot_device(device_name, i);

            std::vector<double> joint_vel(dof, 0.0);
            std::vector<double> joint_torque(dof, 0.0);
            std::vector<double> joint_sensor_torque(dof, 0.0);

            int vel_ret = 0;
            int tq_ret  = 0;
            int stq_ret = 0;
            if (robot_name != nullptr)
            {
                vel_ret = HYYRobotBase::GetGroupVelocity(robot_name, joint_vel.data());
                tq_ret  = HYYRobotBase::GetGroupTorque(robot_name, joint_torque.data());
                stq_ret = HYYRobotBase::GetGroupSensorTorque(robot_name, joint_sensor_torque.data());
            }
            else
            {
                vel_ret = -1;
                tq_ret  = -1;
                stq_ret = -1;

            }

            // 失败时仍发布零向量，并打印日志（避免 JSON 缺字段导致下游解析不一致）
            if (vel_ret < 0)
            {
                std::cerr << "[publisher_loop] GetGroupVelocity failed, robot_index="
                          << i << ", robot_name=" << (robot_name ? robot_name : "null")
                          << ", ret=" << vel_ret << std::endl;
            }
            if (tq_ret < 0)
            {
                std::cerr << "[publisher_loop] GetGroupTorque failed, robot_index="
                          << i << ", robot_name=" << (robot_name ? robot_name : "null")
                          << ", ret=" << tq_ret << std::endl;
            }
            if (stq_ret < 0)
            {
                std::cerr << "[publisher_loop] GetGroupSensorTorque failed, robot_index="
                          << i << ", robot_name=" << (robot_name ? robot_name : "null")
                          << ", ret=" << stq_ret << std::endl;
            }

            data[rk]["JointVelocity"] = joint_vel;
            data[rk]["JointTorque"]   = joint_torque;
            data[rk]["JointSensorTorque"] = joint_sensor_torque;

            // -------- Cartesian / target --------
            std::vector<double> Cartesian(6);

            // 修正：使用 i（对应每台机器人）
            HYYRobotBase::GetCurrentCartesian(NULL, NULL, (HYYRobotBase::robpose*)Cartesian.data(), i);
            data[rk]["Cartesian"] = Cartesian;

            HYYRobotBase::GetCurrentLastTargetCartesian(NULL, NULL, (HYYRobotBase::robpose*)Cartesian.data(), i);
            data[rk]["TargetCartesian"] = Cartesian;
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
            // std::cout << "received joint: " << std::endl;
            for (int i=0;i<rn;i++)
            {
                const std::string rk = std::string("Robot")+std::to_string(i);
                if (cmd_json.contains(rk))
                {
                    if (!cmd_json[rk].contains("time") || !cmd_json[rk]["time"].is_number())
                    {
                        std::cerr << "[Joint] missing/invalid time for " << rk << "\n";
                        continue;
                    }
                    if (!cmd_json[rk].contains("joint") || !cmd_json[rk]["joint"].is_array())
                    {
                        std::cerr << "[Joint] missing/invalid joint for " << rk << "\n";
                        continue;
                    }
                    double time=cmd_json[rk]["time"].get<double>();
                    //printf("===%f\n",time);
                    std::vector<double> joint=cmd_json[rk]["joint"].get<std::vector<double>>();
                    HYYRobotBase::robjoint jt;
                    HYYRobotBase::init_robjoint(&jt,joint.data(),HYYRobotBase::robot_getDOF(i));
                    HYYRobotBase::ServoJoint(&jt,time,i);

                    if (cmd_json[rk].contains("end_effector"))
                    {
                        const auto& ee = cmd_json[rk]["end_effector"];
                        if (ee.contains("mode") && ee["mode"].is_number_integer())
                        {
                            int mode = ee["mode"].get<int>();
                            if (mode == 0 && ee.contains("position") && ee["position"].is_number())
                            {
                                handle_end_effector_position(ee["position"].get<double>());
                            }
                            else if (mode == 1 && ee.contains("preset") && ee["preset"].is_number_integer())
                            {
                                handle_end_effector_preset(ee["preset"].get<int>());
                            }
                            else
                            {
                                std::cerr << "[Joint] end_effector missing/invalid fields for " << rk << "\n";
                            }
                        }
                        else
                        {
                            std::cerr << "[Joint] end_effector missing/invalid mode for " << rk << "\n";
                        }
                    }
                }
            }
        }
        else if ("Cartesian"==topic)
        {
            // std::cout << "received joint: " << std::endl;
            for (int i=0;i<rn;i++)
            {
                const std::string rk = std::string("Robot")+std::to_string(i);
                if (cmd_json.contains(rk))
                {
                    if (!cmd_json[rk].contains("time") || !cmd_json[rk]["time"].is_number())
                    {
                        std::cerr << "[Cartesian] missing/invalid time for " << rk << "\n";
                        continue;
                    }
                    if (!cmd_json[rk].contains("cartesian") || !cmd_json[rk]["cartesian"].is_array())
                    {
                        std::cerr << "[Cartesian] missing/invalid cartesian for " << rk << "\n";
                        continue;
                    }
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

                    if (cmd_json[rk].contains("end_effector"))
                    {
                        // std::cout << "here \n";
                        const auto& ee = cmd_json[rk]["end_effector"];
                        if (ee.contains("mode") && ee["mode"].is_number_integer())
                        {
                            int mode = ee["mode"].get<int>();
                            if (mode == 0 && ee.contains("position") && ee["position"].is_number())
                            {
                                handle_end_effector_position(ee["position"].get<double>());
                            }
                            else if (mode == 1 && ee.contains("preset") && ee["preset"].is_number_integer())
                            {
                                handle_end_effector_preset(ee["preset"].get<int>());
                            }
                            else
                            {
                                std::cerr << "[Cartesian] end_effector missing/invalid fields for " << rk << "\n";
                            }
                        }
                        else
                        {
                            std::cerr << "[Cartesian] end_effector missing/invalid mode for " << rk << "\n";
                        }
                    }
                }
            }
        }
    }
}

void PluginMain()
{
    end_effector_fd = init_serial(kEndEffectorPort, kEndEffectorBaud);
    if (end_effector_fd < 0)
    {
        std::cerr << "end effector serial init failed, end effector disabled\n";
    }
    publisher.set(zmq::sockopt::sndhwm, 0);  // 0 表示无限小队列，但行为是：不能缓存
    publisher.set(zmq::sockopt::immediate, 1);  // SUB 未连接时直接丢弃
    publisher.bind("tcp://*:8000");
    pub_th=new std::thread(publisher_loop);
    pub_th->detach();
    subscriber.connect("tcp://192.168.0.35:8001");
    // subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::rcvhwm, 1);
    subscriber.set(zmq::sockopt::conflate, 1);   // 只保留最后一条
    subscriber.set(zmq::sockopt::subscribe, "Switch ");
    subscriber.set(zmq::sockopt::subscribe, "Cartesian ");  // 只收你需要的
    subscriber.set(zmq::sockopt::subscribe, "Joint ");  // 只收你需要的
    sub_th=new std::thread(subscriber_loop);
    sub_th->detach();
}
