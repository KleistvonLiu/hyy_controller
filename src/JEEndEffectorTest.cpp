#include "end_effector_manager.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include <zmq.hpp>

namespace
{

std::atomic<bool> g_stop(false);

void on_signal(int)
{
    g_stop.store(true, std::memory_order_relaxed);
}

void print_usage(const char* prog)
{
    std::cerr << "Usage: " << prog
              << " [--config path] [--sub endpoint] [--state-ms ms]\n";
}

bool read_end_effector_count(const std::string& config_path, int* out_count, std::string* err)
{
    if (!out_count)
    {
        if (err)
            *err = "out_count is null";
        return false;
    }

    std::ifstream ifs(config_path.c_str());
    if (!ifs.is_open())
    {
        if (err)
            *err = "failed to open config file: " + config_path;
        return false;
    }

    nlohmann::json root;
    try
    {
        ifs >> root;
    }
    catch (const std::exception& e)
    {
        if (err)
            *err = std::string("failed to parse config json: ") + e.what();
        return false;
    }

    if (!root.is_object() ||
        !root.contains("end_effectors") ||
        !root["end_effectors"].is_array())
    {
        if (err)
            *err = "config must contain array field 'end_effectors'";
        return false;
    }

    *out_count = static_cast<int>(root["end_effectors"].size());
    return true;
}

void dump_end_effector_states(const EndEffectorManager& manager)
{
    const std::vector<EndEffectorSlotState> states = manager.GetAllStates();
    for (size_t i = 0; i < states.size(); ++i)
    {
        const EndEffectorSlotState& s = states[i];
        std::cout << "[state] slot=" << s.slot_index
                  << " type=" << s.type
                  << " ready=" << (s.ready ? "true" : "false")
                  << " current_position=" << s.current_position
                  << std::endl;
    }
}

void handle_joint_or_cartesian(const std::string& topic,
                               const nlohmann::ordered_json& cmd_json,
                               EndEffectorManager* manager,
                               int robot_num)
{
    for (int i = 0; i < robot_num; ++i)
    {
        const std::string robot_key = std::string("Robot") + std::to_string(i);
        if (!cmd_json.contains(robot_key) || !cmd_json[robot_key].is_object())
            continue;

        const nlohmann::ordered_json& robot_payload = cmd_json[robot_key];
        if (!robot_payload.contains("EndEffector"))
            continue;

        manager->DispatchByRobotIndex(i, robot_payload["EndEffector"], topic.c_str(), true);
    }
}

bool parse_args(int argc,
                char** argv,
                std::string* config_path,
                std::string* sub_endpoint,
                int* state_period_ms)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc)
        {
            *config_path = argv[++i];
            continue;
        }
        if (arg == "--sub" && i + 1 < argc)
        {
            *sub_endpoint = argv[++i];
            continue;
        }
        if (arg == "--state-ms" && i + 1 < argc)
        {
            *state_period_ms = std::atoi(argv[++i]);
            continue;
        }
        print_usage(argv[0]);
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    std::string config_path = "config/jeserver_end_effectors.json";
    std::string sub_endpoint = "tcp://192.168.0.35:8001";
    int state_period_ms = 500;

    if (!parse_args(argc, argv, &config_path, &sub_endpoint, &state_period_ms))
        return 1;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    int robot_num = 0;
    std::string err;
    if (!read_end_effector_count(config_path, &robot_num, &err))
    {
        std::cerr << "[init] read config failed: " << err << std::endl;
        return 1;
    }

    EndEffectorManager manager;
    if (!manager.LoadAndInit(config_path, robot_num, &err))
    {
        std::cerr << "[init] end effector manager init failed: " << err << std::endl;
        return 1;
    }

    std::cout << "[init] EndEffectorManager initialized, slots=" << manager.SlotCount()
              << ", sub=" << sub_endpoint << std::endl;

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.set(zmq::sockopt::rcvhwm, 1);
    subscriber.set(zmq::sockopt::conflate, 1);
    subscriber.set(zmq::sockopt::rcvtimeo, 100);
    subscriber.set(zmq::sockopt::subscribe, "Switch ");
    subscriber.set(zmq::sockopt::subscribe, "Cartesian ");
    subscriber.set(zmq::sockopt::subscribe, "Joint ");
    subscriber.connect(sub_endpoint);

    std::thread state_thread([&manager, state_period_ms]() {
        while (!g_stop.load(std::memory_order_relaxed))
        {
            dump_end_effector_states(manager);
            if (state_period_ms <= 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(state_period_ms));
            }
        }
    });

    while (!g_stop.load(std::memory_order_relaxed))
    {
        zmq::message_t msg;
        const auto recv_ok = subscriber.recv(msg);
        if (!recv_ok)
            continue;

        const std::string cmd(static_cast<char*>(msg.data()), msg.size());
        const std::string::size_type pos = cmd.find(' ');
        if (pos == std::string::npos || pos + 1 >= cmd.size())
        {
            std::cerr << "[recv] invalid message format: " << cmd << std::endl;
            continue;
        }

        const std::string topic = cmd.substr(0, pos);
        nlohmann::ordered_json cmd_json;
        try
        {
            cmd_json = nlohmann::json::parse(cmd.substr(pos + 1));
        }
        catch (const std::exception& e)
        {
            std::cerr << "[recv] json parse failed: " << e.what() << std::endl;
            continue;
        }

        if (topic == "Joint" || topic == "Cartesian")
        {
            handle_joint_or_cartesian(topic, cmd_json, &manager, robot_num);
            continue;
        }

        if (topic == "Switch")
        {
            std::cout << "[recv] Switch topic received, ignored in end-effector-only test"
                      << std::endl;
            continue;
        }

        std::cout << "[recv] unsupported topic: " << topic << std::endl;
    }

    if (state_thread.joinable())
        state_thread.join();

    std::cout << "[exit] stopped" << std::endl;
    return 0;
}
