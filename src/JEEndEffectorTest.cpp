#include "end_effector_manager.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
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
              << " [--config path] [--sub endpoint] [--pub-bind endpoint] [--state-ms ms] [--dof n]\n";
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

struct RobotMirrorState
{
    int move_state;
    int power_state;
    std::vector<double> joint;
    std::vector<double> target_joint;
    std::vector<double> joint_velocity;
    std::vector<double> joint_torque;
    std::vector<double> joint_sensor_torque;
    std::vector<double> cartesian;
    std::vector<double> target_cartesian;
};

static std::vector<RobotMirrorState> init_robot_states(int robot_num, int dof)
{
    const int fixed_dof = (dof <= 0) ? 7 : dof;
    std::vector<RobotMirrorState> states(static_cast<size_t>(robot_num));
    for (int i = 0; i < robot_num; ++i)
    {
        RobotMirrorState s;
        s.move_state = 0;
        s.power_state = 1;
        s.joint.assign(static_cast<size_t>(fixed_dof), 0.0);
        s.target_joint.assign(static_cast<size_t>(fixed_dof), 0.0);
        s.joint_velocity.assign(static_cast<size_t>(fixed_dof), 0.0);
        s.joint_torque.assign(static_cast<size_t>(fixed_dof), 0.0);
        s.joint_sensor_torque.assign(static_cast<size_t>(fixed_dof), 0.0);
        s.cartesian.assign(6, 0.0);
        s.target_cartesian.assign(6, 0.0);
        states[static_cast<size_t>(i)] = s;
    }
    return states;
}

static bool parse_numeric_array(const nlohmann::ordered_json& node, std::vector<double>* out)
{
    if (!out || !node.is_array())
        return false;
    std::vector<double> values;
    values.reserve(node.size());
    for (size_t i = 0; i < node.size(); ++i)
    {
        if (!node[i].is_number())
            return false;
        values.push_back(node[i].get<double>());
    }
    *out = values;
    return true;
}

nlohmann::ordered_json build_state_json(const EndEffectorManager& manager,
                                        const std::vector<RobotMirrorState>& robot_states)
{
    nlohmann::ordered_json data;
    for (size_t i = 0; i < robot_states.size(); ++i)
    {
        const std::string rk = std::string("Robot") + std::to_string(i);
        const EndEffectorSlotState slot_state = manager.GetSlotState(static_cast<int>(i));
        const RobotMirrorState& rs = robot_states[i];
        data[rk]["MoveState"] = rs.move_state;
        data[rk]["PowerState"] = rs.power_state;
        data[rk]["Joint"] = rs.joint;
        data[rk]["TargetJoint"] = rs.target_joint;
        data[rk]["JointVelocity"] = rs.joint_velocity;
        data[rk]["JointTorque"] = rs.joint_torque;
        data[rk]["JointSensorTorque"] = rs.joint_sensor_torque;
        data[rk]["Cartesian"] = rs.cartesian;
        data[rk]["TargetCartesian"] = rs.target_cartesian;
        data[rk]["EndEffector"]["CurrentPosition"] = slot_state.current_position;
    }

    const std::vector<EndEffectorSlotState> slot_states = manager.GetAllStates();
    data["EndEffectors"] = nlohmann::ordered_json::array();
    for (size_t idx = 0; idx < slot_states.size(); ++idx)
    {
        nlohmann::ordered_json slot_json;
        slot_json["slot_index"] = slot_states[idx].slot_index;
        slot_json["type"] = slot_states[idx].type;
        slot_json["ready"] = slot_states[idx].ready;
        slot_json["current_position"] = slot_states[idx].current_position;
        data["EndEffectors"].push_back(slot_json);
    }

    return data;
}

void handle_joint_or_cartesian(const std::string& topic,
                               const nlohmann::ordered_json& cmd_json,
                               EndEffectorManager* manager,
                               std::vector<RobotMirrorState>* robot_states)
{
    if (!manager || !robot_states)
        return;
    for (size_t i = 0; i < robot_states->size(); ++i)
    {
        const std::string robot_key = std::string("Robot") + std::to_string(i);
        if (!cmd_json.contains(robot_key) || !cmd_json[robot_key].is_object())
            continue;

        const nlohmann::ordered_json& robot_payload = cmd_json[robot_key];
        RobotMirrorState& rs = (*robot_states)[i];

        if (topic == "Joint")
        {
            std::vector<double> joint;
            if (robot_payload.contains("joint") &&
                parse_numeric_array(robot_payload["joint"], &joint) &&
                !joint.empty())
            {
                rs.joint = joint;
                rs.target_joint = joint;
            }
        }
        else if (topic == "Cartesian")
        {
            std::vector<double> cartesian;
            if (robot_payload.contains("cartesian") &&
                parse_numeric_array(robot_payload["cartesian"], &cartesian) &&
                !cartesian.empty())
            {
                rs.cartesian = cartesian;
                rs.target_cartesian = cartesian;
            }
        }

        if (!robot_payload.contains("EndEffector"))
            continue;
        std::cout << "here1" <<std::endl;
        manager->DispatchByRobotIndex(static_cast<int>(i), robot_payload["EndEffector"], topic.c_str(), true);
    }
}

void handle_gripper(const nlohmann::ordered_json& cmd_json,
                    EndEffectorManager* manager,
                    size_t robot_num)
{
    if (!manager)
        return;

    for (size_t i = 0; i < robot_num; ++i)
    {
        const std::string robot_key = std::string("Robot") + std::to_string(i);
        if (!cmd_json.contains(robot_key) || !cmd_json[robot_key].is_object())
            continue;

        const nlohmann::ordered_json& robot_payload = cmd_json[robot_key];
        if (robot_payload.contains("EndEffector") && robot_payload["EndEffector"].is_object())
        {
            manager->DispatchByRobotIndex(static_cast<int>(i), robot_payload["EndEffector"], "Gripper", true);
            continue;
        }

        manager->DispatchByRobotIndex(static_cast<int>(i), robot_payload, "Gripper", true);
    }
}

bool parse_args(int argc,
                char** argv,
                std::string* config_path,
                std::string* sub_endpoint,
                std::string* pub_bind_endpoint,
                int* state_period_ms,
                int* dof)
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
        if (arg == "--pub-bind" && i + 1 < argc)
        {
            *pub_bind_endpoint = argv[++i];
            continue;
        }
        if (arg == "--state-ms" && i + 1 < argc)
        {
            *state_period_ms = std::atoi(argv[++i]);
            continue;
        }
        if (arg == "--dof" && i + 1 < argc)
        {
            *dof = std::atoi(argv[++i]);
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
    std::string pub_bind_endpoint = "tcp://*:8000";
    int state_period_ms = 10;
    int dof = 7;

    if (!parse_args(argc, argv, &config_path, &sub_endpoint, &pub_bind_endpoint, &state_period_ms, &dof))
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
              << ", sub=" << sub_endpoint
              << ", pub_bind=" << pub_bind_endpoint
              << ", dof=" << dof
              << std::endl;

    std::vector<RobotMirrorState> robot_states = init_robot_states(robot_num, dof);

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    publisher.set(zmq::sockopt::sndhwm, 0);
    publisher.set(zmq::sockopt::immediate, 1);
    publisher.bind(pub_bind_endpoint);

    subscriber.set(zmq::sockopt::rcvhwm, 1);
    subscriber.set(zmq::sockopt::conflate, 1);
    subscriber.set(zmq::sockopt::rcvtimeo, 50);
    // subscriber.set(zmq::sockopt::subscribe, "");
    // subscriber.set(zmq::sockopt::subscribe, "Switch");
    // subscriber.set(zmq::sockopt::subscribe, "Switch ");
    // subscriber.set(zmq::sockopt::subscribe, "Cartesian");
    // subscriber.set(zmq::sockopt::subscribe, "Cartesian ");
    // subscriber.set(zmq::sockopt::subscribe, "Joint");
    subscriber.set(zmq::sockopt::subscribe, "Joint ");
    subscriber.set(zmq::sockopt::subscribe, "Gripper ");
    subscriber.connect(sub_endpoint);

    const int publish_period_ms = (state_period_ms <= 0) ? 500 : state_period_ms;
    std::chrono::steady_clock::time_point next_publish = std::chrono::steady_clock::now();
    const std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
    uint64_t loop_count = 0;
    uint64_t publish_count = 0;

    std::cout << "[loop] start main loop, publish_period_ms=" << publish_period_ms << std::endl;
    while (!g_stop.load(std::memory_order_relaxed))
    {
        ++loop_count;
        if (loop_count == 1 || (loop_count % 100) == 0)
        {
            const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::steady_clock::now() - loop_start)
                                           .count();
            std::cout << "[loop] heartbeat iter=" << loop_count
                      << " elapsed_ms=" << elapsed_ms
                      << std::endl;
        }

        std::cout << "[loop] iter=" << loop_count << " before recv()" << std::endl;
        zmq::message_t msg;
        const auto recv_ok = subscriber.recv(msg);
        std::cout << "[loop] iter=" << loop_count
                  << " after recv ok=" << (recv_ok ? "true" : "false")
                  << " msg_size=" << msg.size()
                  << std::endl;
        if (recv_ok)
        {
            const std::string cmd(static_cast<char*>(msg.data()), msg.size());
            std::cout << "[recv] raw: " << cmd << std::endl;
            const std::string::size_type pos = cmd.find(' ');
            if (pos == std::string::npos || pos + 1 >= cmd.size())
            {
                std::cerr << "[recv] invalid message format: " << cmd << std::endl;
            }
            else
            {
                const std::string topic = cmd.substr(0, pos);
                nlohmann::ordered_json cmd_json;
                try
                {
                    cmd_json = nlohmann::json::parse(cmd.substr(pos + 1));
                }
                catch (const std::exception& e)
                {
                    std::cerr << "[recv] json parse failed: " << e.what() << std::endl;
                    cmd_json = nlohmann::ordered_json();
                }

                if (topic == "Joint" || topic == "Cartesian")
                {
                    std::cout << "[recv] topic=" << topic << " dispatch EndEffector" << std::endl;
                    handle_joint_or_cartesian(topic, cmd_json, &manager, &robot_states);
                }
                else if (topic == "Gripper")
                {
                    std::cout << "[recv] topic=Gripper dispatch EndEffector" << std::endl;
                    handle_gripper(cmd_json, &manager, robot_states.size());
                }
                else if (topic == "Switch")
                {
                    std::cout << "[recv] topic=Switch ignored in end-effector-only test"
                              << std::endl;
                }
                else
                {
                    std::cout << "[recv] unsupported topic=" << topic << std::endl;
                }
            }
        }

        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (now >= next_publish)
        {
            const nlohmann::ordered_json state = build_state_json(manager, robot_states);
            const auto send_ok = publisher.send(zmq::buffer("State " + state.dump()));
            ++publish_count;
            std::cout << "[pub] count=" << publish_count
                      << " iter=" << loop_count
                      << " send_ok=" << (send_ok ? "true" : "false")
                      << std::endl;
            next_publish = now + std::chrono::milliseconds(publish_period_ms);
        }
    }

    std::cout << "[exit] stopped" << std::endl;
    return 0;
}
