#include "end_effector_manager.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

bool EndEffectorManager::LoadAndInit(const std::string& config_path,
                                     int robot_num,
                                     std::string* err)
{
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

    const nlohmann::json& arr = root["end_effectors"];
    if (static_cast<int>(arr.size()) != robot_num)
    {
        if (err)
        {
            *err = "slot number mismatch: config=" +
                   std::to_string(arr.size()) +
                   ", robot_num=" +
                   std::to_string(robot_num);
        }
        return false;
    }

    std::vector<std::unique_ptr<IEndEffectorSlot> > new_slots;
    new_slots.reserve(arr.size());

    for (size_t i = 0; i < arr.size(); ++i)
    {
        std::string slot_err;
        std::unique_ptr<IEndEffectorSlot> slot = BuildEndEffectorSlotFromConfig(
            arr[i], static_cast<int>(i), &slot_err);
        if (!slot)
        {
            if (err)
            {
                *err = "slot[" + std::to_string(i) + "] invalid: " + slot_err;
            }
            return false;
        }

        if (!slot->Init(&slot_err))
        {
            if (err)
            {
                *err = "slot[" + std::to_string(i) + "] init failed: " + slot_err;
            }
            return false;
        }
        new_slots.push_back(std::move(slot));
    }

    slots_.swap(new_slots);
    return true;
}

void EndEffectorManager::DispatchByRobotIndex(int robot_index,
                                              const nlohmann::ordered_json& cmd,
                                              const char* context,
                                              bool debug_log)
{
    if (robot_index < 0 || robot_index >= static_cast<int>(slots_.size()))
    {
        std::cerr << context << " no bound end effector for robot_index="
                  << robot_index << std::endl;
        return;
    }

    slots_[robot_index]->AcceptCommand(cmd, context, debug_log);
}

EndEffectorSlotState EndEffectorManager::GetSlotState(int slot_index) const
{
    if (slot_index < 0 || slot_index >= static_cast<int>(slots_.size()))
    {
        EndEffectorSlotState state;
        state.slot_index = slot_index;
        state.type = "none";
        state.ready = false;
        state.current_position = -1.0;
        return state;
    }

    return slots_[slot_index]->GetState();
}

std::vector<EndEffectorSlotState> EndEffectorManager::GetAllStates() const
{
    std::vector<EndEffectorSlotState> states;
    states.reserve(slots_.size());
    for (size_t i = 0; i < slots_.size(); ++i)
    {
        states.push_back(slots_[i]->GetState());
    }
    return states;
}

int EndEffectorManager::SlotCount() const
{
    return static_cast<int>(slots_.size());
}
