#ifndef END_EFFECTOR_MANAGER_H_
#define END_EFFECTOR_MANAGER_H_

#include "end_effector_slot.h"

#include <string>
#include <vector>

class EndEffectorManager
{
public:
    bool LoadAndInit(const std::string& config_path, int robot_num, std::string* err);

    void DispatchByRobotIndex(int robot_index,
                              const nlohmann::ordered_json& cmd,
                              const char* context,
                              bool debug_log);

    EndEffectorSlotState GetSlotState(int slot_index) const;
    std::vector<EndEffectorSlotState> GetAllStates() const;
    int SlotCount() const;

private:
    std::vector<std::unique_ptr<IEndEffectorSlot> > slots_;
};

#endif // END_EFFECTOR_MANAGER_H_
