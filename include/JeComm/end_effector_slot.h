#ifndef END_EFFECTOR_SLOT_H_
#define END_EFFECTOR_SLOT_H_

#include "nlohmann/json.hpp"

#include <memory>
#include <string>

struct EndEffectorSlotState
{
    int slot_index;
    std::string type;
    bool ready;
    double current_position;
};

class IEndEffectorSlot
{
public:
    virtual ~IEndEffectorSlot() {}

    virtual bool Init(std::string* err) = 0;
    virtual void AcceptCommand(const nlohmann::ordered_json& cmd,
                               const char* context,
                               bool debug_log) = 0;
    virtual EndEffectorSlotState GetState() const = 0;
};

std::unique_ptr<IEndEffectorSlot> BuildEndEffectorSlotFromConfig(
    const nlohmann::json& slot_cfg,
    int slot_index,
    std::string* err);

#endif // END_EFFECTOR_SLOT_H_
