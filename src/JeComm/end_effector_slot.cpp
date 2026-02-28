#include "end_effector_slot.h"

#include "dh_modbus_gripper.h"
#include "dh_transport.h"
#include "end_effector_device.h"

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

namespace
{

static void log_with_context(const char* context, const std::string& msg)
{
    std::cerr << context << " " << msg << std::endl;
}

static bool get_required_string(const nlohmann::json& cfg,
                                const char* key,
                                std::string* out,
                                std::string* err)
{
    if (!cfg.contains(key) || !cfg[key].is_string())
    {
        if (err)
            *err = std::string("missing/invalid string field '") + key + "'";
        return false;
    }
    *out = cfg[key].get<std::string>();
    return true;
}

static bool get_required_int(const nlohmann::json& cfg,
                             const char* key,
                             int* out,
                             std::string* err)
{
    if (!cfg.contains(key) || !cfg[key].is_number_integer())
    {
        if (err)
            *err = std::string("missing/invalid integer field '") + key + "'";
        return false;
    }
    *out = cfg[key].get<int>();
    return true;
}

class NoneEndEffectorSlot : public IEndEffectorSlot
{
public:
    explicit NoneEndEffectorSlot(int slot_index)
        : slot_index_(slot_index)
    {
    }

    bool Init(std::string* err) override
    {
        (void)err;
        ready_.store(true, std::memory_order_relaxed);
        return true;
    }

    void AcceptCommand(const nlohmann::ordered_json& cmd,
                       const char* context,
                       bool debug_log) override
    {
        (void)cmd;
        (void)context;
        (void)debug_log;
        // Placeholder slot: silently ignore commands by design.
    }

    EndEffectorSlotState GetState() const override
    {
        EndEffectorSlotState state;
        state.slot_index = slot_index_;
        state.type = "none";
        state.ready = ready_.load(std::memory_order_relaxed);
        state.current_position = -1.0;
        return state;
    }

private:
    int slot_index_;
    std::atomic<bool> ready_{false};
};

class DhGripperEndEffectorSlot : public IEndEffectorSlot
{
public:
    DhGripperEndEffectorSlot(int slot_index,
                             const std::string& port,
                             int baud,
                             int id)
        : slot_index_(slot_index),
          port_(port),
          baud_(baud),
          id_(id)
    {
    }

    bool Init(std::string* err) override
    {
        gripper_ = std::unique_ptr<DH_Modbus_Gripper>(
            new DH_Modbus_Gripper(id_, port_, baud_));
        if (gripper_->open() < 0)
        {
            if (err)
                *err = "DH gripper open failed";
            gripper_.reset();
            ready_.store(false, std::memory_order_relaxed);
            return false;
        }

        int init_state = 0;
        bool ok = gripper_->GetInitState(init_state);
        if (!ok)
        {
            if (err)
                *err = "DH gripper GetInitState failed";
            ready_.store(false, std::memory_order_relaxed);
            return false;
        }

        if (init_state != DH_Modbus_Gripper::S_INIT_FINISHED)
        {
            if (!gripper_->Initialization())
            {
                if (err)
                    *err = "DH gripper Initialization command failed";
                ready_.store(false, std::memory_order_relaxed);
                return false;
            }

            const int max_try = 40;
            bool init_finished = false;
            for (int i = 0; i < max_try; ++i)
            {
                ok = gripper_->GetInitState(init_state);
                if (ok && init_state == DH_Modbus_Gripper::S_INIT_FINISHED)
                {
                    init_finished = true;
                    break;
                }
                usleep(50000);
            }

            if (!init_finished)
            {
                if (err)
                    *err = "DH gripper init did not finish in time";
                ready_.store(false, std::memory_order_relaxed);
                return false;
            }
        }

        ready_.store(true, std::memory_order_relaxed);
        return true;
    }

    void AcceptCommand(const nlohmann::ordered_json& cmd,
                       const char* context,
                       bool debug_log) override
    {
        (void)debug_log;
        if (!gripper_ || !ready_.load(std::memory_order_relaxed))
        {
            log_with_context(context, "gripper_DH not ready");
            return;
        }

        bool handled = false;
        if (cmd.contains("init") && cmd["init"].is_boolean() && cmd["init"].get<bool>())
        {
            if (!gripper_->Initialization())
                log_with_context(context, "gripper_DH init failed");
            handled = true;
        }
        if (cmd.contains("position") && cmd["position"].is_number())
        {
            int pos = static_cast<int>(cmd["position"].get<double>() * 1000.0);
            if (!gripper_->SetTargetPosition(pos))
                log_with_context(context, "gripper_DH set position failed");
            handled = true;
        }
        if (cmd.contains("force") && cmd["force"].is_number())
        {
            int force = static_cast<int>(cmd["force"].get<double>());
            if (!gripper_->SetTargetForce(force))
                log_with_context(context, "gripper_DH set force failed");
            handled = true;
        }
        if (cmd.contains("speed") && cmd["speed"].is_number())
        {
            int speed = static_cast<int>(cmd["speed"].get<double>());
            if (!gripper_->SetTargetSpeed(speed))
                log_with_context(context, "gripper_DH set speed failed");
            handled = true;
        }

        if (!handled)
        {
            log_with_context(context, "gripper_DH missing/invalid fields");
            return;
        }

        int curpos_raw = 0;
        if (gripper_->GetCurrentPosition(curpos_raw))
        {
            current_position_.store(static_cast<double>(curpos_raw) / 1000.0,
                                    std::memory_order_relaxed);
        }
        else
        {
            current_position_.store(-1.0, std::memory_order_relaxed);
            log_with_context(context, "gripper_DH read position failed");
        }
    }

    EndEffectorSlotState GetState() const override
    {
        EndEffectorSlotState state;
        state.slot_index = slot_index_;
        state.type = "gripper_DH";
        state.ready = ready_.load(std::memory_order_relaxed);
        state.current_position = current_position_.load(std::memory_order_relaxed);
        return state;
    }

private:
    int slot_index_;
    std::string port_;
    int baud_;
    int id_;
    std::unique_ptr<DH_Modbus_Gripper> gripper_;
    std::atomic<bool> ready_{false};
    std::atomic<double> current_position_{-1.0};
};

class SerialEndEffectorSlot : public IEndEffectorSlot
{
public:
    SerialEndEffectorSlot(int slot_index,
                          const std::string& port,
                          int baud,
                          const SerialOptions& serial_options)
        : slot_index_(slot_index),
          port_(port),
          baud_(baud),
          serial_options_(serial_options)
    {
    }

    bool Init(std::string* err) override
    {
        device_ = std::unique_ptr<EndEffectorDevice>(
            new EndEffectorDevice(port_, baud_, serial_options_));
        if (!device_->Open())
        {
            if (err)
                *err = "endeffector serial init failed";
            device_.reset();
            ready_.store(false, std::memory_order_relaxed);
            return false;
        }

        ready_.store(true, std::memory_order_relaxed);
        return true;
    }

    void AcceptCommand(const nlohmann::ordered_json& cmd,
                       const char* context,
                       bool debug_log) override
    {
        (void)debug_log;
        if (!device_ || !ready_.load(std::memory_order_relaxed))
        {
            log_with_context(context, "endeffector not ready");
            return;
        }

        if (!cmd.contains("mode") || !cmd["mode"].is_number_integer())
        {
            log_with_context(context, "EndEffector missing/invalid mode");
            return;
        }

        const int mode = cmd["mode"].get<int>();
        if (mode == 0 && cmd.contains("position") && cmd["position"].is_number())
        {
            device_->HandlePosition(cmd["position"].get<double>());
            return;
        }
        if (mode == 1 && cmd.contains("preset") && cmd["preset"].is_number_integer())
        {
            device_->HandlePreset(cmd["preset"].get<int>());
            return;
        }

        log_with_context(context, "EndEffector missing/invalid fields");
    }

    EndEffectorSlotState GetState() const override
    {
        EndEffectorSlotState state;
        state.slot_index = slot_index_;
        state.type = "endeffector";
        state.ready = ready_.load(std::memory_order_relaxed);
        state.current_position = -1.0;
        return state;
    }

private:
    int slot_index_;
    std::string port_;
    int baud_;
    SerialOptions serial_options_;
    std::unique_ptr<EndEffectorDevice> device_;
    std::atomic<bool> ready_{false};
};

static SerialOptions::SerialProfile parse_serial_profile(const nlohmann::json& cfg,
                                                         std::string* err)
{
    if (!cfg.contains("serial_profile"))
        return SerialOptions::kProfileJEServerLegacy;
    if (!cfg["serial_profile"].is_string())
    {
        if (err)
            *err = "invalid serial_profile, expected string";
        return SerialOptions::kProfileJEServerLegacy;
    }
    const std::string profile = cfg["serial_profile"].get<std::string>();
    if (profile == "JEServerLegacy")
        return SerialOptions::kProfileJEServerLegacy;
    if (profile == "DhDevice")
        return SerialOptions::kProfileDhDevice;

    if (err)
        *err = "unsupported serial_profile: " + profile;
    return SerialOptions::kProfileJEServerLegacy;
}

} // namespace

std::unique_ptr<IEndEffectorSlot> BuildEndEffectorSlotFromConfig(
    const nlohmann::json& slot_cfg,
    int slot_index,
    std::string* err)
{
    if (!slot_cfg.is_object())
    {
        if (err)
            *err = "slot config must be object";
        return std::unique_ptr<IEndEffectorSlot>();
    }

    std::string type;
    if (!get_required_string(slot_cfg, "type", &type, err))
        return std::unique_ptr<IEndEffectorSlot>();

    if (type == "none")
    {
        return std::unique_ptr<IEndEffectorSlot>(new NoneEndEffectorSlot(slot_index));
    }

    if (type == "gripper_DH")
    {
        std::string port;
        int baud = 0;
        int id = 0;
        if (!get_required_string(slot_cfg, "port", &port, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_required_int(slot_cfg, "baud", &baud, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_required_int(slot_cfg, "id", &id, err))
            return std::unique_ptr<IEndEffectorSlot>();

        return std::unique_ptr<IEndEffectorSlot>(
            new DhGripperEndEffectorSlot(slot_index, port, baud, id));
    }

    if (type == "endeffector")
    {
        std::string port;
        int baud = 0;
        if (!get_required_string(slot_cfg, "port", &port, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_required_int(slot_cfg, "baud", &baud, err))
            return std::unique_ptr<IEndEffectorSlot>();

        std::string profile_err;
        SerialOptions::SerialProfile profile = parse_serial_profile(slot_cfg, &profile_err);
        if (!profile_err.empty())
        {
            if (err)
                *err = profile_err;
            return std::unique_ptr<IEndEffectorSlot>();
        }

        int vmin = 0;
        int vtime = 5;
        if (slot_cfg.contains("read_timeout_ms"))
        {
            if (!slot_cfg["read_timeout_ms"].is_number_integer())
            {
                if (err)
                    *err = "invalid read_timeout_ms, expected integer";
                return std::unique_ptr<IEndEffectorSlot>();
            }
            vtime = slot_cfg["read_timeout_ms"].get<int>();
        }
        if (slot_cfg.contains("write_timeout_ms"))
        {
            if (!slot_cfg["write_timeout_ms"].is_number_integer())
            {
                if (err)
                    *err = "invalid write_timeout_ms, expected integer";
                return std::unique_ptr<IEndEffectorSlot>();
            }
            vmin = slot_cfg["write_timeout_ms"].get<int>();
        }

        SerialOptions options(vmin, vtime, profile);
        return std::unique_ptr<IEndEffectorSlot>(
            new SerialEndEffectorSlot(slot_index, port, baud, options));
    }

    if (err)
        *err = "unsupported type: " + type;
    return std::unique_ptr<IEndEffectorSlot>();
}
