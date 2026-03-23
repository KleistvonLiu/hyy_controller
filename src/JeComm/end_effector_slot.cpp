#include "end_effector_slot.h"

#include "dh_modbus_gripper.h"
#include "dh_transport.h"
#include "end_effector_device.h"
#include "piper/gripper/gripper_client.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
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

static bool get_optional_int(const nlohmann::json& cfg,
                             const char* key,
                             int* out,
                             std::string* err)
{
    if (!cfg.contains(key))
        return true;
    if (!cfg[key].is_number_integer())
    {
        if (err)
            *err = std::string("invalid integer field '") + key + "'";
        return false;
    }
    *out = cfg[key].get<int>();
    return true;
}

static bool get_optional_double(const nlohmann::json& cfg,
                                const char* key,
                                double* out,
                                std::string* err)
{
    if (!cfg.contains(key))
        return true;
    if (!cfg[key].is_number())
    {
        if (err)
            *err = std::string("invalid number field '") + key + "'";
        return false;
    }
    *out = cfg[key].get<double>();
    return true;
}

static bool get_optional_bool(const nlohmann::json& cfg,
                              const char* key,
                              bool* out,
                              std::string* err)
{
    if (!cfg.contains(key))
        return true;
    if (!cfg[key].is_boolean())
    {
        if (err)
            *err = std::string("invalid boolean field '") + key + "'";
        return false;
    }
    *out = cfg[key].get<bool>();
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
            {
                log_with_context(context, "gripper_DH set position failed");
            }
            else
            {
                current_position_.store(static_cast<double>(pos) / 1000.0,
                                        std::memory_order_relaxed);
            }
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

class PiperGripperEndEffectorSlot : public IEndEffectorSlot
{
public:
    PiperGripperEndEffectorSlot(int slot_index,
                                const piper::gripper::Config& cfg)
        : slot_index_(slot_index),
          cfg_(cfg)
    {
    }

    ~PiperGripperEndEffectorSlot() override
    {
        if (client_)
            client_->Stop();
    }

    bool Init(std::string* err) override
    {
        client_.reset(new piper::gripper::GripperClient(cfg_));
        piper::gripper::VoidResult ret = client_->Start();
        if (ret.code != piper::gripper::ErrorCode::Ok)
        {
            if (err)
                *err = "gripper_piper start failed: " + ret.message;
            client_.reset();
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
        if (!client_ || !ready_.load(std::memory_order_relaxed))
        {
            log_with_context(context, "gripper_piper not ready");
            return;
        }

        piper::gripper::GripperCommand out;
        out.angle_0p001mm = last_angle_raw_.load(std::memory_order_relaxed);
        out.effort_0p001Nm = static_cast<uint16_t>(kPiperDefaultEffortRaw);
        out.status = piper::gripper::StatusCode::Enable;
        out.set_zero = piper::gripper::SetZero::Invalid;

        bool has_control_field = false;
        bool status_explicit = false;

        if (cmd.contains("position") && cmd["position"].is_number())
        {
            const double normalized = clamp_double(cmd["position"].get<double>(), 0.0, 1.0);
            // out.angle_0p001mm = static_cast<int32_t>(std::lround((1-normalized) * static_cast<double>(kPiperAngleMaxRaw)));
            out.angle_0p001mm = static_cast<int32_t>(std::lround(normalized * static_cast<double>(kPiperAngleMaxRaw)));
            has_control_field = true;
        }

        if (cmd.contains("force") && cmd["force"].is_number())
        {
            const double percent = clamp_double(cmd["force"].get<double>(), 0.0, 100.0);
            out.effort_0p001Nm = static_cast<uint16_t>(std::lround(percent * 50.0));
            has_control_field = true;
        }

        if (cmd.contains("torque") && cmd["torque"].is_number())
        {
            const double torque_nm = clamp_double(
                cmd["torque"].get<double>(),
                0.0,
                static_cast<double>(kPiperEffortMaxRaw) * 0.001);
            out.effort_0p001Nm = static_cast<uint16_t>(std::lround(torque_nm * 1000.0));
            has_control_field = true;
        }

        if (cmd.contains("angle_0p001mm") && cmd["angle_0p001mm"].is_number())
        {
            const int raw = static_cast<int>(std::lround(cmd["angle_0p001mm"].get<double>()));
            out.angle_0p001mm = static_cast<int32_t>(clamp_int(raw, kPiperAngleMinRaw, kPiperAngleMaxRaw));
            has_control_field = true;
        }

        if (cmd.contains("effort_0p001Nm") && cmd["effort_0p001Nm"].is_number())
        {
            const int raw = static_cast<int>(std::lround(cmd["effort_0p001Nm"].get<double>()));
            out.effort_0p001Nm = static_cast<uint16_t>(clamp_int(raw, kPiperEffortMinRaw, kPiperEffortMaxRaw));
            has_control_field = true;
        }

        if (cmd.contains("command") || cmd.contains("gripper") || cmd.contains("action"))
        {
            const nlohmann::ordered_json* action_node = nullptr;
            if (cmd.contains("command"))
                action_node = &cmd["command"];
            else if (cmd.contains("gripper"))
                action_node = &cmd["gripper"];
            else
                action_node = &cmd["action"];

            if (!action_node->is_string())
            {
                log_with_context(context, "gripper_piper invalid open/close command");
                return;
            }

            std::string action = action_node->get<std::string>();
            std::transform(action.begin(),
                           action.end(),
                           action.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

            if (action == "open")
            {
                out.angle_0p001mm = kPiperAngleMaxRaw;
            }
            else if (action == "close")
            {
                out.angle_0p001mm = kPiperAngleMinRaw;
            }
            else
            {
                log_with_context(context, "gripper_piper unsupported open/close command");
                return;
            }
            has_control_field = true;
        }

        if (cmd.contains("status"))
        {
            if (!cmd["status"].is_number_integer())
            {
                log_with_context(context, "gripper_piper invalid status");
                return;
            }
            piper::gripper::StatusCode parsed_status = piper::gripper::StatusCode::Enable;
            if (!parse_status(cmd["status"].get<int>(), &parsed_status))
            {
                log_with_context(context, "gripper_piper unsupported status value");
                return;
            }
            out.status = parsed_status;
            status_explicit = true;
            has_control_field = true;
        }

        if (cmd.contains("set_zero"))
        {
            if (!cmd["set_zero"].is_number_integer())
            {
                log_with_context(context, "gripper_piper invalid set_zero");
                return;
            }
            piper::gripper::SetZero parsed_set_zero = piper::gripper::SetZero::Invalid;
            if (!parse_set_zero(cmd["set_zero"].get<int>(), &parsed_set_zero))
            {
                log_with_context(context, "gripper_piper unsupported set_zero value");
                return;
            }
            out.set_zero = parsed_set_zero;
            has_control_field = true;
        }

        const bool init_request =
            cmd.contains("init") && cmd["init"].is_boolean() && cmd["init"].get<bool>();
        if (init_request && !status_explicit)
        {
            out.status = piper::gripper::StatusCode::EnableAndClearError;
            has_control_field = true;
        }

        if (!has_control_field)
        {
            log_with_context(context, "gripper_piper missing/invalid fields");
            return;
        }

        const piper::gripper::VoidResult ret = client_->GripperCtrl(out);
        if (ret.code != piper::gripper::ErrorCode::Ok)
        {
            log_with_context(context, std::string("gripper_piper command failed: ") + ret.message);
            return;
        }

        last_angle_raw_.store(out.angle_0p001mm, std::memory_order_relaxed);
        last_effort_raw_.store(static_cast<int>(out.effort_0p001Nm), std::memory_order_relaxed);

        piper::gripper::Result<piper::gripper::GripperState> state = client_->GetArmGripperMsgs();
        if (state.code == piper::gripper::ErrorCode::Ok)
        {
            const double normalized = clamp_double(
                static_cast<double>(state.value.angle_0p001mm) / static_cast<double>(kPiperAngleMaxRaw),
                0.0,
                1.0);
            current_position_.store(normalized, std::memory_order_relaxed);
        }
        else
        {
            current_position_.store(-1.0, std::memory_order_relaxed);
            log_with_context(context, std::string("gripper_piper read state failed: ") + state.message);
        }
    }

    EndEffectorSlotState GetState() const override
    {
        EndEffectorSlotState state;
        state.slot_index = slot_index_;
        state.type = "gripper_piper";
        state.ready = ready_.load(std::memory_order_relaxed);
        state.current_position = current_position_.load(std::memory_order_relaxed);
        return state;
    }

private:
    static int clamp_int(int value, int min_value, int max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }

    static double clamp_double(double value, double min_value, double max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }

    static bool parse_status(int value, piper::gripper::StatusCode* out)
    {
        if (!out)
            return false;
        switch (value)
        {
            case 0:
                *out = piper::gripper::StatusCode::Disable;
                return true;
            case 1:
                *out = piper::gripper::StatusCode::Enable;
                return true;
            case 2:
                *out = piper::gripper::StatusCode::DisableAndClearError;
                return true;
            case 3:
                *out = piper::gripper::StatusCode::EnableAndClearError;
                return true;
            default:
                return false;
        }
    }

    static bool parse_set_zero(int value, piper::gripper::SetZero* out)
    {
        if (!out)
            return false;
        switch (value)
        {
            case 0:
                *out = piper::gripper::SetZero::Invalid;
                return true;
            case 0xAE:
                *out = piper::gripper::SetZero::SetZero;
                return true;
            default:
                return false;
        }
    }

    static const int kPiperAngleMinRaw = 0;
    static const int kPiperAngleMaxRaw = 70000;
    static const int kPiperEffortMinRaw = 0;
    static const int kPiperEffortMaxRaw = 5000;
    static const int kPiperDefaultEffortRaw = 1000;

    int slot_index_;
    piper::gripper::Config cfg_;
    std::unique_ptr<piper::gripper::GripperClient> client_;
    std::atomic<bool> ready_{false};
    std::atomic<double> current_position_{-1.0};
    std::atomic<int32_t> last_angle_raw_{kPiperAngleMaxRaw};
    std::atomic<int> last_effort_raw_{1000};
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

    if (type == "gripper_piper")
    {
        piper::gripper::Config cfg;
        if (!get_required_string(slot_cfg, "can_ifname", &cfg.can_ifname, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_int(slot_cfg, "bitrate", &cfg.bitrate, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_bool(slot_cfg, "auto_init", &cfg.auto_init, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_int(slot_cfg, "recv_timeout_ms", &cfg.recv_timeout_ms, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_int(slot_cfg, "monitor_period_ms", &cfg.monitor_period_ms, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_int(slot_cfg, "fps_period_ms", &cfg.fps_period_ms, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_bool(slot_cfg, "enable_sdk_gripper_limit", &cfg.enable_sdk_gripper_limit, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_bool(slot_cfg, "enable_abnormal_filter", &cfg.enable_abnormal_filter, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_double(slot_cfg, "sdk_range_min_m", &cfg.sdk_gripper_range.min_m, err))
            return std::unique_ptr<IEndEffectorSlot>();
        if (!get_optional_double(slot_cfg, "sdk_range_max_m", &cfg.sdk_gripper_range.max_m, err))
            return std::unique_ptr<IEndEffectorSlot>();

        if (slot_cfg.contains("is_ok_window"))
        {
            if (!slot_cfg["is_ok_window"].is_number_integer())
            {
                if (err)
                    *err = "invalid integer field 'is_ok_window'";
                return std::unique_ptr<IEndEffectorSlot>();
            }
            const int is_ok_window = slot_cfg["is_ok_window"].get<int>();
            if (is_ok_window <= 0)
            {
                if (err)
                    *err = "is_ok_window must be positive";
                return std::unique_ptr<IEndEffectorSlot>();
            }
            cfg.is_ok_window = static_cast<size_t>(is_ok_window);
        }

        if (cfg.sdk_gripper_range.max_m < cfg.sdk_gripper_range.min_m)
        {
            if (err)
                *err = "sdk_range_max_m must be >= sdk_range_min_m";
            return std::unique_ptr<IEndEffectorSlot>();
        }

        return std::unique_ptr<IEndEffectorSlot>(
            new PiperGripperEndEffectorSlot(slot_index, cfg));
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
