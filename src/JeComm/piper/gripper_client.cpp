#include "piper/gripper/gripper_client.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <linux/can.h>

#include "internal/can_socket.hpp"
#include "internal/fps_counter.hpp"
#include "internal/protocol_codec.hpp"

namespace piper {
namespace gripper {

class GripperClient::Impl {
 public:
  explicit Impl(Config cfg_in) : cfg(std::move(cfg_in)) {
    fps_counter.AddVariable("CanMonitor");
    fps_counter.AddVariable("ArmGripper");
  }

  Config cfg;
  mutable std::mutex cfg_mtx;

  mutable std::mutex state_mtx;
  GripperState state;

  mutable std::mutex ok_window_mtx;
  std::deque<double> can_fps_window;

  std::atomic<bool> running{false};
  std::atomic<bool> connected{false};
  std::atomic<bool> is_ok{true};

  internal::CanSocket can_socket;
  internal::FpsCounter fps_counter;

  std::thread read_thread;
  std::thread monitor_thread;

  int32_t ApplySdkLimit(int32_t value) const {
    std::lock_guard<std::mutex> lock(cfg_mtx);
    if (!cfg.enable_sdk_gripper_limit) {
      return value;
    }
    const int64_t min_raw = static_cast<int64_t>(std::llround(cfg.sdk_gripper_range.min_m * 1000.0 * 1000.0));
    const int64_t max_raw = static_cast<int64_t>(std::llround(cfg.sdk_gripper_range.max_m * 1000.0 * 1000.0));
    const int64_t clamped = std::max(min_raw, std::min(static_cast<int64_t>(value), max_raw));
    return static_cast<int32_t>(clamped);
  }

  bool IsAbnormalFilterEnabled() const {
    std::lock_guard<std::mutex> lock(cfg_mtx);
    return cfg.enable_abnormal_filter;
  }

  int MonitorPeriodMs() const {
    std::lock_guard<std::mutex> lock(cfg_mtx);
    return cfg.monitor_period_ms;
  }

  void FillFocStatus(uint8_t status_code, FocStatus* foc) const {
    if (foc == nullptr) {
      return;
    }
    foc->voltage_too_low = (status_code & (1U << 0)) != 0;
    foc->motor_overheating = (status_code & (1U << 1)) != 0;
    foc->driver_overcurrent = (status_code & (1U << 2)) != 0;
    foc->driver_overheating = (status_code & (1U << 3)) != 0;
    foc->sensor_status = (status_code & (1U << 4)) != 0;
    foc->driver_error_status = (status_code & (1U << 5)) != 0;
    foc->driver_enable_status = (status_code & (1U << 6)) != 0;
    foc->homing_status = (status_code & (1U << 7)) != 0;
  }

  static double NowSec() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
  }

  void ReadCanLoop() {
    while (running.load()) {
      can_frame frame{};
      const auto recv_ret = can_socket.RecvFrame(&frame);
      if (recv_ret == internal::CanSocketError::Timeout) {
        continue;
      }
      if (recv_ret == internal::CanSocketError::InterfaceDown) {
        is_ok.store(false);
        continue;
      }
      if (recv_ret != internal::CanSocketError::Ok) {
        if (!running.load()) {
          break;
        }
        is_ok.store(false);
        continue;
      }

      internal::DecodedGripperFeedback decoded{};
      if (!internal::ProtocolCodec::DecodeGripperFeedback(frame, &decoded)) {
        continue;
      }

      fps_counter.Increment("CanMonitor");

      int32_t angle = ApplySdkLimit(decoded.angle_0p001mm);
      if (IsAbnormalFilterEnabled() && std::abs(angle) > 150000) {
        continue;
      }

      fps_counter.Increment("ArmGripper");

      std::lock_guard<std::mutex> lock(state_mtx);
      state.timestamp_sec = NowSec();
      state.angle_0p001mm = angle;
      state.effort_0p001Nm = decoded.effort_0p001Nm;
      state.status_code = decoded.status_code;
      FillFocStatus(decoded.status_code, &state.foc);
    }
  }

  void MonitorLoop() {
    while (running.load()) {
      const double fps = fps_counter.GetFps("CanMonitor");
      {
        std::lock_guard<std::mutex> lock(ok_window_mtx);
        const size_t window_limit = std::max<size_t>(1, cfg.is_ok_window);
        if (can_fps_window.size() >= window_limit) {
          can_fps_window.pop_front();
        }
        can_fps_window.push_back(fps);

        bool all_zero = can_fps_window.size() == window_limit;
        if (all_zero) {
          for (double v : can_fps_window) {
            if (v != 0.0) {
              all_zero = false;
              break;
            }
          }
        }
        is_ok.store(!all_zero);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(MonitorPeriodMs()));
    }
  }
};

namespace {

ErrorCode MapCanOpenError(internal::CanSocketError err) {
  switch (err) {
    case internal::CanSocketError::Ok:
      return ErrorCode::Ok;
    case internal::CanSocketError::InterfaceDown:
      return ErrorCode::CanInterfaceDown;
    case internal::CanSocketError::BindFailed:
      return ErrorCode::CanBindFailed;
    case internal::CanSocketError::OpenFailed:
    case internal::CanSocketError::InterfaceNotFound:
      return ErrorCode::CanOpenFailed;
    default:
      return ErrorCode::InternalError;
  }
}

ErrorCode MapCanSendError(internal::CanSocketError err) {
  switch (err) {
    case internal::CanSocketError::Ok:
      return ErrorCode::Ok;
    case internal::CanSocketError::InterfaceDown:
      return ErrorCode::CanInterfaceDown;
    default:
      return ErrorCode::CanSendFailed;
  }
}

bool IsValidStatus(StatusCode status) {
  switch (status) {
    case StatusCode::Disable:
    case StatusCode::Enable:
    case StatusCode::DisableAndClearError:
    case StatusCode::EnableAndClearError:
      return true;
    default:
      return false;
  }
}

bool IsValidSetZero(SetZero set_zero) {
  switch (set_zero) {
    case SetZero::Invalid:
    case SetZero::SetZero:
      return true;
    default:
      return false;
  }
}

}  // namespace

GripperClient::GripperClient(Config cfg) : impl_(new Impl(std::move(cfg))) {}

GripperClient::~GripperClient() {
  (void)Stop();
  delete impl_;
  impl_ = nullptr;
}

VoidResult GripperClient::Start() {
  if (impl_->connected.load()) {
    return {ErrorCode::Ok, "already started"};
  }

  Config cfg_snapshot;
  {
    std::lock_guard<std::mutex> lock(impl_->cfg_mtx);
    cfg_snapshot = impl_->cfg;
  }

  const auto open_ret = impl_->can_socket.Open(cfg_snapshot.can_ifname, cfg_snapshot.recv_timeout_ms,
                                               internal::kCanIdGripperFeedback);
  if (open_ret != internal::CanSocketError::Ok) {
    return {MapCanOpenError(open_ret), "failed to open can socket"};
  }

  impl_->is_ok.store(true);
  {
    std::lock_guard<std::mutex> lock(impl_->ok_window_mtx);
    impl_->can_fps_window.clear();
  }

  impl_->fps_counter.Start(cfg_snapshot.fps_period_ms);
  impl_->running.store(true);
  try {
    impl_->read_thread = std::thread(&Impl::ReadCanLoop, impl_);
    impl_->monitor_thread = std::thread(&Impl::MonitorLoop, impl_);
  } catch (...) {
    impl_->running.store(false);
    impl_->fps_counter.Stop();
    (void)impl_->can_socket.Close();
    return {ErrorCode::InternalError, "failed to start worker threads"};
  }

  impl_->connected.store(true);
  return {ErrorCode::Ok, "started"};
}

VoidResult GripperClient::Stop() {
  impl_->running.store(false);

  if (impl_->read_thread.joinable()) {
    impl_->read_thread.join();
  }
  if (impl_->monitor_thread.joinable()) {
    impl_->monitor_thread.join();
  }

  impl_->fps_counter.Stop();
  (void)impl_->can_socket.Close();
  impl_->connected.store(false);
  return {ErrorCode::Ok, "stopped"};
}

VoidResult GripperClient::GripperCtrl(const GripperCommand& cmd) {
  if (cmd.effort_0p001Nm > 5000U) {
    return {ErrorCode::InvalidArgument, "effort_0p001Nm out of range 0..5000"};
  }
  if (!IsValidStatus(cmd.status)) {
    return {ErrorCode::InvalidArgument, "invalid status"};
  }
  if (!IsValidSetZero(cmd.set_zero)) {
    return {ErrorCode::InvalidArgument, "invalid set_zero"};
  }
  if (!impl_->connected.load()) {
    return {ErrorCode::NotStarted, "client not started"};
  }

  GripperCommand tx = cmd;
  tx.angle_0p001mm = impl_->ApplySdkLimit(tx.angle_0p001mm);

  const auto payload = internal::ProtocolCodec::EncodeGripperCtrl(tx);
  const auto send_ret = impl_->can_socket.SendFrame(internal::kCanIdGripperCtrl, payload.data(), payload.size());
  if (send_ret != internal::CanSocketError::Ok) {
    return {MapCanSendError(send_ret), "failed to send can frame"};
  }
  return {ErrorCode::Ok, "sent"};
}

Result<GripperState> GripperClient::GetArmGripperMsgs() const {
  Result<GripperState> result;
  if (!impl_->connected.load()) {
    result.code = ErrorCode::NotStarted;
    result.message = "client not started";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->state_mtx);
    result.value = impl_->state;
  }
  result.value.hz = impl_->fps_counter.GetFps("ArmGripper");
  result.code = ErrorCode::Ok;
  result.message = "ok";
  return result;
}

bool GripperClient::IsOk() const { return impl_->is_ok.load(); }

GripperRange GripperClient::GetSDKGripperRangeParam() const {
  std::lock_guard<std::mutex> lock(impl_->cfg_mtx);
  return impl_->cfg.sdk_gripper_range;
}

VoidResult GripperClient::SetSDKGripperRangeParam(double min_m, double max_m) {
  if (max_m < min_m) {
    return {ErrorCode::InvalidArgument, "max_m should be >= min_m"};
  }
  std::lock_guard<std::mutex> lock(impl_->cfg_mtx);
  impl_->cfg.sdk_gripper_range.min_m = min_m;
  impl_->cfg.sdk_gripper_range.max_m = max_m;
  return {ErrorCode::Ok, "updated"};
}

void GripperClient::EnableSDKGripperLimit(bool enable) {
  std::lock_guard<std::mutex> lock(impl_->cfg_mtx);
  impl_->cfg.enable_sdk_gripper_limit = enable;
}

void GripperClient::EnableFilterAbnormalData(bool enable) {
  std::lock_guard<std::mutex> lock(impl_->cfg_mtx);
  impl_->cfg.enable_abnormal_filter = enable;
}

bool GripperClient::IsConnected() const { return impl_->connected.load(); }

}  // namespace gripper
}  // namespace piper
