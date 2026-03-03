#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace piper {
namespace gripper {

enum class StatusCode : uint8_t {
  Disable = 0x00,
  Enable = 0x01,
  DisableAndClearError = 0x02,
  EnableAndClearError = 0x03,
};

enum class SetZero : uint8_t {
  Invalid = 0x00,
  SetZero = 0xAE,
};

enum class ErrorCode {
  Ok,
  NotStarted,
  InvalidArgument,
  CanOpenFailed,
  CanBindFailed,
  CanSendFailed,
  CanRecvFailed,
  CanInterfaceDown,
  Timeout,
  InternalError,
};

struct GripperCommand {
  int32_t angle_0p001mm = 0;
  uint16_t effort_0p001Nm = 0;
  StatusCode status = StatusCode::Disable;
  SetZero set_zero = SetZero::Invalid;
};

struct FocStatus {
  bool voltage_too_low = false;
  bool motor_overheating = false;
  bool driver_overcurrent = false;
  bool driver_overheating = false;
  bool sensor_status = false;
  bool driver_error_status = false;
  bool driver_enable_status = false;
  bool homing_status = false;
};

struct GripperState {
  double timestamp_sec = 0.0;
  double hz = 0.0;
  int32_t angle_0p001mm = 0;
  int16_t effort_0p001Nm = 0;
  uint8_t status_code = 0;
  FocStatus foc;
};

struct GripperRange {
  double min_m = 0.0;
  double max_m = 0.07;
};

struct Config {
  std::string can_ifname = "can0";
  int bitrate = 1000000;
  bool auto_init = true;
  bool enable_sdk_gripper_limit = false;
  bool enable_abnormal_filter = true;
  GripperRange sdk_gripper_range{};
  int recv_timeout_ms = 1000;
  int monitor_period_ms = 50;
  int fps_period_ms = 100;
  size_t is_ok_window = 5;
};

template <typename T>
struct Result {
  Result() : code(ErrorCode::Ok), message(), value() {}
  Result(ErrorCode code_in, const std::string& message_in)
      : code(code_in), message(message_in), value() {}
  Result(ErrorCode code_in, const std::string& message_in, const T& value_in)
      : code(code_in), message(message_in), value(value_in) {}

  ErrorCode code = ErrorCode::Ok;
  std::string message;
  T value{};
};

struct VoidResult {
  VoidResult() : code(ErrorCode::Ok), message() {}
  VoidResult(ErrorCode code_in, const std::string& message_in)
      : code(code_in), message(message_in) {}

  ErrorCode code = ErrorCode::Ok;
  std::string message;
};

class GripperClient {
 public:
  explicit GripperClient(Config cfg = {});
  ~GripperClient();

  GripperClient(const GripperClient&) = delete;
  GripperClient& operator=(const GripperClient&) = delete;

  VoidResult Start();
  VoidResult Stop();

  VoidResult GripperCtrl(const GripperCommand& cmd);

  Result<GripperState> GetArmGripperMsgs() const;
  bool IsOk() const;

  GripperRange GetSDKGripperRangeParam() const;
  VoidResult SetSDKGripperRangeParam(double min_m, double max_m);

  void EnableSDKGripperLimit(bool enable);
  void EnableFilterAbnormalData(bool enable);

  bool IsConnected() const;

 private:
  class Impl;
  Impl* impl_;
};

}  // namespace gripper
}  // namespace piper
