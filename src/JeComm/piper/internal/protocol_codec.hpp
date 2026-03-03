#pragma once

#include <array>
#include <cstdint>

#include <linux/can.h>

#include "piper/gripper/gripper_client.hpp"

namespace piper {
namespace gripper {
namespace internal {

constexpr canid_t kCanIdGripperCtrl = 0x159;
constexpr canid_t kCanIdGripperFeedback = 0x2A8;

struct DecodedGripperFeedback {
  int32_t angle_0p001mm = 0;
  int16_t effort_0p001Nm = 0;
  uint8_t status_code = 0;
};

class ProtocolCodec {
 public:
  static std::array<uint8_t, 8> EncodeGripperCtrl(const GripperCommand& cmd);
  static bool DecodeGripperFeedback(const can_frame& frame, DecodedGripperFeedback* out);
};

}  // namespace internal
}  // namespace gripper
}  // namespace piper
