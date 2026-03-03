#include "protocol_codec.hpp"

#include <array>

namespace piper {
namespace gripper {
namespace internal {

namespace {

inline uint8_t Byte0FromInt32(int32_t value) { return static_cast<uint8_t>((value >> 24) & 0xFF); }
inline uint8_t Byte1FromInt32(int32_t value) { return static_cast<uint8_t>((value >> 16) & 0xFF); }
inline uint8_t Byte2FromInt32(int32_t value) { return static_cast<uint8_t>((value >> 8) & 0xFF); }
inline uint8_t Byte3FromInt32(int32_t value) { return static_cast<uint8_t>(value & 0xFF); }

inline int32_t Int32FromBigEndian(const uint8_t* data) {
  const uint32_t raw = (static_cast<uint32_t>(data[0]) << 24) |
                       (static_cast<uint32_t>(data[1]) << 16) |
                       (static_cast<uint32_t>(data[2]) << 8) |
                       static_cast<uint32_t>(data[3]);
  return static_cast<int32_t>(raw);
}

inline int16_t Int16FromBigEndian(const uint8_t* data) {
  const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) |
                       static_cast<uint16_t>(data[1]);
  return static_cast<int16_t>(raw);
}

}  // namespace

std::array<uint8_t, 8> ProtocolCodec::EncodeGripperCtrl(const GripperCommand& cmd) {
  std::array<uint8_t, 8> out{};
  out[0] = Byte0FromInt32(cmd.angle_0p001mm);
  out[1] = Byte1FromInt32(cmd.angle_0p001mm);
  out[2] = Byte2FromInt32(cmd.angle_0p001mm);
  out[3] = Byte3FromInt32(cmd.angle_0p001mm);
  out[4] = static_cast<uint8_t>((cmd.effort_0p001Nm >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>(cmd.effort_0p001Nm & 0xFF);
  out[6] = static_cast<uint8_t>(cmd.status);
  out[7] = static_cast<uint8_t>(cmd.set_zero);
  return out;
}

bool ProtocolCodec::DecodeGripperFeedback(const can_frame& frame, DecodedGripperFeedback* out) {
  if (out == nullptr) {
    return false;
  }
  if (frame.can_id != kCanIdGripperFeedback || frame.can_dlc < 7) {
    return false;
  }

  out->angle_0p001mm = Int32FromBigEndian(frame.data);
  out->effort_0p001Nm = Int16FromBigEndian(&frame.data[4]);
  out->status_code = frame.data[6];
  return true;
}

}  // namespace internal
}  // namespace gripper
}  // namespace piper
