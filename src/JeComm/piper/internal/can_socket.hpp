#pragma once

#include <string>

#include <linux/can.h>
#include <cstdint>

namespace piper {
namespace gripper {
namespace internal {

enum class CanSocketError {
  Ok,
  NotOpen,
  OpenFailed,
  BindFailed,
  InterfaceNotFound,
  InterfaceDown,
  SendFailed,
  RecvFailed,
  Timeout,
};

class CanSocket {
 public:
  CanSocket();
  ~CanSocket();

  CanSocket(const CanSocket&) = delete;
  CanSocket& operator=(const CanSocket&) = delete;

  CanSocketError Open(const std::string& ifname, int recv_timeout_ms, canid_t filter_id);
  CanSocketError Close();

  CanSocketError SendFrame(canid_t can_id, const uint8_t* data, size_t len);
  CanSocketError RecvFrame(can_frame* out);

  bool IsOpen() const;

 private:
  int fd_;
};

}  // namespace internal
}  // namespace gripper
}  // namespace piper
