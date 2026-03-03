#include "can_socket.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace piper {
namespace gripper {
namespace internal {

CanSocket::CanSocket() : fd_(-1) {}

CanSocket::~CanSocket() { Close(); }

CanSocketError CanSocket::Open(const std::string& ifname, int recv_timeout_ms, canid_t filter_id) {
  if (fd_ >= 0) {
    return CanSocketError::Ok;
  }

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return CanSocketError::OpenFailed;
  }

  ifreq ifr{};
  if (ifname.size() >= IFNAMSIZ) {
    Close();
    return CanSocketError::InterfaceNotFound;
  }
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    Close();
    return CanSocketError::InterfaceNotFound;
  }

  ifreq flags_ifr{};
  std::strncpy(flags_ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFFLAGS, &flags_ifr) < 0) {
    Close();
    return CanSocketError::InterfaceNotFound;
  }
  if ((flags_ifr.ifr_flags & IFF_UP) == 0) {
    Close();
    return CanSocketError::InterfaceDown;
  }

  can_filter rfilter{};
  rfilter.can_id = filter_id;
  rfilter.can_mask = CAN_SFF_MASK;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
    Close();
    return CanSocketError::OpenFailed;
  }

  timeval tv{};
  tv.tv_sec = recv_timeout_ms / 1000;
  tv.tv_usec = (recv_timeout_ms % 1000) * 1000;
  if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    Close();
    return CanSocketError::OpenFailed;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    Close();
    return CanSocketError::BindFailed;
  }

  return CanSocketError::Ok;
}

CanSocketError CanSocket::Close() {
  if (fd_ < 0) {
    return CanSocketError::NotOpen;
  }
  ::close(fd_);
  fd_ = -1;
  return CanSocketError::Ok;
}

CanSocketError CanSocket::SendFrame(canid_t can_id, const uint8_t* data, size_t len) {
  if (fd_ < 0) {
    return CanSocketError::NotOpen;
  }
  if (len > CAN_MAX_DLEN) {
    return CanSocketError::SendFailed;
  }

  can_frame frame{};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(len);
  for (size_t i = 0; i < len; ++i) {
    frame.data[i] = data[i];
  }

  const ssize_t nbytes = ::write(fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
    if (errno == ENETDOWN || errno == ENODEV) {
      return CanSocketError::InterfaceDown;
    }
    return CanSocketError::SendFailed;
  }
  return CanSocketError::Ok;
}

CanSocketError CanSocket::RecvFrame(can_frame* out) {
  if (out == nullptr) {
    return CanSocketError::RecvFailed;
  }
  if (fd_ < 0) {
    return CanSocketError::NotOpen;
  }

  can_frame frame{};
  const ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return CanSocketError::Timeout;
    }
    if (errno == ENETDOWN || errno == ENODEV) {
      return CanSocketError::InterfaceDown;
    }
    return CanSocketError::RecvFailed;
  }
  if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
    return CanSocketError::RecvFailed;
  }

  *out = frame;
  return CanSocketError::Ok;
}

bool CanSocket::IsOpen() const { return fd_ >= 0; }

}  // namespace internal
}  // namespace gripper
}  // namespace piper
