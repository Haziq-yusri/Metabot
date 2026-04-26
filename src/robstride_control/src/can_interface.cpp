// Copyright 2024 Metabot contributors
// SPDX-License-Identifier: MIT

#include "robstride_control/can_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

namespace robstride_control
{

CanInterface::CanInterface()
: socket_fd_(-1) {}

CanInterface::~CanInterface()
{
  close();
}

bool CanInterface::init(const std::string & interface)
{
  close();
  interface_name_ = interface;

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[robstride] socket() failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);

  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[robstride] ioctl(" << interface << ") failed: "
              << std::strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "[robstride] bind() failed: " << std::strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

void CanInterface::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanInterface::send_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (socket_fd_ < 0) {
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id | CAN_EFF_FLAG;
  frame.can_dlc = dlc;

  if (data != nullptr && dlc > 0) {
    std::memcpy(frame.data, data, dlc);
  }

  ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

bool CanInterface::read_frame(struct can_frame * frame, int timeout_ms)
{
  if (socket_fd_ < 0 || frame == nullptr) {
    return false;
  }

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  fd_set rdfs;
  FD_ZERO(&rdfs);
  FD_SET(socket_fd_, &rdfs);

  int ret = ::select(socket_fd_ + 1, &rdfs, nullptr, nullptr, &tv);
  if (ret <= 0) {
    return false;
  }

  ssize_t n = ::read(socket_fd_, frame, sizeof(*frame));
  return n == static_cast<ssize_t>(sizeof(*frame));
}

}  // namespace robstride_control
