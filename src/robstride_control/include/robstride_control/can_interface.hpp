// Copyright 2024 Metabot contributors
//
// SPDX-License-Identifier: MIT
//
// SocketCAN wrapper, derived from the upstream
// Seeed-Projects/RobStride_Control project (cpp/include/can_interface.h)
// and adapted for ROS 2 Jazzy.

#ifndef ROBSTRIDE_CONTROL__CAN_INTERFACE_HPP_
#define ROBSTRIDE_CONTROL__CAN_INTERFACE_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <string>

namespace robstride_control
{

class CanInterface
{
public:
  CanInterface();
  ~CanInterface();

  CanInterface(const CanInterface &) = delete;
  CanInterface & operator=(const CanInterface &) = delete;

  /// Open and bind a SocketCAN interface (e.g. "can0").
  bool init(const std::string & interface = "can0");

  /// Close the underlying socket.
  void close();

  /// Send a CAN frame using the 29-bit extended ID.
  bool send_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

  /// Read a CAN frame, blocking up to ``timeout_ms`` milliseconds.
  bool read_frame(struct can_frame * frame, int timeout_ms = 100);

  bool is_ready() const {return socket_fd_ >= 0;}

private:
  int socket_fd_;
  std::string interface_name_;
};

}  // namespace robstride_control

#endif  // ROBSTRIDE_CONTROL__CAN_INTERFACE_HPP_
