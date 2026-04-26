// Copyright 2024 Metabot contributors
//
// SPDX-License-Identifier: MIT
//
// High level RobStride motor driver. Provides MIT-mode position control and
// status feedback by combining the SocketCAN wrapper with the protocol
// helpers. Derived from the upstream cpp/src/position_control.cpp logic.

#ifndef ROBSTRIDE_CONTROL__MOTOR_DRIVER_HPP_
#define ROBSTRIDE_CONTROL__MOTOR_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "robstride_control/can_interface.hpp"
#include "robstride_control/protocol.hpp"

namespace robstride_control
{

struct MotorState
{
  double position{0.0};      // rad
  double velocity{0.0};      // rad/s
  double torque{0.0};        // Nm
  double temperature{0.0};   // deg C
  bool valid{false};
};

class MotorDriver
{
public:
  MotorDriver();

  /// Open the CAN bus.
  bool connect(const std::string & can_interface = "can0");

  /// Close the CAN bus.
  void disconnect();

  /// Send the enable frame to a motor.
  bool enable(uint8_t motor_id);

  /// Switch a motor into MIT mode (mode 0).
  bool set_mit_mode(uint8_t motor_id);

  /// Write a single float parameter (e.g. velocity / torque limits).
  bool write_float_parameter(uint8_t motor_id, uint16_t param_id, float value);

  /// Send an MIT operation control frame.
  ///
  /// ``position`` is in radians, ``kp`` and ``kd`` use the upstream RS-03
  /// scaling. ``velocity_ff`` and ``torque_ff`` default to zero (centred
  /// in their unsigned ranges).
  bool send_mit_command(
    uint8_t motor_id,
    double position,
    double kp,
    double kd,
    double velocity_ff = 0.0,
    double torque_ff = 0.0);

  /// Drain pending status frames and return the most recent state for
  /// ``motor_id``.  Non-matching frames are ignored.
  MotorState read_status(uint8_t motor_id, int timeout_ms = 5);

  /// Stop a motor by sending an MIT frame with zero gains and zero target.
  bool stop(uint8_t motor_id);

  bool is_connected() const {return can_.is_ready();}

private:
  CanInterface can_;
  std::mutex mutex_;
};

}  // namespace robstride_control

#endif  // ROBSTRIDE_CONTROL__MOTOR_DRIVER_HPP_
