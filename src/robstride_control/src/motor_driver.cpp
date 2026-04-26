// Copyright 2024 Metabot contributors
// SPDX-License-Identifier: MIT

#include "robstride_control/motor_driver.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cstring>
#include <mutex>

namespace robstride_control
{

namespace
{

inline uint16_t encode_unsigned(double value, double range)
{
  // Map [-range, +range] -> [0, 0xFFFF] (upstream uses ±range → ±0x7FFF offset).
  double clamped = std::max(-range, std::min(range, value));
  double scaled = (clamped / range + 1.0) * 0x7FFF;
  if (scaled < 0.0) {scaled = 0.0;}
  if (scaled > 0xFFFF) {scaled = 0xFFFF;}
  return static_cast<uint16_t>(scaled);
}

inline uint16_t encode_gain(double value, double range)
{
  double clamped = std::max(0.0, std::min(range, value));
  double scaled = (clamped / range) * 0xFFFF;
  if (scaled < 0.0) {scaled = 0.0;}
  if (scaled > 0xFFFF) {scaled = 0xFFFF;}
  return static_cast<uint16_t>(scaled);
}

inline double decode_signed(uint16_t raw, double range)
{
  return (static_cast<double>(raw) / 0x7FFF - 1.0) * range;
}

}  // namespace

MotorDriver::MotorDriver() = default;

bool MotorDriver::connect(const std::string & can_interface)
{
  return can_.init(can_interface);
}

void MotorDriver::disconnect()
{
  can_.close();
}

bool MotorDriver::enable(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint32_t ext_id = (comm_type::ENABLE << 24) | (HOST_ID << 8) | motor_id;
  return can_.send_frame(ext_id, nullptr, 0);
}

bool MotorDriver::set_mit_mode(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint32_t ext_id = (comm_type::WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param_id::MODE);
  data[4] = static_cast<uint8_t>(control_mode::MIT);
  return can_.send_frame(ext_id, data, 8);
}

bool MotorDriver::write_float_parameter(uint8_t motor_id, uint16_t param, float value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint32_t ext_id = (comm_type::WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  uint8_t data[8] = {0};
  pack_u16_le(&data[0], param);
  pack_float_le(&data[4], value);
  return can_.send_frame(ext_id, data, 8);
}

bool MotorDriver::send_mit_command(
  uint8_t motor_id,
  double position,
  double kp,
  double kd,
  double velocity_ff,
  double torque_ff)
{
  std::lock_guard<std::mutex> lock(mutex_);

  uint16_t pos_u16 = encode_unsigned(position, model_scale::POSITION);
  uint16_t vel_u16 = encode_unsigned(velocity_ff, model_scale::VELOCITY);
  uint16_t kp_u16 = encode_gain(kp, model_scale::KP);
  uint16_t kd_u16 = encode_gain(kd, model_scale::KD);
  uint16_t torque_u16 = encode_unsigned(torque_ff, model_scale::TORQUE);

  uint8_t data[8];
  pack_u16_be(&data[0], pos_u16);
  pack_u16_be(&data[2], vel_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);

  // Upstream packs the torque feed-forward into the upper bits of the CAN ID.
  uint32_t ext_id =
    (comm_type::OPERATION_CONTROL << 24) |
    ((static_cast<uint32_t>(torque_u16) & 0xFFFF) << 8) |
    motor_id;

  return can_.send_frame(ext_id, data, 8);
}

MotorState MotorDriver::read_status(uint8_t motor_id, int timeout_ms)
{
  MotorState state;
  struct can_frame frame;

  // Drain the receive queue and keep the latest matching status frame.
  while (can_.read_frame(&frame, timeout_ms)) {
    if ((frame.can_id & CAN_EFF_FLAG) == 0) {continue;}
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    uint32_t comm = (id >> 24) & 0x1F;
    if (comm != comm_type::OPERATION_STATUS) {continue;}

    // Per upstream protocol: status frame ID is
    //   (2 << 24) | (motor_id << 8) | host_id
    uint8_t reporting_id = static_cast<uint8_t>((id >> 8) & 0xFF);
    if (reporting_id != motor_id) {continue;}

    if (frame.can_dlc < 8) {continue;}

    uint16_t pos_raw = unpack_u16_be(&frame.data[0]);
    uint16_t vel_raw = unpack_u16_be(&frame.data[2]);
    uint16_t tor_raw = unpack_u16_be(&frame.data[4]);
    uint16_t tmp_raw = unpack_u16_be(&frame.data[6]);

    state.position = decode_signed(pos_raw, model_scale::POSITION);
    state.velocity = decode_signed(vel_raw, model_scale::VELOCITY);
    state.torque = decode_signed(tor_raw, model_scale::TORQUE);
    state.temperature = static_cast<double>(tmp_raw) / 10.0;
    state.valid = true;

    // Use a zero timeout for subsequent reads so we only consume what is
    // already buffered.
    timeout_ms = 0;
  }

  return state;
}

bool MotorDriver::stop(uint8_t motor_id)
{
  return send_mit_command(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0);
}

}  // namespace robstride_control
