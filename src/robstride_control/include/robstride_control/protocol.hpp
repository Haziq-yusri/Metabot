// Copyright 2024 Metabot contributors
//
// SPDX-License-Identifier: MIT
//
// RobStride motor protocol definitions, derived from the upstream
// Seeed-Projects/RobStride_Control project (cpp/include/protocol.h) and
// adapted for use inside a ROS 2 Jazzy ament_cmake package.

#ifndef ROBSTRIDE_CONTROL__PROTOCOL_HPP_
#define ROBSTRIDE_CONTROL__PROTOCOL_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>

namespace robstride_control
{

// --- Communication Types ---
namespace comm_type
{
constexpr uint32_t GET_DEVICE_ID = 0;
constexpr uint32_t OPERATION_CONTROL = 1;
constexpr uint32_t OPERATION_STATUS = 2;
constexpr uint32_t ENABLE = 3;
constexpr uint32_t DISABLE = 4;
constexpr uint32_t SET_ZERO_POSITION = 6;
constexpr uint32_t SET_DEVICE_ID = 7;
constexpr uint32_t READ_PARAMETER = 17;
constexpr uint32_t WRITE_PARAMETER = 18;
constexpr uint32_t FAULT_REPORT = 21;
constexpr uint32_t SAVE_PARAMETERS = 22;
constexpr uint32_t SET_BAUDRATE = 23;
constexpr uint32_t ACTIVE_REPORT = 24;
constexpr uint32_t SET_PROTOCOL = 25;
}  // namespace comm_type

// --- Parameter IDs ---
namespace param_id
{
constexpr uint16_t MECHANICAL_OFFSET = 0x2005;
constexpr uint16_t MEASURED_POSITION = 0x3016;
constexpr uint16_t MEASURED_VELOCITY = 0x3017;
constexpr uint16_t MEASURED_TORQUE = 0x302C;
constexpr uint16_t MODE = 0x7005;
constexpr uint16_t IQ_TARGET = 0x7006;
constexpr uint16_t VELOCITY_TARGET = 0x700A;
constexpr uint16_t TORQUE_LIMIT = 0x700B;
constexpr uint16_t POSITION_TARGET = 0x7016;
constexpr uint16_t VELOCITY_LIMIT = 0x7017;
constexpr uint16_t CURRENT_LIMIT = 0x7018;
}  // namespace param_id

// --- Default scaling for the RS-03 motor (upstream default) ---
namespace model_scale
{
constexpr double POSITION = 4.0 * M_PI;  // ±4π rad
constexpr double VELOCITY = 50.0;        // ±50 rad/s
constexpr double TORQUE = 60.0;          // ±60 Nm
constexpr double KP = 5000.0;            // 0-5000 Nm/rad
constexpr double KD = 100.0;             // 0-100 Nm/rad/s
}  // namespace model_scale

// --- Control Modes ---
namespace control_mode
{
constexpr int8_t MIT = 0;
constexpr int8_t POSITION = 1;
constexpr int8_t SPEED = 2;
constexpr int8_t TORQUE = 3;
}  // namespace control_mode

constexpr uint32_t HOST_ID = 0xFF;

inline void pack_float_le(uint8_t * buf, float val)
{
  std::memcpy(buf, &val, sizeof(float));
}

inline void pack_u16_le(uint8_t * buf, uint16_t val)
{
  buf[0] = static_cast<uint8_t>(val & 0xFF);
  buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

inline void pack_u16_be(uint8_t * buf, uint16_t val)
{
  buf[0] = static_cast<uint8_t>((val >> 8) & 0xFF);
  buf[1] = static_cast<uint8_t>(val & 0xFF);
}

inline uint16_t unpack_u16_be(const uint8_t * buf)
{
  return static_cast<uint16_t>((buf[0] << 8) | buf[1]);
}

}  // namespace robstride_control

#endif  // ROBSTRIDE_CONTROL__PROTOCOL_HPP_
