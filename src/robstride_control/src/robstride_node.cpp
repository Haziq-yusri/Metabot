// Copyright 2024 Metabot contributors
// SPDX-License-Identifier: MIT
//
// ROS 2 Jazzy node that wraps the RobStride MIT-mode driver. It periodically
// sends position commands to one or more motors and publishes their measured
// state on ``/joint_states``.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "robstride_control/motor_driver.hpp"

namespace robstride_control
{

class RobstrideNode : public rclcpp::Node
{
public:
  RobstrideNode()
  : rclcpp::Node("robstride_node")
  {
    // --- Parameters ---
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint_1"});
    auto motor_ids_long = this->declare_parameter<std::vector<int64_t>>(
      "motor_ids", std::vector<int64_t>{11});
    default_kp_ = this->declare_parameter<double>("default_kp", 50.0);
    default_kd_ = this->declare_parameter<double>("default_kd", 1.0);
    velocity_limit_ = this->declare_parameter<double>("velocity_limit", 20.0);
    torque_limit_ = this->declare_parameter<double>("torque_limit", 20.0);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);

    if (joint_names_.size() != motor_ids_long.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "joint_names (%zu) and motor_ids (%zu) must have the same length",
        joint_names_.size(), motor_ids_long.size());
      throw std::runtime_error("invalid robstride_node configuration");
    }

    motor_ids_.reserve(motor_ids_long.size());
    for (auto id : motor_ids_long) {
      if (id < 0 || id > 0xFF) {
        RCLCPP_FATAL(get_logger(), "motor_ids entry %ld is out of range (0..255)", id);
        throw std::runtime_error("invalid robstride_node configuration");
      }
      motor_ids_.push_back(static_cast<uint8_t>(id));
    }

    target_positions_.assign(joint_names_.size(), 0.0);

    // --- Connect to the bus ---
    if (!driver_.connect(can_interface_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open CAN interface '%s'. The node will keep running but "
        "no commands will be sent until the bus is available.",
        can_interface_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Connected to CAN interface '%s'", can_interface_.c_str());
      initialise_motors();
    }

    // --- ROS interfaces ---
    state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", 10,
      std::bind(&RobstrideNode::on_command, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RobstrideNode::on_timer, this));
  }

  ~RobstrideNode() override
  {
    if (driver_.is_connected()) {
      for (auto id : motor_ids_) {
        driver_.stop(id);
      }
      driver_.disconnect();
    }
  }

private:
  void initialise_motors()
  {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      auto id = motor_ids_[i];
      driver_.enable(id);
      driver_.set_mit_mode(id);
      driver_.write_float_parameter(id, param_id::VELOCITY_LIMIT,
        static_cast<float>(velocity_limit_));
      driver_.write_float_parameter(id, param_id::TORQUE_LIMIT,
        static_cast<float>(torque_limit_));
      RCLCPP_INFO(
        get_logger(), "Initialised motor id=%u as joint '%s'",
        static_cast<unsigned>(id), joint_names_[i].c_str());
    }
  }

  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Build a name -> position map from the incoming command for partial
    // updates (only joints listed in the message are updated).
    std::unordered_map<std::string, double> positions;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      positions[msg->name[i]] = msg->position[i];
    }

    if (positions.empty() && msg->position.size() == joint_names_.size()) {
      // Allow positional-only commands when the size matches our joints.
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        target_positions_[i] = msg->position[i];
      }
      return;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = positions.find(joint_names_[i]);
      if (it != positions.end()) {
        target_positions_[i] = it->second;
      }
    }
  }

  void on_timer()
  {
    if (!driver_.is_connected()) {
      // Try to reconnect periodically.
      if (driver_.connect(can_interface_)) {
        RCLCPP_INFO(get_logger(), "Re-connected to CAN interface '%s'", can_interface_.c_str());
        initialise_motors();
      } else {
        return;
      }
    }

    sensor_msgs::msg::JointState state_msg;
    state_msg.header.stamp = this->now();
    state_msg.name = joint_names_;
    state_msg.position.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    state_msg.velocity.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    state_msg.effort.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      auto id = motor_ids_[i];
      driver_.send_mit_command(id, target_positions_[i], default_kp_, default_kd_);
      auto state = driver_.read_status(id, /*timeout_ms=*/1);
      if (state.valid) {
        state_msg.position[i] = state.position;
        state_msg.velocity[i] = state.velocity;
        state_msg.effort[i] = state.torque;
      }
    }

    state_pub_->publish(state_msg);
  }

  // Parameters
  std::string can_interface_;
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> motor_ids_;
  double default_kp_{50.0};
  double default_kd_{1.0};
  double velocity_limit_{20.0};
  double torque_limit_{20.0};
  double publish_rate_hz_{50.0};

  // State
  std::vector<double> target_positions_;
  MotorDriver driver_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robstride_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robstride_control::RobstrideNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "robstride_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
