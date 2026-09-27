// Copyright 2026 xiayuru
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffbot_safety/safety_supervisor_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace diffbot_safety
{

namespace
{

Duration seconds_to_duration(const double seconds)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::duration<double>(seconds));
}

Duration checked_positive_seconds(const double seconds, const char * name)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
  return seconds_to_duration(seconds);
}

}  // namespace

SafetySupervisorNode::SafetySupervisorNode(const rclcpp::NodeOptions & options)
: Node("safety_supervisor", options)
{
  const SafetyConfig config = declare_and_load_config();
  policy_ = std::make_unique<SafetyPolicy>(config);

  control_frequency_hz_ = declare_parameter("control_frequency_hz", 50.0);
  status_frequency_hz_ = declare_parameter("status_frequency_hz", 10.0);
  validate_frequency(control_frequency_hz_, "control_frequency_hz");
  validate_frequency(status_frequency_hz_, "status_frequency_hz");
  status_period_ = seconds_to_duration(1.0 / status_frequency_hz_);

  const std::string input_command_topic = declare_parameter(
    "input_command_topic", "/cmd_vel_collision_checked");
  const std::string scan_topic = declare_parameter("scan_topic", "/scan");
  const std::string odom_topic = declare_parameter(
    "odom_topic", "/odometry/filtered");
  const std::string output_command_topic = declare_parameter(
    "output_command_topic", "/cmd_vel_safe");
  const std::string status_topic = declare_parameter(
    "status_topic", "/safety/status");

  validate_topic(input_command_topic, "input_command_topic");
  validate_topic(scan_topic, "scan_topic");
  validate_topic(odom_topic, "odom_topic");
  validate_topic(output_command_topic, "output_command_topic");
  validate_topic(status_topic, "status_topic");

  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    output_command_topic, rclcpp::QoS(10).reliable());
  status_pub_ = create_publisher<diffbot_interfaces::msg::SafetyStatus>(
    status_topic, rclcpp::QoS(1).reliable().transient_local());

  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_command_topic,
    rclcpp::QoS(10).reliable(),
    std::bind(
      &SafetySupervisorNode::command_callback,
      this,
      std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(
      &SafetySupervisorNode::scan_callback,
      this,
      std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::QoS(10).reliable(),
    std::bind(
      &SafetySupervisorNode::odom_callback,
      this,
      std::placeholders::_1));

  const Duration control_period = seconds_to_duration(
    1.0 / control_frequency_hz_);
  control_timer_ = create_wall_timer(
    control_period,
    std::bind(&SafetySupervisorNode::control_cycle, this));

  RCLCPP_INFO(
    get_logger(),
    "Safety command gate: %s -> %s at %.1f Hz",
    input_command_topic.c_str(),
    output_command_topic.c_str(),
    control_frequency_hz_);
}

SafetyConfig SafetySupervisorNode::declare_and_load_config()
{
  SafetyConfig config;
  config.command_timeout = checked_positive_seconds(
    declare_parameter("command_timeout_sec", 0.30),
    "command_timeout_sec");
  config.scan_timeout = checked_positive_seconds(
    declare_parameter("scan_timeout_sec", 0.50),
    "scan_timeout_sec");
  config.odom_timeout = checked_positive_seconds(
    declare_parameter("odom_timeout_sec", 0.50),
    "odom_timeout_sec");
  config.mcu_heartbeat_timeout = checked_positive_seconds(
    declare_parameter("mcu_heartbeat_timeout_sec", 0.20),
    "mcu_heartbeat_timeout_sec");

  config.max_forward_velocity = declare_parameter(
    "max_forward_velocity", 0.25);
  config.max_reverse_velocity = declare_parameter(
    "max_reverse_velocity", 0.10);
  config.max_angular_velocity = declare_parameter(
    "max_angular_velocity", 1.0);
  config.zero_velocity_epsilon = declare_parameter(
    "zero_velocity_epsilon", 1.0e-4);

  config.require_scan = declare_parameter("require_scan", true);
  config.require_odom = declare_parameter("require_odom", true);
  config.require_nav2_active = declare_parameter(
    "require_nav2_active", false);
  config.require_mcu_heartbeat = declare_parameter(
    "require_mcu_heartbeat", false);
  return config;
}

void SafetySupervisorNode::validate_frequency(
  const double value,
  const char * name) const
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void SafetySupervisorNode::validate_topic(
  const std::string & value,
  const char * name) const
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(name) + " cannot be empty");
  }
}

void SafetySupervisorNode::command_callback(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  command_seen_ = true;
  last_command_.linear_x = message->linear.x;
  last_command_.angular_z = message->angular.z;
  last_command_.linear_y = message->linear.y;
  last_command_.linear_z = message->linear.z;
  last_command_.angular_x = message->angular.x;
  last_command_.angular_y = message->angular.y;
  last_command_time_ = SteadyClock::now();
}

void SafetySupervisorNode::scan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  (void)message;
  std::lock_guard<std::mutex> lock(input_mutex_);
  scan_seen_ = true;
  last_scan_time_ = SteadyClock::now();
}

void SafetySupervisorNode::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  (void)message;
  std::lock_guard<std::mutex> lock(input_mutex_);
  odom_seen_ = true;
  last_odom_time_ = SteadyClock::now();
}

void SafetySupervisorNode::control_cycle()
{
  const SteadyTimePoint now = SteadyClock::now();
  const SafetySnapshot snapshot = make_snapshot(now);
  const SafetyDecision decision = policy_->evaluate(snapshot);
  const bool state_changed = !last_state_.has_value() ||
    decision.state != last_state_.value();

  publish_command(decision);
  publish_status(decision, snapshot, now, state_changed);

  if (state_changed) {
    RCLCPP_INFO(
      get_logger(),
      "Safety state=%s active=%s latched=%s",
      state_name(decision.state).c_str(),
      fault_mask_to_string(decision.active_faults).c_str(),
      fault_mask_to_string(decision.latched_faults).c_str());
    last_state_ = decision.state;
  }
}

SafetySnapshot SafetySupervisorNode::make_snapshot(
  const SteadyTimePoint now) const
{
  SafetySnapshot snapshot;
  std::lock_guard<std::mutex> lock(input_mutex_);

  snapshot.command_seen = command_seen_;
  snapshot.command = last_command_;
  if (command_seen_) {
    snapshot.command_age = now - last_command_time_;
  }

  snapshot.scan_seen = scan_seen_;
  if (scan_seen_) {
    snapshot.scan_age = now - last_scan_time_;
  }

  snapshot.odom_seen = odom_seen_;
  if (odom_seen_) {
    snapshot.odom_age = now - last_odom_time_;
  }

  return snapshot;
}

void SafetySupervisorNode::publish_command(const SafetyDecision & decision)
{
  geometry_msgs::msg::Twist output;
  if (!decision.output_inhibited) {
    output.linear.x = decision.output_command.linear_x;
    output.angular.z = decision.output_command.angular_z;
  }
  command_pub_->publish(output);
}

void SafetySupervisorNode::publish_status(
  const SafetyDecision & decision,
  const SafetySnapshot & snapshot,
  const SteadyTimePoint now,
  const bool state_changed)
{
  const bool period_elapsed = !last_status_time_.has_value() ||
    now - last_status_time_.value() >= status_period_;
  if (!state_changed && !period_elapsed) {
    return;
  }

  diffbot_interfaces::msg::SafetyStatus status;
  status.stamp = get_clock()->now();
  status.state = static_cast<std::uint8_t>(decision.state);
  status.active_faults = decision.active_faults;
  status.latched_faults = decision.latched_faults;
  status.output_inhibited = decision.output_inhibited;
  status.command_age_sec = age_seconds(
    snapshot.command_seen, snapshot.command_age);
  status.scan_age_sec = age_seconds(snapshot.scan_seen, snapshot.scan_age);
  status.odom_age_sec = age_seconds(snapshot.odom_seen, snapshot.odom_age);
  status.mcu_heartbeat_age_sec = age_seconds(
    snapshot.mcu_heartbeat_seen, snapshot.mcu_heartbeat_age);
  status.summary = state_name(decision.state) +
    " active=" + fault_mask_to_string(decision.active_faults) +
    " latched=" + fault_mask_to_string(decision.latched_faults);
  status_pub_->publish(status);
  last_status_time_ = now;
}

float SafetySupervisorNode::age_seconds(const bool seen, const Duration age)
{
  if (!seen) {
    return -1.0F;
  }
  return std::chrono::duration<float>(age).count();
}

}  // namespace diffbot_safety
