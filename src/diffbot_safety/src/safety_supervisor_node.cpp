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
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

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

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string bool_string(const bool value)
{
  return value ? "true" : "false";
}

std::string duration_string(const Duration value)
{
  return std::to_string(std::chrono::duration<double>(value).count());
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
  nav2_poll_period_ = checked_positive_seconds(
    declare_parameter("nav2_poll_period_sec", 0.25),
    "nav2_poll_period_sec");
  nav2_health_timeout_ = checked_positive_seconds(
    declare_parameter("nav2_health_timeout_sec", 1.00),
    "nav2_health_timeout_sec");

  const std::string input_command_topic = declare_parameter(
    "input_command_topic", "/cmd_vel_collision_checked");
  const std::string scan_topic = declare_parameter("scan_topic", "/scan");
  const std::string odom_topic = declare_parameter(
    "odom_topic", "/odometry/filtered");
  const std::string output_command_topic = declare_parameter(
    "output_command_topic", "/cmd_vel_safe");
  const std::string status_topic = declare_parameter(
    "status_topic", "/safety/status");
  const std::string diagnostics_topic = declare_parameter(
    "diagnostics_topic", "/diagnostics");
  const std::string localization_manager_service = declare_parameter(
    "localization_manager_service",
    "/lifecycle_manager_localization/is_active");
  const std::string navigation_manager_service = declare_parameter(
    "navigation_manager_service",
    "/lifecycle_manager_navigation/is_active");

  validate_topic(input_command_topic, "input_command_topic");
  validate_topic(scan_topic, "scan_topic");
  validate_topic(odom_topic, "odom_topic");
  validate_topic(output_command_topic, "output_command_topic");
  validate_topic(status_topic, "status_topic");
  validate_topic(diagnostics_topic, "diagnostics_topic");
  validate_topic(
    localization_manager_service,
    "localization_manager_service");
  validate_topic(navigation_manager_service, "navigation_manager_service");

  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    output_command_topic, rclcpp::QoS(10).reliable());
  status_pub_ = create_publisher<diffbot_interfaces::msg::SafetyStatus>(
    status_topic, rclcpp::QoS(1).reliable().transient_local());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic, rclcpp::QoS(10).reliable());

  localization_health_client_ = create_client<Trigger>(
    localization_manager_service);
  navigation_health_client_ = create_client<Trigger>(
    navigation_manager_service);

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
  nav2_health_timer_ = create_wall_timer(
    nav2_poll_period_,
    std::bind(&SafetySupervisorNode::poll_nav2_health, this));

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
    "require_nav2_active", true);
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

void SafetySupervisorNode::poll_nav2_health()
{
  const SteadyTimePoint now = SteadyClock::now();
  poll_manager(
    localization_health_client_,
    localization_health_,
    "localization",
    now);
  poll_manager(
    navigation_health_client_,
    navigation_health_,
    "navigation",
    now);
}

void SafetySupervisorNode::poll_manager(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  ManagerHealth & health,
  const char * manager_name,
  const SteadyTimePoint now)
{
  std::int64_t expired_request_id = 0;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (health.request_pending) {
      if (now - health.request_started <= nav2_health_timeout_) {
        return;
      }
      expired_request_id = health.pending_request_id;
      health.request_pending = false;
      health.pending_request_id = 0;
      health.active = false;
      health.message = "request timed out";
    }
  }

  if (expired_request_id != 0) {
    client->remove_pending_request(expired_request_id);
    RCLCPP_WARN(
      get_logger(),
      "%s lifecycle-manager health request timed out",
      manager_name);
  }

  if (!client->service_is_ready()) {
    return;
  }

  const auto request = std::make_shared<Trigger::Request>();
  std::lock_guard<std::mutex> lock(input_mutex_);
  auto future = client->async_send_request(
    request,
    [this, &health, manager_name](rclcpp::Client<Trigger>::SharedFuture result) {
      manager_response_callback(health, manager_name, result);
    });
  health.request_pending = true;
  health.pending_request_id = future.request_id;
  health.request_started = now;
}

void SafetySupervisorNode::manager_response_callback(
  ManagerHealth & health,
  const char * manager_name,
  rclcpp::Client<Trigger>::SharedFuture future)
{
  bool active = false;
  std::string message;
  try {
    const auto response = future.get();
    active = response->success;
    message = response->message;
  } catch (const std::exception & error) {
    message = error.what();
  }

  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    health.response_seen = true;
    health.active = active;
    health.request_pending = false;
    health.pending_request_id = 0;
    health.last_response_time = SteadyClock::now();
    health.message = message;
  }

  if (!active) {
    RCLCPP_WARN(
      get_logger(),
      "%s lifecycle manager reported inactive: %s",
      manager_name,
      message.c_str());
  }
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

  snapshot.nav2_active =
    manager_is_healthy(localization_health_, now) &&
    manager_is_healthy(navigation_health_, now);

  return snapshot;
}

SafetySupervisorNode::ManagerHealthView
SafetySupervisorNode::manager_health_view(
  const ManagerHealth & health,
  const SteadyTimePoint now) const
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  ManagerHealthView view;
  view.response_seen = health.response_seen;
  view.active = health.active;
  view.request_pending = health.request_pending;
  if (health.response_seen) {
    view.response_age = now - health.last_response_time;
  }
  view.message = health.message;
  return view;
}

bool SafetySupervisorNode::manager_is_healthy(
  const ManagerHealth & health,
  const SteadyTimePoint now) const
{
  return health.response_seen && health.active &&
         now - health.last_response_time <= nav2_health_timeout_;
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
  publish_diagnostics(decision, snapshot, now);
  last_status_time_ = now;
}

void SafetySupervisorNode::publish_diagnostics(
  const SafetyDecision & decision,
  const SafetySnapshot & snapshot,
  const SteadyTimePoint now)
{
  const auto localization = manager_health_view(localization_health_, now);
  const auto navigation = manager_health_view(navigation_health_, now);
  const SafetyConfig & config = policy_->config();

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "diffbot_safety/supervisor";
  status.hardware_id = "host";
  status.message = state_name(decision.state);
  if (decision.state == SafetyState::kReady &&
    decision.active_faults == to_mask(Fault::kNone) &&
    decision.latched_faults == to_mask(Fault::kNone))
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (decision.state == SafetyState::kStartupInhibit) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  status.values = {
    diagnostic_value("state", state_name(decision.state)),
    diagnostic_value(
      "active_faults", fault_mask_to_string(decision.active_faults)),
    diagnostic_value(
      "latched_faults", fault_mask_to_string(decision.latched_faults)),
    diagnostic_value("output_inhibited", bool_string(decision.output_inhibited)),
    diagnostic_value(
      "command_age_sec",
      std::to_string(age_seconds(snapshot.command_seen, snapshot.command_age))),
    diagnostic_value(
      "scan_age_sec",
      std::to_string(age_seconds(snapshot.scan_seen, snapshot.scan_age))),
    diagnostic_value(
      "odom_age_sec",
      std::to_string(age_seconds(snapshot.odom_seen, snapshot.odom_age))),
    diagnostic_value(
      "mcu_heartbeat_age_sec",
      std::to_string(age_seconds(
          snapshot.mcu_heartbeat_seen,
          snapshot.mcu_heartbeat_age))),
    diagnostic_value(
      "localization_manager_response_seen",
      bool_string(localization.response_seen)),
    diagnostic_value(
      "localization_manager_active", bool_string(localization.active)),
    diagnostic_value(
      "localization_manager_request_pending",
      bool_string(localization.request_pending)),
    diagnostic_value(
      "localization_manager_age_sec",
      std::to_string(age_seconds(
          localization.response_seen,
          localization.response_age))),
    diagnostic_value(
      "localization_manager_message", localization.message),
    diagnostic_value(
      "navigation_manager_response_seen",
      bool_string(navigation.response_seen)),
    diagnostic_value(
      "navigation_manager_active", bool_string(navigation.active)),
    diagnostic_value(
      "navigation_manager_request_pending",
      bool_string(navigation.request_pending)),
    diagnostic_value(
      "navigation_manager_age_sec",
      std::to_string(age_seconds(
          navigation.response_seen,
          navigation.response_age))),
    diagnostic_value("navigation_manager_message", navigation.message),
    diagnostic_value(
      "command_timeout_sec", duration_string(config.command_timeout)),
    diagnostic_value(
      "scan_timeout_sec", duration_string(config.scan_timeout)),
    diagnostic_value(
      "odom_timeout_sec", duration_string(config.odom_timeout)),
    diagnostic_value(
      "nav2_poll_period_sec", duration_string(nav2_poll_period_)),
    diagnostic_value(
      "nav2_health_timeout_sec", duration_string(nav2_health_timeout_)),
  };
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

float SafetySupervisorNode::age_seconds(const bool seen, const Duration age)
{
  if (!seen) {
    return -1.0F;
  }
  return std::chrono::duration<float>(age).count();
}

}  // namespace diffbot_safety
