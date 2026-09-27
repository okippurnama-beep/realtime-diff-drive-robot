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

#ifndef DIFFBOT_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
#define DIFFBOT_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "diffbot_safety/safety_policy.hpp"

namespace diffbot_safety
{

class SafetySupervisorNode : public rclcpp::Node
{
public:
  explicit SafetySupervisorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTimePoint = SteadyClock::time_point;
  using Trigger = std_srvs::srv::Trigger;

  struct ManagerHealth
  {
    bool response_seen{false};
    bool active{false};
    bool request_pending{false};
    std::int64_t pending_request_id{0};
    SteadyTimePoint request_started{};
    SteadyTimePoint last_response_time{};
    std::string message{"no response"};
  };

  struct ManagerHealthView
  {
    bool response_seen{false};
    bool active{false};
    bool request_pending{false};
    Duration response_age{Duration::zero()};
    std::string message;
  };

  SafetyConfig declare_and_load_config();
  void validate_frequency(double value, const char * name) const;
  void validate_topic(const std::string & value, const char * name) const;

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void poll_nav2_health();
  void poll_manager(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    ManagerHealth & health,
    const char * manager_name,
    SteadyTimePoint now);
  void manager_response_callback(
    ManagerHealth & health,
    const char * manager_name,
    rclcpp::Client<Trigger>::SharedFuture future);
  void control_cycle();

  SafetySnapshot make_snapshot(SteadyTimePoint now) const;
  ManagerHealthView manager_health_view(
    const ManagerHealth & health,
    SteadyTimePoint now) const;
  bool manager_is_healthy(
    const ManagerHealth & health,
    SteadyTimePoint now) const;
  void publish_command(const SafetyDecision & decision);
  void publish_status(
    const SafetyDecision & decision,
    const SafetySnapshot & snapshot,
    SteadyTimePoint now,
    bool state_changed);
  void publish_diagnostics(
    const SafetyDecision & decision,
    const SafetySnapshot & snapshot,
    SteadyTimePoint now);
  static float age_seconds(bool seen, Duration age);

  mutable std::mutex input_mutex_;
  bool command_seen_{false};
  MotionCommand last_command_{};
  SteadyTimePoint last_command_time_{};
  bool scan_seen_{false};
  SteadyTimePoint last_scan_time_{};
  bool odom_seen_{false};
  SteadyTimePoint last_odom_time_{};
  ManagerHealth localization_health_;
  ManagerHealth navigation_health_;

  std::unique_ptr<SafetyPolicy> policy_;
  double control_frequency_hz_{50.0};
  double status_frequency_hz_{10.0};
  Duration status_period_{std::chrono::milliseconds(100)};
  Duration nav2_poll_period_{std::chrono::milliseconds(250)};
  Duration nav2_health_timeout_{std::chrono::seconds(1)};
  std::optional<SteadyTimePoint> last_status_time_;
  std::optional<SafetyState> last_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<diffbot_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Client<Trigger>::SharedPtr localization_health_client_;
  rclcpp::Client<Trigger>::SharedPtr navigation_health_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr nav2_health_timer_;
};

}  // namespace diffbot_safety

#endif  // DIFFBOT_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
