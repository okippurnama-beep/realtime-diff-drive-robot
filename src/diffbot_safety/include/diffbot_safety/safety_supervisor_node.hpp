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
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

  SafetyConfig declare_and_load_config();
  void validate_frequency(double value, const char * name) const;
  void validate_topic(const std::string & value, const char * name) const;

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void control_cycle();

  SafetySnapshot make_snapshot(SteadyTimePoint now) const;
  void publish_command(const SafetyDecision & decision);
  void publish_status(
    const SafetyDecision & decision,
    const SafetySnapshot & snapshot,
    SteadyTimePoint now,
    bool state_changed);
  static float age_seconds(bool seen, Duration age);

  mutable std::mutex input_mutex_;
  bool command_seen_{false};
  MotionCommand last_command_{};
  SteadyTimePoint last_command_time_{};
  bool scan_seen_{false};
  SteadyTimePoint last_scan_time_{};
  bool odom_seen_{false};
  SteadyTimePoint last_odom_time_{};

  std::unique_ptr<SafetyPolicy> policy_;
  double control_frequency_hz_{50.0};
  double status_frequency_hz_{10.0};
  Duration status_period_{std::chrono::milliseconds(100)};
  std::optional<SteadyTimePoint> last_status_time_;
  std::optional<SafetyState> last_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<diffbot_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace diffbot_safety

#endif  // DIFFBOT_SAFETY__SAFETY_SUPERVISOR_NODE_HPP_
