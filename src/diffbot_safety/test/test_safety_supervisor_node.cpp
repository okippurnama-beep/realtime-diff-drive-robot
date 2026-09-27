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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "gtest/gtest.h"

#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "diffbot_safety/safety_supervisor_node.hpp"

namespace diffbot_safety
{
namespace
{

using namespace std::chrono_literals;

class SafetySupervisorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
          rclcpp::Parameter("control_frequency_hz", 100.0),
          rclcpp::Parameter("status_frequency_hz", 100.0),
          rclcpp::Parameter("command_timeout_sec", 1.0),
          rclcpp::Parameter("scan_timeout_sec", 1.0),
          rclcpp::Parameter("odom_timeout_sec", 1.0),
          rclcpp::Parameter("require_nav2_active", false),
          rclcpp::Parameter("input_command_topic", "/m7_2_test/cmd_in"),
          rclcpp::Parameter("scan_topic", "/m7_2_test/scan"),
          rclcpp::Parameter("odom_topic", "/m7_2_test/odom"),
          rclcpp::Parameter("output_command_topic", "/m7_2_test/cmd_safe"),
          rclcpp::Parameter("status_topic", "/m7_2_test/status"),
    });

    supervisor_ = std::make_shared<SafetySupervisorNode>(options);
    driver_ = std::make_shared<rclcpp::Node>("safety_supervisor_test_driver");

    command_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>(
      "/m7_2_test/cmd_in", rclcpp::QoS(10).reliable());
    scan_pub_ = driver_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/m7_2_test/scan", rclcpp::SensorDataQoS());
    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/m7_2_test/odom", rclcpp::QoS(10).reliable());

    safe_sub_ = driver_->create_subscription<geometry_msgs::msg::Twist>(
      "/m7_2_test/cmd_safe",
      rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_safe_command_ = *message;
      });
    status_sub_ =
      driver_->create_subscription<diffbot_interfaces::msg::SafetyStatus>(
      "/m7_2_test/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const diffbot_interfaces::msg::SafetyStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_status_ = *message;
      });

    executor_.add_node(supervisor_);
    executor_.add_node(driver_);
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(supervisor_);
    status_sub_.reset();
    safe_sub_.reset();
    odom_pub_.reset();
    scan_pub_.reset();
    command_pub_.reset();
    driver_.reset();
    supervisor_.reset();
  }

  template<typename RepT, typename PeriodT>
  bool spin_until(
    const std::function<bool()> & predicate,
    const std::chrono::duration<RepT, PeriodT> timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  std::optional<geometry_msgs::msg::Twist> safe_command()
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_safe_command_;
  }

  std::optional<diffbot_interfaces::msg::SafetyStatus> status()
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_status_;
  }

  void publish_health_and_command(const geometry_msgs::msg::Twist & command)
  {
    scan_pub_->publish(sensor_msgs::msg::LaserScan{});
    odom_pub_->publish(nav_msgs::msg::Odometry{});
    command_pub_->publish(command);
  }

  bool wait_for_state(
    const std::uint8_t expected_state,
    const geometry_msgs::msg::Twist & command)
  {
    return spin_until(
      [this, expected_state, command]() {
        publish_health_and_command(command);
        const auto current = status();
        return current.has_value() && current->state == expected_state;
      },
      1s);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<SafetySupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_sub_;
  rclcpp::Subscription<diffbot_interfaces::msg::SafetyStatus>::SharedPtr
    status_sub_;
  std::mutex result_mutex_;
  std::optional<geometry_msgs::msg::Twist> last_safe_command_;
  std::optional<diffbot_interfaces::msg::SafetyStatus> last_status_;
};

TEST_F(SafetySupervisorNodeTest, InhibitsStartupThenPassesApprovedCommand)
{
  ASSERT_TRUE(spin_until([this]() {return safe_command().has_value();}, 1s));
  EXPECT_DOUBLE_EQ(safe_command()->linear.x, 0.0);
  ASSERT_TRUE(spin_until([this]() {return status().has_value();}, 1s));
  EXPECT_EQ(
    status()->state,
    diffbot_interfaces::msg::SafetyStatus::STATE_STARTUP_INHIBIT);

  ASSERT_TRUE(wait_for_state(
      diffbot_interfaces::msg::SafetyStatus::STATE_READY,
      geometry_msgs::msg::Twist{}));

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.angular.z = -0.4;
  ASSERT_TRUE(spin_until(
      [this, command]() {
        publish_health_and_command(command);
        const auto output = safe_command();
        return output.has_value() &&
               std::abs(output->linear.x - 0.2) < 1.0e-9 &&
               std::abs(output->angular.z + 0.4) < 1.0e-9;
      },
      1s));
}

TEST_F(SafetySupervisorNodeTest, RejectsUnsupportedDifferentialDriveAxis)
{
  ASSERT_TRUE(wait_for_state(
      diffbot_interfaces::msg::SafetyStatus::STATE_READY,
      geometry_msgs::msg::Twist{}));

  geometry_msgs::msg::Twist invalid;
  invalid.linear.y = 0.1;
  ASSERT_TRUE(spin_until(
      [this, invalid]() {
        publish_health_and_command(invalid);
        return status().has_value() &&
               status()->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_FAULT_LATCHED;
      },
      1s));
  EXPECT_NE(
    status()->latched_faults &
    diffbot_interfaces::msg::SafetyStatus::FAULT_COMMAND_LIMIT_VIOLATION,
    0U);
  ASSERT_TRUE(safe_command().has_value());
  EXPECT_DOUBLE_EQ(safe_command()->linear.x, 0.0);
  EXPECT_DOUBLE_EQ(safe_command()->linear.y, 0.0);
  EXPECT_DOUBLE_EQ(safe_command()->angular.z, 0.0);
}

}  // namespace
}  // namespace diffbot_safety
