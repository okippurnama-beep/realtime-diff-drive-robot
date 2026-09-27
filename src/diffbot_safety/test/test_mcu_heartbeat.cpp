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
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "gtest/gtest.h"

#include "diffbot_interfaces/msg/mcu_state.hpp"
#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diffbot_safety/safety_supervisor_node.hpp"

namespace diffbot_safety
{
namespace
{

using namespace std::chrono_literals;

class McuHeartbeatTest : public ::testing::Test
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
    driver_ = std::make_shared<rclcpp::Node>("m8_4_heartbeat_driver");
    rclcpp::NodeOptions options;
    options.parameter_overrides({
          rclcpp::Parameter("control_frequency_hz", 100.0),
          rclcpp::Parameter("status_frequency_hz", 100.0),
          rclcpp::Parameter("command_timeout_sec", 0.20),
          rclcpp::Parameter("mcu_heartbeat_timeout_sec", 0.08),
          rclcpp::Parameter("require_scan", false),
          rclcpp::Parameter("require_odom", false),
          rclcpp::Parameter("require_nav2_active", false),
          rclcpp::Parameter("require_mcu_heartbeat", true),
          rclcpp::Parameter("input_command_topic", "/m8_4_test/cmd_in"),
          rclcpp::Parameter("mcu_state_topic", "/m8_4_test/mcu_state"),
          rclcpp::Parameter("output_command_topic", "/m8_4_test/cmd_safe"),
          rclcpp::Parameter("status_topic", "/m8_4_test/status"),
          rclcpp::Parameter("diagnostics_topic", "/m8_4_test/diagnostics"),
          rclcpp::Parameter("estop_topic", "/m8_4_test/estop"),
          rclcpp::Parameter("reset_service", "/m8_4_test/reset"),
          rclcpp::Parameter(
            "localization_manager_service", "/m8_4_test/localization"),
          rclcpp::Parameter(
            "navigation_manager_service", "/m8_4_test/navigation"),
    });
    supervisor_ = std::make_shared<SafetySupervisorNode>(options);
    command_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>(
      "/m8_4_test/cmd_in", 10);
    mcu_pub_ = driver_->create_publisher<diffbot_interfaces::msg::McuState>(
      "/m8_4_test/mcu_state", 10);
    status_sub_ = driver_->create_subscription<
      diffbot_interfaces::msg::SafetyStatus>(
      "/m8_4_test/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const diffbot_interfaces::msg::SafetyStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = *message;
      });
    executor_.add_node(driver_);
    executor_.add_node(supervisor_);
  }

  void TearDown() override
  {
    executor_.remove_node(supervisor_);
    executor_.remove_node(driver_);
    status_sub_.reset();
    mcu_pub_.reset();
    command_pub_.reset();
    supervisor_.reset();
    driver_.reset();
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
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
    return predicate();
  }

  std::optional<diffbot_interfaces::msg::SafetyStatus> status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  static diffbot_interfaces::msg::McuState valid_state(
    const std::uint32_t sequence)
  {
    diffbot_interfaces::msg::McuState state;
    state.protocol_version = state.PROTOCOL_VERSION_CURRENT;
    state.mcu_boot_id = 7U;
    state.state_sequence = sequence;
    state.accepted_host_session_id = 9U;
    state.mode = state.MODE_ARMED;
    state.active_faults = state.FAULT_NONE;
    return state;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr driver_;
  std::shared_ptr<SafetySupervisorNode> supervisor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<diffbot_interfaces::msg::McuState>::SharedPtr mcu_pub_;
  rclcpp::Subscription<diffbot_interfaces::msg::SafetyStatus>::SharedPtr status_sub_;
  std::mutex mutex_;
  std::optional<diffbot_interfaces::msg::SafetyStatus> status_;
};

TEST_F(McuHeartbeatTest, OnlyFreshValidatedStateMaintainsHeartbeat)
{
  std::uint32_t sequence = 1U;
  const geometry_msgs::msg::Twist zero;
  ASSERT_TRUE(spin_until(
      [this, &sequence, &zero]() {
        command_pub_->publish(zero);
        mcu_pub_->publish(valid_state(sequence++));
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_READY;
      },
      1s));

  auto invalid = valid_state(sequence);
  invalid.mode = invalid.MODE_FAULT;
  invalid.active_faults = invalid.FAULT_COMMAND_TIMEOUT;
  ASSERT_TRUE(spin_until(
      [this, &invalid, &zero]() {
        command_pub_->publish(zero);
        mcu_pub_->publish(invalid);
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_FAULT_LATCHED &&
               (current->latched_faults &
               diffbot_interfaces::msg::SafetyStatus::FAULT_MCU_HEARTBEAT_LOST) != 0U;
      },
      300ms));
}

TEST_F(McuHeartbeatTest, DuplicateStateSequenceCannotRefreshHeartbeat)
{
  const geometry_msgs::msg::Twist zero;
  const auto state = valid_state(11U);
  ASSERT_TRUE(spin_until(
      [this, &state, &zero]() {
        command_pub_->publish(zero);
        mcu_pub_->publish(state);
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_READY;
      },
      1s));

  ASSERT_TRUE(spin_until(
      [this, &state, &zero]() {
        command_pub_->publish(zero);
        mcu_pub_->publish(state);
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_FAULT_LATCHED;
      },
      300ms));
}

}  // namespace
}  // namespace diffbot_safety
