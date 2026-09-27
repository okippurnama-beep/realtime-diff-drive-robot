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

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "diffbot_safety/safety_supervisor_node.hpp"

namespace diffbot_safety
{
namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

class SafetyEstopResetTest : public ::testing::Test
{
protected:
  struct SafeSample
  {
    geometry_msgs::msg::Twist command;
    SteadyClock::time_point received_at;
  };

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
    driver_ = std::make_shared<rclcpp::Node>("m7_4_estop_reset_driver");
    localization_manager_ = std::make_shared<rclcpp::Node>(
      "m7_4_localization_manager");
    navigation_manager_ = std::make_shared<rclcpp::Node>(
      "m7_4_navigation_manager");

    localization_service_ =
      localization_manager_->create_service<std_srvs::srv::Trigger>(
      "/m7_4_test/localization/is_active",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = localization_active_.load();
        response->message = response->success ? "active" : "inactive";
      });
    navigation_service_ =
      navigation_manager_->create_service<std_srvs::srv::Trigger>(
      "/m7_4_test/navigation/is_active",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = navigation_active_.load();
        response->message = response->success ? "active" : "inactive";
      });

    command_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>(
      "/m7_4_test/cmd_in", rclcpp::QoS(10).reliable());
    scan_pub_ = driver_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/m7_4_test/scan", rclcpp::SensorDataQoS());
    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/m7_4_test/odom", rclcpp::QoS(10).reliable());
    estop_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/m7_4_test/estop",
      rclcpp::QoS(1).reliable().transient_local());

    safe_sub_ = driver_->create_subscription<geometry_msgs::msg::Twist>(
      "/m7_4_test/cmd_safe",
      rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_safe_sample_ = SafeSample{*message, SteadyClock::now()};
      });
    status_sub_ =
      driver_->create_subscription<diffbot_interfaces::msg::SafetyStatus>(
      "/m7_4_test/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const diffbot_interfaces::msg::SafetyStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_status_ = *message;
      });

    executor_.add_node(driver_);
    executor_.add_node(localization_manager_);
    executor_.add_node(navigation_manager_);
    create_supervisor();
  }

  void TearDown() override
  {
    destroy_supervisor();
    executor_.remove_node(navigation_manager_);
    executor_.remove_node(localization_manager_);
    executor_.remove_node(driver_);

    status_sub_.reset();
    safe_sub_.reset();
    estop_pub_.reset();
    odom_pub_.reset();
    scan_pub_.reset();
    command_pub_.reset();
    navigation_service_.reset();
    localization_service_.reset();
    navigation_manager_.reset();
    localization_manager_.reset();
    driver_.reset();
  }

  void create_supervisor()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
          rclcpp::Parameter("control_frequency_hz", 100.0),
          rclcpp::Parameter("status_frequency_hz", 100.0),
          rclcpp::Parameter("command_timeout_sec", 0.12),
          rclcpp::Parameter("scan_timeout_sec", 0.12),
          rclcpp::Parameter("odom_timeout_sec", 0.12),
          rclcpp::Parameter("nav2_poll_period_sec", 0.02),
          rclcpp::Parameter("nav2_health_timeout_sec", 0.15),
          rclcpp::Parameter("reset_health_hold_sec", 0.10),
          rclcpp::Parameter("require_nav2_active", true),
          rclcpp::Parameter("input_command_topic", "/m7_4_test/cmd_in"),
          rclcpp::Parameter("scan_topic", "/m7_4_test/scan"),
          rclcpp::Parameter("odom_topic", "/m7_4_test/odom"),
          rclcpp::Parameter("output_command_topic", "/m7_4_test/cmd_safe"),
          rclcpp::Parameter("status_topic", "/m7_4_test/status"),
          rclcpp::Parameter("diagnostics_topic", "/m7_4_test/diagnostics"),
          rclcpp::Parameter("estop_topic", "/m7_4_test/estop"),
          rclcpp::Parameter("reset_service", "/m7_4_test/reset"),
          rclcpp::Parameter(
            "localization_manager_service",
            "/m7_4_test/localization/is_active"),
          rclcpp::Parameter(
            "navigation_manager_service",
            "/m7_4_test/navigation/is_active"),
    });
    supervisor_ = std::make_shared<SafetySupervisorNode>(options);
    executor_.add_node(supervisor_);
    reset_client_ = driver_->create_client<std_srvs::srv::Trigger>(
      "/m7_4_test/reset");
  }

  void destroy_supervisor()
  {
    reset_client_.reset();
    if (supervisor_) {
      executor_.remove_node(supervisor_);
      supervisor_.reset();
    }
  }

  template<typename RepT, typename PeriodT>
  bool spin_until(
    const std::function<bool()> & predicate,
    const std::chrono::duration<RepT, PeriodT> timeout)
  {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
    return predicate();
  }

  void publish_inputs(
    const geometry_msgs::msg::Twist * command,
    const bool publish_scan,
    const bool publish_odom)
  {
    if (command != nullptr) {
      command_pub_->publish(*command);
    }
    if (publish_scan) {
      scan_pub_->publish(sensor_msgs::msg::LaserScan{});
    }
    if (publish_odom) {
      odom_pub_->publish(nav_msgs::msg::Odometry{});
    }
  }

  void publish_estop(const bool asserted)
  {
    std_msgs::msg::Bool message;
    message.data = asserted;
    estop_pub_->publish(message);
  }

  bool wait_for_ready()
  {
    const geometry_msgs::msg::Twist zero;
    return spin_until(
      [this, &zero]() {
        publish_inputs(&zero, true, true);
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_READY;
      },
      2s);
  }

  bool wait_for_moving_command(const geometry_msgs::msg::Twist & command)
  {
    return spin_until(
      [this, &command]() {
        publish_inputs(&command, true, true);
        const auto sample = safe_sample();
        return sample.has_value() &&
               std::abs(sample->command.linear.x - command.linear.x) < 1.0e-9;
      },
      1s);
  }

  void maintain_zero_health(const std::chrono::milliseconds duration)
  {
    const geometry_msgs::msg::Twist zero;
    const auto deadline = SteadyClock::now() + duration;
    while (SteadyClock::now() < deadline) {
      publish_inputs(&zero, true, true);
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
  }

  std::optional<std_srvs::srv::Trigger::Response> call_reset()
  {
    if (!spin_until(
        [this]() {return reset_client_->service_is_ready();}, 1s))
    {
      return std::nullopt;
    }
    auto future = reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!spin_until(
        [&future]() {
          return future.wait_for(0s) == std::future_status::ready;
        },
        1s))
    {
      return std::nullopt;
    }
    return *future.get();
  }

  std::optional<SafeSample> safe_sample()
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_safe_sample_;
  }

  std::optional<diffbot_interfaces::msg::SafetyStatus> status()
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_status_;
  }

  bool state_and_zero(const std::uint8_t state)
  {
    const auto current_status = status();
    const auto sample = safe_sample();
    return current_status.has_value() && sample.has_value() &&
           current_status->state == state &&
           std::abs(sample->command.linear.x) < 1.0e-12 &&
           std::abs(sample->command.angular.z) < 1.0e-12;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Node::SharedPtr localization_manager_;
  rclcpp::Node::SharedPtr navigation_manager_;
  std::shared_ptr<SafetySupervisorNode> supervisor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localization_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr navigation_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_sub_;
  rclcpp::Subscription<diffbot_interfaces::msg::SafetyStatus>::SharedPtr
    status_sub_;
  std::atomic<bool> localization_active_{true};
  std::atomic<bool> navigation_active_{true};
  std::mutex result_mutex_;
  std::optional<SafeSample> last_safe_sample_;
  std::optional<diffbot_interfaces::msg::SafetyStatus> last_status_;
};

TEST_F(SafetyEstopResetTest, ReadyEstopImmediatelyPublishesZeroAndLatches)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  const auto asserted_at = SteadyClock::now();
  publish_estop(true);
  ASSERT_TRUE(spin_until(
      [this]() {
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      200ms));
  ASSERT_TRUE(safe_sample().has_value());
  EXPECT_LT(safe_sample()->received_at - asserted_at, 40ms);
  ASSERT_TRUE(status().has_value());
  EXPECT_NE(
    status()->latched_faults &
    diffbot_interfaces::msg::SafetyStatus::FAULT_MANUAL_ESTOP,
    0U);
}

TEST_F(SafetyEstopResetTest, EstopDuringStartupAlsoLatches)
{
  publish_estop(true);
  ASSERT_TRUE(spin_until(
      [this]() {
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      500ms));
  ASSERT_TRUE(status().has_value());
  EXPECT_NE(
    status()->active_faults &
    diffbot_interfaces::msg::SafetyStatus::FAULT_MANUAL_ESTOP,
    0U);
}

TEST_F(SafetyEstopResetTest, ReleaseDoesNotResumeAndAssertedResetIsRejected)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));
  publish_estop(true);
  ASSERT_TRUE(spin_until(
      [this]() {
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      200ms));

  const auto asserted_reset = call_reset();
  ASSERT_TRUE(asserted_reset.has_value());
  EXPECT_FALSE(asserted_reset->success);
  EXPECT_EQ(
    asserted_reset->message,
    "reset rejected: MANUAL_ESTOP active");

  publish_estop(false);
  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, true, true);
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      200ms));
  const auto moving_reset = call_reset();
  ASSERT_TRUE(moving_reset.has_value());
  EXPECT_FALSE(moving_reset->success);
  EXPECT_EQ(
    moving_reset->message,
    "reset rejected: fresh zero command required");
}

TEST_F(SafetyEstopResetTest, ResetRejectsActiveSensorAndNav2Faults)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, false, true);
        const auto current = status();
        return current.has_value() &&
               current->state ==
               diffbot_interfaces::msg::SafetyStatus::STATE_FAULT_LATCHED;
      },
      400ms));
  const geometry_msgs::msg::Twist zero;
  publish_inputs(&zero, false, true);
  const auto sensor_reset = call_reset();
  ASSERT_TRUE(sensor_reset.has_value());
  EXPECT_FALSE(sensor_reset->success);
  EXPECT_EQ(sensor_reset->message, "reset rejected: SCAN_STALE active");

  navigation_active_.store(false);
  ASSERT_TRUE(spin_until(
      [this, &zero]() {
        publish_inputs(&zero, true, true);
        const auto current = status();
        return current.has_value() &&
               (current->active_faults &
               diffbot_interfaces::msg::SafetyStatus::FAULT_NAV2_INACTIVE) !=
               0U;
      },
      500ms));
  const auto nav2_reset = call_reset();
  ASSERT_TRUE(nav2_reset.has_value());
  EXPECT_FALSE(nav2_reset->success);
  EXPECT_EQ(nav2_reset->message, "reset rejected: NAV2_INACTIVE active");
}

TEST_F(SafetyEstopResetTest, ResetRequiresFreshZeroAndFullHealthHold)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));
  publish_estop(true);
  ASSERT_TRUE(spin_until(
      [this]() {
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      200ms));
  publish_estop(false);

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, true, true);
        return status().has_value() &&
               status()->active_faults == 0U;
      },
      200ms));
  auto reset = call_reset();
  ASSERT_TRUE(reset.has_value());
  EXPECT_EQ(reset->message, "reset rejected: fresh zero command required");

  const geometry_msgs::msg::Twist zero;
  publish_inputs(&zero, true, true);
  const auto stale_deadline = SteadyClock::now() + 160ms;
  while (SteadyClock::now() < stale_deadline) {
    publish_inputs(nullptr, true, true);
    executor_.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  reset = call_reset();
  ASSERT_TRUE(reset.has_value());
  EXPECT_EQ(reset->message, "reset rejected: fresh zero command required");

  maintain_zero_health(30ms);
  reset = call_reset();
  ASSERT_TRUE(reset.has_value());
  EXPECT_EQ(
    reset->message,
    "reset rejected: recovery hold 0.10 s not satisfied");

  maintain_zero_health(120ms);
  reset = call_reset();
  ASSERT_TRUE(reset.has_value());
  EXPECT_TRUE(reset->success);
  EXPECT_EQ(reset->message, "reset accepted");
  ASSERT_TRUE(spin_until(
      [this]() {
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_READY);
      },
      200ms));

  std::this_thread::sleep_for(30ms);
  executor_.spin_some();
  EXPECT_TRUE(state_and_zero(
      diffbot_interfaces::msg::SafetyStatus::STATE_READY));

  ASSERT_TRUE(wait_for_moving_command(moving));
}

TEST_F(SafetyEstopResetTest, RetainedEstopBeforeSupervisorStartupPreventsArming)
{
  destroy_supervisor();
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    last_status_.reset();
    last_safe_sample_.reset();
  }
  publish_estop(true);
  create_supervisor();

  const geometry_msgs::msg::Twist zero;
  ASSERT_TRUE(spin_until(
      [this, &zero]() {
        publish_inputs(&zero, true, true);
        return state_and_zero(
          diffbot_interfaces::msg::SafetyStatus::STATE_ESTOP_LATCHED);
      },
      1s));
  ASSERT_TRUE(status().has_value());
  EXPECT_NE(
    status()->latched_faults &
    diffbot_interfaces::msg::SafetyStatus::FAULT_MANUAL_ESTOP,
    0U);
}

}  // namespace
}  // namespace diffbot_safety
