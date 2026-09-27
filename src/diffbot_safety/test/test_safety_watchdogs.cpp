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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diffbot_interfaces/msg/safety_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "diffbot_safety/safety_supervisor_node.hpp"

namespace diffbot_safety
{
namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

class SafetyWatchdogTest : public ::testing::Test
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
    driver_ = std::make_shared<rclcpp::Node>("m7_3_watchdog_driver");
    localization_manager_ = std::make_shared<rclcpp::Node>(
      "m7_3_localization_manager");
    navigation_manager_ = std::make_shared<rclcpp::Node>(
      "m7_3_navigation_manager");

    localization_service_ =
      localization_manager_->create_service<std_srvs::srv::Trigger>(
      "/m7_3_test/localization/is_active",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = localization_active_.load();
        response->message = response->success ? "active" : "inactive";
      });
    navigation_service_ =
      navigation_manager_->create_service<std_srvs::srv::Trigger>(
      "/m7_3_test/navigation/is_active",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = navigation_active_.load();
        response->message = response->success ? "active" : "inactive";
      });

    rclcpp::NodeOptions options;
    options.parameter_overrides({
          rclcpp::Parameter("control_frequency_hz", 100.0),
          rclcpp::Parameter("status_frequency_hz", 100.0),
          rclcpp::Parameter("command_timeout_sec", 0.08),
          rclcpp::Parameter("scan_timeout_sec", 0.08),
          rclcpp::Parameter("odom_timeout_sec", 0.08),
          rclcpp::Parameter("nav2_poll_period_sec", 0.02),
          rclcpp::Parameter("nav2_health_timeout_sec", 0.10),
          rclcpp::Parameter("require_nav2_active", true),
          rclcpp::Parameter("input_command_topic", "/m7_3_test/cmd_in"),
          rclcpp::Parameter("scan_topic", "/m7_3_test/scan"),
          rclcpp::Parameter("odom_topic", "/m7_3_test/odom"),
          rclcpp::Parameter("output_command_topic", "/m7_3_test/cmd_safe"),
          rclcpp::Parameter("status_topic", "/m7_3_test/status"),
          rclcpp::Parameter("diagnostics_topic", "/m7_3_test/diagnostics"),
          rclcpp::Parameter(
            "localization_manager_service",
            "/m7_3_test/localization/is_active"),
          rclcpp::Parameter(
            "navigation_manager_service",
            "/m7_3_test/navigation/is_active"),
    });
    supervisor_ = std::make_shared<SafetySupervisorNode>(options);

    command_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>(
      "/m7_3_test/cmd_in", rclcpp::QoS(10).reliable());
    scan_pub_ = driver_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/m7_3_test/scan", rclcpp::SensorDataQoS());
    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/m7_3_test/odom", rclcpp::QoS(10).reliable());

    safe_sub_ = driver_->create_subscription<geometry_msgs::msg::Twist>(
      "/m7_3_test/cmd_safe",
      rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_safe_sample_ = SafeSample{*message, SteadyClock::now()};
      });
    status_sub_ =
      driver_->create_subscription<diffbot_interfaces::msg::SafetyStatus>(
      "/m7_3_test/status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const diffbot_interfaces::msg::SafetyStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_status_ = *message;
      });
    diagnostics_sub_ =
      driver_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/m7_3_test/diagnostics",
      rclcpp::QoS(10).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        last_diagnostics_ = *message;
      });

    executor_.add_node(localization_manager_);
    executor_.add_node(navigation_manager_);
    executor_.add_node(supervisor_);
    executor_.add_node(driver_);
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(supervisor_);
    if (navigation_manager_) {
      executor_.remove_node(navigation_manager_);
    }
    executor_.remove_node(localization_manager_);

    diagnostics_sub_.reset();
    status_sub_.reset();
    safe_sub_.reset();
    odom_pub_.reset();
    scan_pub_.reset();
    command_pub_.reset();
    navigation_service_.reset();
    localization_service_.reset();
    driver_.reset();
    supervisor_.reset();
    navigation_manager_.reset();
    localization_manager_.reset();
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

  std::optional<diagnostic_msgs::msg::DiagnosticArray> diagnostics()
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_diagnostics_;
  }

  bool fault_and_zero(const std::uint32_t fault)
  {
    const auto current_status = status();
    const auto current_safe = safe_sample();
    return current_status.has_value() && current_safe.has_value() &&
           current_status->state ==
           diffbot_interfaces::msg::SafetyStatus::STATE_FAULT_LATCHED &&
           (current_status->latched_faults & fault) != 0U &&
           std::abs(current_safe->command.linear.x) < 1.0e-12 &&
           std::abs(current_safe->command.angular.z) < 1.0e-12;
  }

  std::optional<std::string> diagnostic_value(const std::string & key)
  {
    const auto current = diagnostics();
    if (!current.has_value() || current->status.empty()) {
      return std::nullopt;
    }
    for (const auto & value : current->status.front().values) {
      if (value.key == key) {
        return value.value;
      }
    }
    return std::nullopt;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Node::SharedPtr localization_manager_;
  rclcpp::Node::SharedPtr navigation_manager_;
  std::shared_ptr<SafetySupervisorNode> supervisor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localization_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr navigation_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_sub_;
  rclcpp::Subscription<diffbot_interfaces::msg::SafetyStatus>::SharedPtr
    status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_sub_;
  std::atomic<bool> localization_active_{true};
  std::atomic<bool> navigation_active_{true};
  std::mutex result_mutex_;
  std::optional<SafeSample> last_safe_sample_;
  std::optional<diffbot_interfaces::msg::SafetyStatus> last_status_;
  std::optional<diagnostic_msgs::msg::DiagnosticArray> last_diagnostics_;
};

TEST_F(SafetyWatchdogTest, HealthyManagersPermitReadyAndOkDiagnostics)
{
  ASSERT_TRUE(wait_for_ready());
  ASSERT_TRUE(spin_until(
      [this]() {
        const auto current = diagnostics();
        return current.has_value() && !current->status.empty() &&
               current->status.front().level ==
               diagnostic_msgs::msg::DiagnosticStatus::OK;
      },
      1s));
  EXPECT_EQ(diagnostic_value("state"), "READY");
  EXPECT_EQ(diagnostic_value("localization_manager_active"), "true");
  EXPECT_EQ(diagnostic_value("navigation_manager_active"), "true");
}

TEST_F(SafetyWatchdogTest, StaleNonzeroCommandLatchesAndPublishesZero)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  const auto loss_started = SteadyClock::now();
  ASSERT_TRUE(spin_until(
      [this]() {
        publish_inputs(nullptr, true, true);
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_COMMAND_STALE);
      },
      300ms));
  EXPECT_LT(SteadyClock::now() - loss_started, 200ms);
}

TEST_F(SafetyWatchdogTest, StaleScanLatchesAndPublishesZero)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, false, true);
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_SCAN_STALE);
      },
      300ms));
}

TEST_F(SafetyWatchdogTest, StaleOdometryLatchesAndPublishesZero)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, true, false);
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_ODOM_STALE);
      },
      300ms));
}

TEST_F(SafetyWatchdogTest, InactiveManagerLatchesNav2FaultAndErrorDiagnostic)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));
  navigation_active_.store(false);

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, true, true);
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_NAV2_INACTIVE);
      },
      400ms));
  ASSERT_TRUE(spin_until(
      [this]() {
        const auto current = diagnostics();
        return current.has_value() && !current->status.empty() &&
               current->status.front().level ==
               diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      },
      200ms));
  EXPECT_EQ(diagnostic_value("navigation_manager_active"), "false");
  EXPECT_EQ(diagnostic_value("active_faults"), "NAV2_INACTIVE");
}

TEST_F(SafetyWatchdogTest, MissingManagerServiceBecomesStale)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  executor_.remove_node(navigation_manager_);
  navigation_service_.reset();
  navigation_manager_.reset();
  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, true, true);
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_NAV2_INACTIVE);
      },
      500ms));
}

TEST_F(SafetyWatchdogTest, SimultaneousSensorLossPreservesBothFaultBits)
{
  ASSERT_TRUE(wait_for_ready());
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.15;
  ASSERT_TRUE(wait_for_moving_command(moving));

  ASSERT_TRUE(spin_until(
      [this, &moving]() {
        publish_inputs(&moving, false, false);
        const auto current = status();
        return fault_and_zero(
          diffbot_interfaces::msg::SafetyStatus::FAULT_SCAN_STALE) &&
               current.has_value() &&
               (current->latched_faults &
               diffbot_interfaces::msg::SafetyStatus::FAULT_ODOM_STALE) != 0U;
      },
      300ms));
}

}  // namespace
}  // namespace diffbot_safety
