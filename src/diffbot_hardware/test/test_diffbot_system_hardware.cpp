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
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffbot_hardware/diffbot_system_hardware.hpp"

namespace
{

using diffbot_hardware::CommandMode;
using diffbot_hardware::DiffbotSystemHardware;
using diffbot_hardware::HostCommandFrame;
using diffbot_hardware::McuMode;
using diffbot_hardware::McuStateFrame;
using diffbot_hardware::McuTransport;
using diffbot_hardware::ReceivedMcuState;
using diffbot_hardware::TransportConfig;
using diffbot_hardware::TransportResult;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class ManualClock
{
public:
  DiffbotSystemHardware::TimePoint now() const noexcept {return now_;}

  void advance(const std::chrono::milliseconds amount) noexcept {now_ += amount;}

private:
  DiffbotSystemHardware::TimePoint now_{};
};

class ScriptedTransport final : public McuTransport
{
public:
  explicit ScriptedTransport(ManualClock & clock)
  : clock_(clock)
  {
    state_.mcu_boot_id = 41U;
    state_.mode = McuMode::kDisarmed;
  }

  TransportResult configure(const TransportConfig & config) noexcept override
  {
    config_ = config;
    configured_ = config.host_session_id != 0U;
    return configured_ ? TransportResult::kOk : TransportResult::kProtocolError;
  }

  TransportResult activate() noexcept override
  {
    active_ = configured_;
    return active_ ? TransportResult::kOk : TransportResult::kProtocolError;
  }

  TransportResult send_command(const HostCommandFrame & command) noexcept override
  {
    if (!active_) {
      return TransportResult::kDisconnected;
    }
    last_command_ = command;
    ++sent_count_;
    state_.accepted_host_session_id = command.host_session_id;
    state_.last_accepted_command_sequence = command.command_sequence;
    state_.mode = command.mode == CommandMode::kArmed ? McuMode::kArmed : McuMode::kDisarmed;
    return TransportResult::kOk;
  }

  TransportResult receive_latest(ReceivedMcuState & received) noexcept override
  {
    if (!active_) {
      return TransportResult::kDisconnected;
    }
    if (drop_states_) {
      return TransportResult::kNoData;
    }
    ++state_.state_sequence;
    received.frame = state_;
    received.received_at = clock_.now();
    return TransportResult::kOk;
  }

  void deactivate() noexcept override {active_ = false;}

  void set_feedback(
    const std::int64_t left_count, const std::int64_t right_count,
    const std::int32_t left_velocity, const std::int32_t right_velocity) noexcept
  {
    state_.left_encoder_count = left_count;
    state_.right_encoder_count = right_count;
    state_.left_velocity_mrad_s = left_velocity;
    state_.right_velocity_mrad_s = right_velocity;
  }

  void reboot() noexcept
  {
    ++state_.mcu_boot_id;
    state_.state_sequence = 0U;
    state_.accepted_host_session_id = 0U;
    state_.mode = McuMode::kDisarmed;
  }

  void set_drop_states(const bool enabled) noexcept {drop_states_ = enabled;}

  const HostCommandFrame & last_command() const noexcept {return last_command_;}
  std::uint32_t sent_count() const noexcept {return sent_count_;}

private:
  ManualClock & clock_;
  TransportConfig config_{};
  McuStateFrame state_{};
  HostCommandFrame last_command_{};
  std::uint32_t sent_count_{0U};
  bool configured_{false};
  bool active_{false};
  bool drop_states_{false};
};

hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo value{};
  value.name = name;
  value.data_type = "double";
  return value;
}

hardware_interface::ComponentInfo wheel(const std::string & name)
{
  hardware_interface::ComponentInfo value{};
  value.name = name;
  value.type = "joint";
  value.command_interfaces = {interface("velocity")};
  value.state_interfaces = {interface("position"), interface("velocity")};
  return value;
}

hardware_interface::HardwareInfo hardware_info()
{
  hardware_interface::HardwareInfo value{};
  value.name = "DiffbotMcuSystem";
  value.type = "system";
  value.hardware_plugin_name = "diffbot_hardware/DiffbotSystemHardware";
  value.hardware_parameters["encoder_counts_per_revolution"] = "2048";
  value.hardware_parameters["command_validity_ms"] = "100";
  value.hardware_parameters["state_timeout_ms"] = "20";
  value.hardware_parameters["activation_timeout_ms"] = "50";
  value.joints = {wheel("left_wheel_joint"), wheel("right_wheel_joint")};
  return value;
}

hardware_interface::HardwareComponentParams component_params(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = info;
  params.logger = rclcpp::get_logger("test_diffbot_system_hardware");
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return params;
}

class HardwareFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto transport = std::make_unique<ScriptedTransport>(clock_);
    transport_ = transport.get();
    hardware_ = std::make_unique<DiffbotSystemHardware>(
      std::move(transport), [this]() noexcept {return clock_.now();});
    const auto info = hardware_info();
    ASSERT_EQ(hardware_->init(component_params(info)), CallbackReturn::SUCCESS);
    static_cast<void>(hardware_->on_export_state_interfaces());
    static_cast<void>(hardware_->on_export_command_interfaces());
    ASSERT_EQ(hardware_->on_configure(state_), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_->on_activate(state_), CallbackReturn::SUCCESS);
  }

  ManualClock clock_;
  ScriptedTransport * transport_{nullptr};
  std::unique_ptr<DiffbotSystemHardware> hardware_;
  rclcpp_lifecycle::State state_;
};

TEST(DiffbotSystemHardware, RejectsInvalidJointContract)
{
  ManualClock clock;
  auto transport = std::make_unique<ScriptedTransport>(clock);
  DiffbotSystemHardware hardware(
    std::move(transport), [&clock]() noexcept {return clock.now();});
  auto info = hardware_info();
  info.joints[1].state_interfaces.pop_back();
  EXPECT_EQ(hardware.init(component_params(info)), CallbackReturn::ERROR);
}

TEST(DiffbotSystemHardware, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance(
    "diffbot_hardware/DiffbotSystemHardware");
  ASSERT_NE(instance, nullptr);
}

TEST(DiffbotSystemHardware, RunsThroughResourceManagerLifecycle)
{
  const std::string urdf =
    R"(
<robot name="diffbot_m8_test">
  <link name="base_link"/>
  <link name="left_wheel_link"/>
  <link name="right_wheel_link"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  <ros2_control name="DiffbotMcuSystem" type="system">
    <hardware>
      <plugin>diffbot_hardware/DiffbotSystemHardware</plugin>
      <param name="encoder_counts_per_revolution">2048</param>
      <param name="command_validity_ms">100</param>
      <param name="state_timeout_ms">200</param>
      <param name="activation_timeout_ms">100</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>)";

  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager manager(
    urdf, clock, rclcpp::get_logger("test_diffbot_resource_manager"), false, 100U);
  ASSERT_TRUE(manager.are_components_initialized());
  EXPECT_TRUE(manager.command_interface_exists("left_wheel_joint/velocity"));
  EXPECT_TRUE(manager.state_interface_exists("right_wheel_joint/position"));

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("DiffbotMcuSystem", active_state), return_type::OK);

  auto left_command = manager.claim_command_interface("left_wheel_joint/velocity");
  auto right_command = manager.claim_command_interface("right_wheel_joint/velocity");
  ASSERT_TRUE(left_command.set_value(1.0));
  ASSERT_TRUE(right_command.set_value(-0.5));
  EXPECT_EQ(
    manager.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)).result,
    return_type::OK);

  std::this_thread::sleep_for(std::chrono::milliseconds{15});
  EXPECT_EQ(
    manager.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)).result,
    return_type::OK);
  auto left_velocity = manager.claim_state_interface("left_wheel_joint/velocity");
  auto right_velocity = manager.claim_state_interface("right_wheel_joint/velocity");
  ASSERT_TRUE(left_velocity.get_optional<double>().has_value());
  ASSERT_TRUE(right_velocity.get_optional<double>().has_value());
  EXPECT_DOUBLE_EQ(left_velocity.get_optional<double>().value(), 1.0);
  EXPECT_DOUBLE_EQ(right_velocity.get_optional<double>().value(), -0.5);
}

TEST_F(HardwareFixture, ActivationPerformsZeroDisarmAndArmHandshake)
{
  EXPECT_EQ(transport_->sent_count(), 2U);
  EXPECT_EQ(transport_->last_command().mode, CommandMode::kArmed);
  EXPECT_EQ(transport_->last_command().left_target_velocity_mrad_s, 0);
  EXPECT_EQ(transport_->last_command().right_target_velocity_mrad_s, 0);
  EXPECT_NE(transport_->last_command().host_session_id, 0U);
}

TEST_F(HardwareFixture, ConvertsWheelCommandsAndFeedback)
{
  hardware_->set_command("left_wheel_joint/velocity", 1.25);
  hardware_->set_command("right_wheel_joint/velocity", -0.5);
  ASSERT_EQ(
    hardware_->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::OK);
  EXPECT_EQ(transport_->last_command().left_target_velocity_mrad_s, 1250);
  EXPECT_EQ(transport_->last_command().right_target_velocity_mrad_s, -500);

  transport_->set_feedback(2048, -1024, 1250, -500);
  ASSERT_EQ(
    hardware_->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::OK);
  EXPECT_NEAR(hardware_->get_state<double>("left_wheel_joint/position"), 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(hardware_->get_state<double>("right_wheel_joint/position"), -M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(hardware_->get_state<double>("left_wheel_joint/velocity"), 1.25);
  EXPECT_DOUBLE_EQ(hardware_->get_state<double>("right_wheel_joint/velocity"), -0.5);
}

TEST_F(HardwareFixture, RejectsNonFiniteAndOutOfRangeCommands)
{
  hardware_->set_command(
    "left_wheel_joint/velocity", std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(
    hardware_->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::ERROR);

  hardware_->set_command("left_wheel_joint/velocity", 10.001);
  EXPECT_EQ(
    hardware_->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::ERROR);
}

TEST_F(HardwareFixture, RejectsStaleFeedback)
{
  transport_->set_drop_states(true);
  clock_.advance(std::chrono::milliseconds{21});
  EXPECT_EQ(
    hardware_->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::ERROR);
}

TEST_F(HardwareFixture, DetectsMcuReboot)
{
  transport_->reboot();
  EXPECT_EQ(
    hardware_->read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
    return_type::ERROR);
}

TEST_F(HardwareFixture, DeactivationSendsZeroDisarm)
{
  ASSERT_EQ(hardware_->on_deactivate(state_), CallbackReturn::SUCCESS);
  EXPECT_EQ(transport_->last_command().mode, CommandMode::kDisarm);
  EXPECT_EQ(transport_->last_command().left_target_velocity_mrad_s, 0);
  EXPECT_EQ(transport_->last_command().right_target_velocity_mrad_s, 0);
}

}  // namespace
