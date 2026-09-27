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
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "diffbot_safety/safety_policy.hpp"

namespace diffbot_safety
{
namespace
{

using namespace std::chrono_literals;

SafetySnapshot healthy_snapshot()
{
  SafetySnapshot snapshot;
  snapshot.scan_seen = true;
  snapshot.scan_age = 10ms;
  snapshot.odom_seen = true;
  snapshot.odom_age = 10ms;
  snapshot.nav2_active = true;
  return snapshot;
}

TEST(SafetyPolicyTest, InhibitsStartupUntilRequiredInputsAreHealthy)
{
  SafetyPolicy policy;

  const auto initial = policy.evaluate(SafetySnapshot{});
  EXPECT_EQ(initial.state, SafetyState::kStartupInhibit);
  EXPECT_TRUE(initial.output_inhibited);
  EXPECT_TRUE(has_fault(initial.active_faults, Fault::kScanStale));
  EXPECT_TRUE(has_fault(initial.active_faults, Fault::kOdomStale));
  EXPECT_TRUE(has_fault(initial.active_faults, Fault::kNav2Inactive));
  EXPECT_EQ(initial.latched_faults, to_mask(Fault::kNone));

  const auto ready = policy.evaluate(healthy_snapshot());
  EXPECT_EQ(ready.state, SafetyState::kReady);
  EXPECT_TRUE(ready.output_inhibited);
  EXPECT_EQ(ready.active_faults, to_mask(Fault::kNone));
}

TEST(SafetyPolicyTest, PassesCommandsOnlyWhenReady)
{
  SafetyPolicy policy;
  policy.evaluate(healthy_snapshot());

  auto snapshot = healthy_snapshot();
  snapshot.command_seen = true;
  snapshot.command = MotionCommand{0.2, -0.4};
  snapshot.command_age = 20ms;

  const auto decision = policy.evaluate(snapshot);

  EXPECT_EQ(decision.state, SafetyState::kReady);
  EXPECT_FALSE(decision.output_inhibited);
  EXPECT_DOUBLE_EQ(decision.output_command.linear_x, 0.2);
  EXPECT_DOUBLE_EQ(decision.output_command.angular_z, -0.4);
}

TEST(SafetyPolicyTest, RequiresZeroCommandBeforeLeavingStartupInhibit)
{
  SafetyPolicy policy;
  auto snapshot = healthy_snapshot();
  snapshot.command_seen = true;
  snapshot.command = MotionCommand{0.1, 0.0};
  snapshot.command_age = 10ms;

  const auto inhibited = policy.evaluate(snapshot);
  EXPECT_EQ(inhibited.state, SafetyState::kStartupInhibit);
  EXPECT_TRUE(inhibited.output_inhibited);

  snapshot.command = MotionCommand{};
  const auto armed = policy.evaluate(snapshot);
  EXPECT_EQ(armed.state, SafetyState::kReady);
  EXPECT_FALSE(armed.output_inhibited);
}

TEST(SafetyPolicyTest, LatchesSensorFaultUntilExplicitReset)
{
  SafetyPolicy policy;
  auto snapshot = healthy_snapshot();
  policy.evaluate(snapshot);

  snapshot.scan_age = 600ms;
  const auto faulted = policy.evaluate(snapshot);
  EXPECT_EQ(faulted.state, SafetyState::kFaultLatched);
  EXPECT_TRUE(has_fault(faulted.latched_faults, Fault::kScanStale));
  EXPECT_TRUE(faulted.output_inhibited);

  snapshot.scan_age = 10ms;
  const auto recovered_but_latched = policy.evaluate(snapshot);
  EXPECT_EQ(recovered_but_latched.state, SafetyState::kFaultLatched);
  EXPECT_EQ(recovered_but_latched.active_faults, to_mask(Fault::kNone));

  EXPECT_TRUE(policy.reset(snapshot));
  EXPECT_EQ(policy.state(), SafetyState::kReady);
  EXPECT_EQ(policy.latched_faults(), to_mask(Fault::kNone));
}

TEST(SafetyPolicyTest, EstopHasPriorityAndCannotResetWhileAsserted)
{
  SafetyPolicy policy;
  auto snapshot = healthy_snapshot();
  policy.evaluate(snapshot);

  snapshot.emergency_stop = true;
  const auto estopped = policy.evaluate(snapshot);
  EXPECT_EQ(estopped.state, SafetyState::kEstopLatched);
  EXPECT_TRUE(has_fault(estopped.latched_faults, Fault::kManualEstop));
  EXPECT_FALSE(policy.reset(snapshot));

  snapshot.emergency_stop = false;
  EXPECT_TRUE(policy.reset(snapshot));
  EXPECT_EQ(policy.state(), SafetyState::kReady);
}

TEST(SafetyPolicyTest, StaleNonzeroCommandFaultsButStaleZeroIsSafe)
{
  SafetyPolicy moving_policy;
  moving_policy.evaluate(healthy_snapshot());

  auto moving = healthy_snapshot();
  moving.command_seen = true;
  moving.command = MotionCommand{0.1, 0.0};
  moving.command_age = 301ms;

  const auto moving_decision = moving_policy.evaluate(moving);
  EXPECT_EQ(moving_decision.state, SafetyState::kFaultLatched);
  EXPECT_TRUE(has_fault(
    moving_decision.active_faults,
    Fault::kCommandStale));
  EXPECT_TRUE(moving_decision.output_inhibited);

  auto stopped = moving;
  stopped.command = MotionCommand{};
  SafetyPolicy stopped_policy;
  const auto stopped_decision = stopped_policy.evaluate(stopped);
  EXPECT_EQ(stopped_decision.state, SafetyState::kReady);
  EXPECT_FALSE(has_fault(
    stopped_decision.active_faults,
    Fault::kCommandStale));
}

TEST(SafetyPolicyTest, ResetRequiresRecoveredHealthAndZeroCommand)
{
  SafetyConfig config;
  config.odom_timeout = 200ms;
  SafetyPolicy policy(config);
  policy.evaluate(healthy_snapshot());

  auto snapshot = healthy_snapshot();
  snapshot.command_seen = true;
  snapshot.command = MotionCommand{0.1, 0.0};
  snapshot.command_age = 10ms;
  policy.evaluate(snapshot);

  snapshot.odom_age = 300ms;
  policy.evaluate(snapshot);
  EXPECT_FALSE(policy.reset(snapshot));

  snapshot.odom_age = 10ms;
  EXPECT_FALSE(policy.reset(snapshot));

  snapshot.command = MotionCommand{};
  EXPECT_TRUE(policy.reset(snapshot));
}

TEST(SafetyPolicyTest, RejectsInvalidAndOutOfRangeCommands)
{
  auto invalid = healthy_snapshot();
  invalid.command_seen = true;
  invalid.command.linear_x = std::numeric_limits<double>::quiet_NaN();

  SafetyPolicy invalid_policy;
  const auto invalid_decision = invalid_policy.evaluate(invalid);
  EXPECT_EQ(invalid_decision.state, SafetyState::kFaultLatched);
  EXPECT_TRUE(has_fault(
    invalid_decision.latched_faults,
    Fault::kInvalidCommand));

  auto excessive = healthy_snapshot();
  excessive.command_seen = true;
  excessive.command.angular_z = 1.01;

  SafetyPolicy limit_policy;
  const auto limit_decision = limit_policy.evaluate(excessive);
  EXPECT_EQ(limit_decision.state, SafetyState::kFaultLatched);
  EXPECT_TRUE(has_fault(
    limit_decision.latched_faults,
    Fault::kCommandLimitViolation));

  auto unsupported = healthy_snapshot();
  unsupported.command_seen = true;
  unsupported.command.linear_y = 0.01;

  SafetyPolicy differential_drive_policy;
  const auto unsupported_decision =
    differential_drive_policy.evaluate(unsupported);
  EXPECT_EQ(unsupported_decision.state, SafetyState::kFaultLatched);
  EXPECT_TRUE(has_fault(
    unsupported_decision.latched_faults,
    Fault::kCommandLimitViolation));
}

TEST(SafetyPolicyTest, RejectsInvalidConfiguration)
{
  SafetyConfig config;
  config.command_timeout = Duration::zero();
  EXPECT_THROW(SafetyPolicy policy(config), std::invalid_argument);

  config = SafetyConfig{};
  config.max_forward_velocity =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SafetyPolicy policy(config), std::invalid_argument);
}

TEST(SafetyPolicyTest, McuHeartbeatIsOptionalAndCanBeRequired)
{
  const auto snapshot = healthy_snapshot();
  SafetyPolicy default_policy;
  EXPECT_EQ(
    default_policy.evaluate(snapshot).state,
    SafetyState::kReady);

  SafetyConfig config;
  config.require_mcu_heartbeat = true;
  SafetyPolicy hardware_policy(config);
  const auto missing = hardware_policy.evaluate(snapshot);
  EXPECT_EQ(missing.state, SafetyState::kStartupInhibit);
  EXPECT_TRUE(has_fault(
    missing.active_faults,
    Fault::kMcuHeartbeatLost));

  auto heartbeat_snapshot = snapshot;
  heartbeat_snapshot.mcu_heartbeat_seen = true;
  heartbeat_snapshot.mcu_heartbeat_age = 10ms;
  EXPECT_EQ(
    hardware_policy.evaluate(heartbeat_snapshot).state,
    SafetyState::kReady);
}

TEST(SafetyPolicyTest, FormatsStateAndCombinedFaultNames)
{
  EXPECT_EQ(state_name(SafetyState::kReady), "READY");
  const FaultMask faults =
    to_mask(Fault::kScanStale) | to_mask(Fault::kNav2Inactive);
  EXPECT_EQ(fault_mask_to_string(faults), "SCAN_STALE|NAV2_INACTIVE");
}

}  // namespace
}  // namespace diffbot_safety
