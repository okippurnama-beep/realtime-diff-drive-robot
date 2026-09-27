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
#include <cstdint>

#include "gtest/gtest.h"

#include "diffbot_hardware/fake_mcu.hpp"

namespace
{

using namespace std::chrono_literals;
using diffbot_hardware::CommandMode;
using diffbot_hardware::FakeMcu;
using diffbot_hardware::HostCommandFrame;
using diffbot_hardware::McuFault;
using diffbot_hardware::McuMode;

HostCommandFrame command(
  const std::uint32_t session, const std::uint32_t sequence, const CommandMode mode,
  const std::int32_t left = 0, const std::int32_t right = 0)
{
  HostCommandFrame frame;
  frame.host_session_id = session;
  frame.command_sequence = sequence;
  frame.mode = mode;
  frame.left_target_velocity_mrad_s = left;
  frame.right_target_velocity_mrad_s = right;
  return frame;
}

TEST(FakeMcu, RequiresZeroDisarmToClaimSession)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);

  EXPECT_NE(
    mcu.accept_command(command(7U, 1U, CommandMode::kArmed), start),
    diffbot_hardware::FrameValidationError::kNone);
  EXPECT_EQ(mcu.mode(), McuMode::kFault);

  EXPECT_EQ(
    mcu.accept_command(command(7U, 2U, CommandMode::kDisarm), start),
    diffbot_hardware::FrameValidationError::kNone);
  EXPECT_EQ(mcu.mode(), McuMode::kDisarmed);
  EXPECT_EQ(mcu.active_faults(), 0U);
}

TEST(FakeMcu, IntegratesEncoderCountsWhileArmed)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);
  mcu.accept_command(command(7U, 1U, CommandMode::kDisarm), start);
  mcu.accept_command(command(7U, 2U, CommandMode::kArmed, 1000, -1000), start);

  const auto state = mcu.make_state(start + 50ms);
  EXPECT_EQ(state.mode, McuMode::kArmed);
  EXPECT_EQ(state.left_velocity_mrad_s, 1000);
  EXPECT_EQ(state.right_velocity_mrad_s, -1000);
  EXPECT_GT(state.left_encoder_count, 0);
  EXPECT_LT(state.right_encoder_count, 0);
}

TEST(FakeMcu, CommandWatchdogLatchesAndStops)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);
  mcu.accept_command(command(7U, 1U, CommandMode::kDisarm), start);
  mcu.accept_command(command(7U, 2U, CommandMode::kArmed, 1000, 1000), start);

  const auto state = mcu.make_state(start + 100ms);
  EXPECT_EQ(state.mode, McuMode::kFault);
  EXPECT_EQ(state.left_velocity_mrad_s, 0);
  EXPECT_NE(
    state.active_faults & static_cast<std::uint32_t>(McuFault::kCommandTimeout), 0U);
}

TEST(FakeMcu, DuplicateSequenceDoesNotRefreshWatchdog)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);
  mcu.accept_command(command(7U, 1U, CommandMode::kDisarm), start);
  mcu.accept_command(command(7U, 2U, CommandMode::kArmed, 500, 500), start);

  EXPECT_EQ(
    mcu.accept_command(command(7U, 2U, CommandMode::kArmed, 500, 500), start + 20ms),
    diffbot_hardware::FrameValidationError::kCommandSequence);
  const auto state = mcu.make_state(start + 30ms);
  EXPECT_EQ(state.mode, McuMode::kFault);
  EXPECT_NE(
    state.active_faults & static_cast<std::uint32_t>(McuFault::kCommandSequence), 0U);
  EXPECT_EQ(state.last_command_age_ms, 30U);
}

TEST(FakeMcu, WatchdogDoesNotIntegrateMotionPastDeadline)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);
  mcu.accept_command(command(7U, 1U, CommandMode::kDisarm), start);
  mcu.accept_command(command(7U, 2U, CommandMode::kArmed, 1000, 1000), start);

  const auto state = mcu.make_state(start + 500ms);
  EXPECT_EQ(state.mode, McuMode::kFault);
  EXPECT_NEAR(state.left_encoder_count, 33, 1);
  EXPECT_NEAR(state.right_encoder_count, 33, 1);
}

TEST(FakeMcu, RebootChangesBootIdAndInvalidatesSession)
{
  FakeMcu mcu;
  const FakeMcu::TimePoint start{};
  mcu.boot(start);
  mcu.accept_command(command(7U, 1U, CommandMode::kDisarm), start);
  const auto first = mcu.make_state(start);

  mcu.boot(start + 1s);
  const auto second = mcu.make_state(start + 1s);
  EXPECT_NE(second.mcu_boot_id, first.mcu_boot_id);
  EXPECT_EQ(second.accepted_host_session_id, 0U);
  EXPECT_EQ(second.mode, McuMode::kDisarmed);
}

}  // namespace
