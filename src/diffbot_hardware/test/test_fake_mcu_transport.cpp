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

#include "diffbot_hardware/fake_mcu_transport.hpp"

namespace
{

using namespace std::chrono_literals;
using diffbot_hardware::CommandMode;
using diffbot_hardware::FakeMcuTransport;
using diffbot_hardware::HostCommandFrame;
using diffbot_hardware::McuFault;
using diffbot_hardware::McuMode;
using diffbot_hardware::ReceivedMcuState;
using diffbot_hardware::TransportConfig;
using diffbot_hardware::TransportResult;

class ManualClock
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  TimePoint now() const noexcept {return now_;}
  void advance(const std::chrono::milliseconds amount) noexcept {now_ += amount;}

private:
  TimePoint now_{};
};

HostCommandFrame command(
  const std::uint32_t session, const std::uint32_t sequence, const CommandMode mode,
  const std::int32_t velocity = 0)
{
  HostCommandFrame frame;
  frame.host_session_id = session;
  frame.command_sequence = sequence;
  frame.mode = mode;
  frame.left_target_velocity_mrad_s = velocity;
  frame.right_target_velocity_mrad_s = velocity;
  return frame;
}

TransportConfig config(const std::uint32_t session = 7U)
{
  TransportConfig value;
  value.host_session_id = session;
  return value;
}

TEST(FakeMcuTransport, RequiresConfigurationBeforeActivation)
{
  ManualClock clock;
  FakeMcuTransport transport([&clock]() noexcept {return clock.now();});
  EXPECT_EQ(transport.activate(), TransportResult::kProtocolError);
  EXPECT_EQ(transport.configure(config(0U)), TransportResult::kProtocolError);
  EXPECT_EQ(transport.configure(config()), TransportResult::kOk);
  EXPECT_EQ(transport.activate(), TransportResult::kOk);
}

TEST(FakeMcuTransport, ExecutesSessionHandshakeAndReturnsState)
{
  ManualClock clock;
  FakeMcuTransport transport([&clock]() noexcept {return clock.now();});
  ASSERT_EQ(transport.configure(config()), TransportResult::kOk);
  ASSERT_EQ(transport.activate(), TransportResult::kOk);
  ASSERT_EQ(transport.send_command(command(7U, 1U, CommandMode::kDisarm)), TransportResult::kOk);

  ReceivedMcuState state;
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_EQ(state.frame.accepted_host_session_id, 7U);
  EXPECT_EQ(state.frame.mode, McuMode::kDisarmed);

  clock.advance(10ms);
  ASSERT_EQ(
    transport.send_command(command(7U, 2U, CommandMode::kArmed, 1000)),
    TransportResult::kOk);
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_EQ(state.frame.mode, McuMode::kArmed);
  EXPECT_EQ(state.frame.left_velocity_mrad_s, 1000);
}

TEST(FakeMcuTransport, DroppedCommandsTriggerMcuWatchdog)
{
  ManualClock clock;
  FakeMcuTransport transport([&clock]() noexcept {return clock.now();});
  ASSERT_EQ(transport.configure(config()), TransportResult::kOk);
  ASSERT_EQ(transport.activate(), TransportResult::kOk);
  ASSERT_EQ(transport.send_command(command(7U, 1U, CommandMode::kDisarm)), TransportResult::kOk);
  ReceivedMcuState state;
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);

  clock.advance(10ms);
  ASSERT_EQ(
    transport.send_command(command(7U, 2U, CommandMode::kArmed, 1000)),
    TransportResult::kOk);
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  transport.set_drop_commands(true);

  clock.advance(100ms);
  ASSERT_EQ(
    transport.send_command(command(7U, 3U, CommandMode::kArmed, 1000)),
    TransportResult::kOk);
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_EQ(state.frame.mode, McuMode::kFault);
  EXPECT_NE(
    state.frame.active_faults & static_cast<std::uint32_t>(McuFault::kCommandTimeout), 0U);
}

TEST(FakeMcuTransport, CommandDelayIsDeterministic)
{
  ManualClock clock;
  FakeMcuTransport transport([&clock]() noexcept {return clock.now();});
  ASSERT_EQ(transport.configure(config()), TransportResult::kOk);
  ASSERT_EQ(transport.activate(), TransportResult::kOk);
  transport.set_command_delay(30ms);
  ASSERT_EQ(transport.send_command(command(7U, 1U, CommandMode::kDisarm)), TransportResult::kOk);

  ReceivedMcuState state;
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_EQ(state.frame.accepted_host_session_id, 0U);

  clock.advance(30ms);
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_EQ(state.frame.accepted_host_session_id, 7U);
}

TEST(FakeMcuTransport, StateDropAndRebootAreObservable)
{
  ManualClock clock;
  FakeMcuTransport transport([&clock]() noexcept {return clock.now();});
  ASSERT_EQ(transport.configure(config()), TransportResult::kOk);
  ASSERT_EQ(transport.activate(), TransportResult::kOk);
  ReceivedMcuState state;
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  const auto first_boot_id = state.frame.mcu_boot_id;

  transport.set_drop_states(true);
  clock.advance(10ms);
  EXPECT_EQ(transport.receive_latest(state), TransportResult::kNoData);

  transport.set_drop_states(false);
  transport.reboot();
  ASSERT_EQ(transport.receive_latest(state), TransportResult::kOk);
  EXPECT_NE(state.frame.mcu_boot_id, first_boot_id);
  EXPECT_EQ(state.frame.accepted_host_session_id, 0U);
}

}  // namespace
