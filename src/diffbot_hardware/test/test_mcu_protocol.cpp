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

#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

#include "diffbot_hardware/mcu_protocol.hpp"
#include "diffbot_interfaces/msg/mcu_command.hpp"
#include "diffbot_interfaces/msg/mcu_state.hpp"

namespace
{

using diffbot_hardware::CommandMode;
using diffbot_hardware::FrameValidationError;
using diffbot_hardware::HostCommandFrame;
using diffbot_hardware::McuMode;
using diffbot_hardware::McuStateFrame;

TEST(McuProtocol, RosMessagesMatchPureCppContract)
{
  using McuCommand = diffbot_interfaces::msg::McuCommand;
  using McuState = diffbot_interfaces::msg::McuState;

  EXPECT_EQ(McuCommand::PROTOCOL_VERSION_CURRENT, diffbot_hardware::kProtocolVersion);
  EXPECT_EQ(McuCommand::MODE_DISARM, static_cast<std::uint8_t>(CommandMode::kDisarm));
  EXPECT_EQ(McuCommand::MODE_ARMED, static_cast<std::uint8_t>(CommandMode::kArmed));
  EXPECT_EQ(McuCommand::MODE_ESTOP, static_cast<std::uint8_t>(CommandMode::kEstop));

  EXPECT_EQ(McuState::PROTOCOL_VERSION_CURRENT, diffbot_hardware::kProtocolVersion);
  EXPECT_EQ(McuState::MODE_BOOT, static_cast<std::uint8_t>(McuMode::kBoot));
  EXPECT_EQ(McuState::MODE_DISARMED, static_cast<std::uint8_t>(McuMode::kDisarmed));
  EXPECT_EQ(McuState::MODE_ARMED, static_cast<std::uint8_t>(McuMode::kArmed));
  EXPECT_EQ(McuState::MODE_FAULT, static_cast<std::uint8_t>(McuMode::kFault));
  EXPECT_EQ(McuState::MODE_ESTOP, static_cast<std::uint8_t>(McuMode::kEstop));
  EXPECT_EQ(McuState::FAULT_ESTOP_INPUT, 1U << 12U);
  EXPECT_EQ(diffbot_hardware::kKnownFaultMask, (1U << 13U) - 1U);
}

TEST(McuProtocol, AcceptsValidArmedCommand)
{
  HostCommandFrame command;
  command.host_session_id = 17U;
  command.command_sequence = 42U;
  command.mode = CommandMode::kArmed;
  command.left_target_velocity_mrad_s = 1500;
  command.right_target_velocity_mrad_s = -1500;

  EXPECT_EQ(diffbot_hardware::validate_command(command), FrameValidationError::kNone);
}

TEST(McuProtocol, RejectsInvalidCommandEnvelope)
{
  HostCommandFrame command;
  command.host_session_id = 1U;

  command.protocol_version = 2U;
  EXPECT_EQ(
    diffbot_hardware::validate_command(command), FrameValidationError::kProtocolVersion);

  command.protocol_version = diffbot_hardware::kProtocolVersion;
  command.valid_for_ms = 0U;
  EXPECT_EQ(
    diffbot_hardware::validate_command(command), FrameValidationError::kCommandValidity);

  command.valid_for_ms = diffbot_hardware::kDefaultCommandValidityMs;
  command.left_target_velocity_mrad_s = 1;
  EXPECT_EQ(
    diffbot_hardware::validate_command(command), FrameValidationError::kNonzeroSafeCommand);

  command.mode = CommandMode::kArmed;
  command.left_target_velocity_mrad_s =
    diffbot_hardware::kMaximumTargetVelocityMradS + 1;
  EXPECT_EQ(
    diffbot_hardware::validate_command(command), FrameValidationError::kTargetVelocity);
}

TEST(McuProtocol, RequiresSessionAndRecognizedMode)
{
  HostCommandFrame command;
  EXPECT_EQ(diffbot_hardware::validate_command(command), FrameValidationError::kSessionId);

  command.host_session_id = 1U;
  command.mode = static_cast<CommandMode>(99U);
  EXPECT_EQ(diffbot_hardware::validate_command(command), FrameValidationError::kCommandMode);
}

TEST(McuProtocol, ComparesSequencesAcrossWraparound)
{
  EXPECT_TRUE(diffbot_hardware::sequence_is_newer(11U, 10U));
  EXPECT_FALSE(diffbot_hardware::sequence_is_newer(10U, 10U));
  EXPECT_FALSE(diffbot_hardware::sequence_is_newer(9U, 10U));
  EXPECT_TRUE(
    diffbot_hardware::sequence_is_newer(0U, std::numeric_limits<std::uint32_t>::max()));
  EXPECT_FALSE(
    diffbot_hardware::sequence_is_newer(std::numeric_limits<std::uint32_t>::max(), 0U));
}

TEST(McuProtocol, AcceptsValidArmedState)
{
  McuStateFrame state;
  state.mcu_boot_id = 9U;
  state.accepted_host_session_id = 17U;
  state.mode = McuMode::kArmed;
  state.left_velocity_mrad_s = 1000;
  state.right_velocity_mrad_s = 1001;

  EXPECT_EQ(diffbot_hardware::validate_state(state), FrameValidationError::kNone);
}

TEST(McuProtocol, RejectsUnsafeStateMetadata)
{
  McuStateFrame state;
  state.mcu_boot_id = 1U;
  state.mode = McuMode::kArmed;
  EXPECT_EQ(diffbot_hardware::validate_state(state), FrameValidationError::kSessionId);

  state.accepted_host_session_id = 2U;
  state.active_faults = 1U << 31U;
  EXPECT_EQ(diffbot_hardware::validate_state(state), FrameValidationError::kUnknownFault);

  state.active_faults = 0U;
  state.right_velocity_mrad_s = diffbot_hardware::kMaximumMeasuredVelocityMradS + 1;
  EXPECT_EQ(
    diffbot_hardware::validate_state(state), FrameValidationError::kMeasuredVelocity);
}

TEST(McuProtocol, RejectsUnknownStateModeAndMissingBootId)
{
  McuStateFrame state;
  EXPECT_EQ(diffbot_hardware::validate_state(state), FrameValidationError::kBootId);

  state.mcu_boot_id = 1U;
  state.mode = static_cast<McuMode>(99U);
  EXPECT_EQ(diffbot_hardware::validate_state(state), FrameValidationError::kMcuMode);
}

}  // namespace
