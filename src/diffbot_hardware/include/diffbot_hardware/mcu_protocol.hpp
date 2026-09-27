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

#ifndef DIFFBOT_HARDWARE__MCU_PROTOCOL_HPP_
#define DIFFBOT_HARDWARE__MCU_PROTOCOL_HPP_

#include <cstdint>

namespace diffbot_hardware
{

inline constexpr std::uint16_t kProtocolVersion = 1U;
inline constexpr std::uint16_t kDefaultCommandValidityMs = 100U;
inline constexpr std::uint16_t kMaximumCommandValidityMs = 1000U;
inline constexpr std::uint32_t kDefaultStateTimeoutMs = 200U;
inline constexpr std::int32_t kMaximumTargetVelocityMradS = 10000;
inline constexpr std::int32_t kMaximumMeasuredVelocityMradS = 20000;

enum class CommandMode : std::uint8_t
{
  kDisarm = 0U,
  kArmed = 1U,
  kEstop = 2U,
};

enum class McuMode : std::uint8_t
{
  kBoot = 0U,
  kDisarmed = 1U,
  kArmed = 2U,
  kFault = 3U,
  kEstop = 4U,
};

enum class McuFault : std::uint32_t
{
  kNone = 0U,
  kCommandTimeout = 1U << 0U,
  kProtocolMismatch = 1U << 1U,
  kCommandSequence = 1U << 2U,
  kInvalidCommand = 1U << 3U,
  kLeftEncoder = 1U << 4U,
  kRightEncoder = 1U << 5U,
  kLeftDriver = 1U << 6U,
  kRightDriver = 1U << 7U,
  kUndervoltage = 1U << 8U,
  kOvercurrent = 1U << 9U,
  kControlLoopOverrun = 1U << 10U,
  kMcuWatchdogReset = 1U << 11U,
  kEstopInput = 1U << 12U,
};

inline constexpr std::uint32_t kKnownFaultMask =
  (1U << 13U) - 1U;

struct HostCommandFrame
{
  std::uint16_t protocol_version{kProtocolVersion};
  std::uint32_t host_session_id{0U};
  std::uint32_t command_sequence{0U};
  std::uint16_t valid_for_ms{kDefaultCommandValidityMs};
  CommandMode mode{CommandMode::kDisarm};
  std::int32_t left_target_velocity_mrad_s{0};
  std::int32_t right_target_velocity_mrad_s{0};
};

struct McuStateFrame
{
  std::uint16_t protocol_version{kProtocolVersion};
  std::uint32_t mcu_boot_id{0U};
  std::uint32_t state_sequence{0U};
  std::uint32_t accepted_host_session_id{0U};
  std::uint32_t last_accepted_command_sequence{0U};
  std::uint64_t mcu_uptime_ms{0U};
  McuMode mode{McuMode::kBoot};
  std::uint32_t active_faults{0U};
  std::int64_t left_encoder_count{0};
  std::int64_t right_encoder_count{0};
  std::int32_t left_velocity_mrad_s{0};
  std::int32_t right_velocity_mrad_s{0};
  std::uint32_t last_command_age_ms{0U};
  std::uint32_t control_loop_overrun_count{0U};
};

enum class FrameValidationError : std::uint8_t
{
  kNone = 0U,
  kProtocolVersion,
  kSessionId,
  kCommandValidity,
  kCommandMode,
  kNonzeroSafeCommand,
  kTargetVelocity,
  kBootId,
  kMcuMode,
  kMeasuredVelocity,
  kUnknownFault,
};

constexpr bool sequence_is_newer(
  const std::uint32_t candidate, const std::uint32_t reference) noexcept
{
  const auto distance = static_cast<std::uint32_t>(candidate - reference);
  return distance != 0U && distance < 0x80000000U;
}

constexpr FrameValidationError validate_command(
  const HostCommandFrame & frame) noexcept
{
  if (frame.protocol_version != kProtocolVersion) {
    return FrameValidationError::kProtocolVersion;
  }
  if (frame.host_session_id == 0U) {
    return FrameValidationError::kSessionId;
  }
  if (frame.valid_for_ms == 0U || frame.valid_for_ms > kMaximumCommandValidityMs) {
    return FrameValidationError::kCommandValidity;
  }
  if (frame.mode != CommandMode::kDisarm && frame.mode != CommandMode::kArmed &&
    frame.mode != CommandMode::kEstop)
  {
    return FrameValidationError::kCommandMode;
  }
  if (frame.left_target_velocity_mrad_s < -kMaximumTargetVelocityMradS ||
    frame.left_target_velocity_mrad_s > kMaximumTargetVelocityMradS ||
    frame.right_target_velocity_mrad_s < -kMaximumTargetVelocityMradS ||
    frame.right_target_velocity_mrad_s > kMaximumTargetVelocityMradS)
  {
    return FrameValidationError::kTargetVelocity;
  }
  if (frame.mode != CommandMode::kArmed &&
    (frame.left_target_velocity_mrad_s != 0 || frame.right_target_velocity_mrad_s != 0))
  {
    return FrameValidationError::kNonzeroSafeCommand;
  }
  return FrameValidationError::kNone;
}

constexpr FrameValidationError validate_state(const McuStateFrame & frame) noexcept
{
  if (frame.protocol_version != kProtocolVersion) {
    return FrameValidationError::kProtocolVersion;
  }
  if (frame.mcu_boot_id == 0U) {
    return FrameValidationError::kBootId;
  }
  if (frame.mode != McuMode::kBoot && frame.mode != McuMode::kDisarmed &&
    frame.mode != McuMode::kArmed && frame.mode != McuMode::kFault &&
    frame.mode != McuMode::kEstop)
  {
    return FrameValidationError::kMcuMode;
  }
  if (frame.mode == McuMode::kArmed && frame.accepted_host_session_id == 0U) {
    return FrameValidationError::kSessionId;
  }
  if (frame.left_velocity_mrad_s < -kMaximumMeasuredVelocityMradS ||
    frame.left_velocity_mrad_s > kMaximumMeasuredVelocityMradS ||
    frame.right_velocity_mrad_s < -kMaximumMeasuredVelocityMradS ||
    frame.right_velocity_mrad_s > kMaximumMeasuredVelocityMradS)
  {
    return FrameValidationError::kMeasuredVelocity;
  }
  if ((frame.active_faults & ~kKnownFaultMask) != 0U) {
    return FrameValidationError::kUnknownFault;
  }
  return FrameValidationError::kNone;
}

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__MCU_PROTOCOL_HPP_
