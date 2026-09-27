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

#include "diffbot_hardware/fake_mcu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace diffbot_hardware
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

std::uint32_t fault_bit(const McuFault fault) noexcept
{
  return static_cast<std::uint32_t>(fault);
}

}  // namespace

FakeMcu::FakeMcu(FakeMcuConfig config)
: config_(config)
{
  if (config_.encoder_counts_per_revolution == 0U ||
    config_.maximum_velocity_mrad_s <= 0 ||
    config_.maximum_velocity_mrad_s > kMaximumMeasuredVelocityMradS)
  {
    throw std::invalid_argument("invalid fake MCU configuration");
  }
}

void FakeMcu::boot(const TimePoint now) noexcept
{
  ++boot_id_;
  if (boot_id_ == 0U) {
    boot_id_ = 1U;
  }
  state_sequence_ = 0U;
  accepted_host_session_id_ = 0U;
  last_accepted_command_sequence_ = 0U;
  active_faults_ = 0U;
  mode_ = McuMode::kBoot;
  left_target_velocity_mrad_s_ = 0;
  right_target_velocity_mrad_s_ = 0;
  left_encoder_count_ = 0.0;
  right_encoder_count_ = 0.0;
  command_received_ = false;
  boot_time_ = now;
  last_step_time_ = now;
  last_command_time_ = now;
  command_deadline_ = now;
}

FrameValidationError FakeMcu::accept_command(
  const HostCommandFrame & command, const TimePoint now) noexcept
{
  step(now);
  const auto validation = validate_command(command);
  if (validation != FrameValidationError::kNone) {
    latch_validation_fault(validation);
    return validation;
  }

  const bool new_session = command.host_session_id != accepted_host_session_id_;
  const bool stale_sequence = !new_session && command_received_ &&
    !sequence_is_newer(command.command_sequence, last_accepted_command_sequence_);
  if (new_session) {
    if (command.mode != CommandMode::kDisarm) {
      latch_validation_fault(FrameValidationError::kSessionId);
      return FrameValidationError::kSessionId;
    }
    accepted_host_session_id_ = command.host_session_id;
    active_faults_ &= ~kRecoverableCommunicationFaults;
    command_received_ = false;
    stop(McuMode::kDisarmed);
  }
  if (stale_sequence) {
    active_faults_ |= fault_bit(McuFault::kCommandSequence);
    stop(McuMode::kFault);
    return FrameValidationError::kCommandSequence;
  }

  last_accepted_command_sequence_ = command.command_sequence;
  last_command_time_ = now;
  command_deadline_ = now + std::chrono::milliseconds(command.valid_for_ms);
  command_received_ = true;

  if (command.mode == CommandMode::kEstop) {
    stop(McuMode::kEstop);
  } else if (command.mode == CommandMode::kDisarm) {
    stop(active_faults_ == 0U ? McuMode::kDisarmed : McuMode::kFault);
  } else if (active_faults_ == 0U) {
    mode_ = McuMode::kArmed;
    left_target_velocity_mrad_s_ = command.left_target_velocity_mrad_s;
    right_target_velocity_mrad_s_ = command.right_target_velocity_mrad_s;
  } else {
    stop(McuMode::kFault);
  }

  return FrameValidationError::kNone;
}

void FakeMcu::step(const TimePoint now) noexcept
{
  if (mode_ == McuMode::kBoot) {
    mode_ = McuMode::kDisarmed;
  }
  if (now <= last_step_time_) {
    return;
  }

  auto motion_end = now;
  if (mode_ == McuMode::kArmed && command_received_ && command_deadline_ < motion_end) {
    motion_end = command_deadline_;
  }
  const auto elapsed = std::chrono::duration<double>(motion_end - last_step_time_).count();
  if (mode_ == McuMode::kArmed) {
    const double counts_per_radian =
      static_cast<double>(config_.encoder_counts_per_revolution) / kTwoPi;
    left_encoder_count_ +=
      static_cast<double>(left_target_velocity_mrad_s_) * 0.001 * elapsed * counts_per_radian;
    right_encoder_count_ +=
      static_cast<double>(right_target_velocity_mrad_s_) * 0.001 * elapsed * counts_per_radian;
  }

  if (mode_ == McuMode::kArmed && command_received_ && now >= command_deadline_) {
    active_faults_ |= fault_bit(McuFault::kCommandTimeout);
    stop(McuMode::kFault);
  }
  last_step_time_ = now;
}

McuStateFrame FakeMcu::make_state(const TimePoint now) noexcept
{
  step(now);
  McuStateFrame state;
  state.mcu_boot_id = boot_id_;
  state.state_sequence = state_sequence_++;
  state.accepted_host_session_id = accepted_host_session_id_;
  state.last_accepted_command_sequence = last_accepted_command_sequence_;
  state.mcu_uptime_ms = static_cast<std::uint64_t>(
    std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
      now - boot_time_).count()));
  state.mode = mode_;
  state.active_faults = active_faults_;
  state.left_encoder_count = static_cast<std::int64_t>(std::llround(left_encoder_count_));
  state.right_encoder_count = static_cast<std::int64_t>(std::llround(right_encoder_count_));
  state.left_velocity_mrad_s =
    mode_ == McuMode::kArmed ? left_target_velocity_mrad_s_ : 0;
  state.right_velocity_mrad_s =
    mode_ == McuMode::kArmed ? right_target_velocity_mrad_s_ : 0;
  state.last_command_age_ms = command_age_ms(now);
  return state;
}

void FakeMcu::inject_fault(const std::uint32_t fault_mask) noexcept
{
  active_faults_ |= fault_mask & kKnownFaultMask;
  if (active_faults_ != 0U) {
    stop(McuMode::kFault);
  }
}

void FakeMcu::clear_injected_fault(const std::uint32_t fault_mask) noexcept
{
  active_faults_ &= ~(fault_mask & ~kRecoverableCommunicationFaults);
}

std::uint32_t FakeMcu::boot_id() const noexcept
{
  return boot_id_;
}

McuMode FakeMcu::mode() const noexcept
{
  return mode_;
}

std::uint32_t FakeMcu::active_faults() const noexcept
{
  return active_faults_;
}

void FakeMcu::stop(const McuMode mode) noexcept
{
  mode_ = mode;
  left_target_velocity_mrad_s_ = 0;
  right_target_velocity_mrad_s_ = 0;
}

void FakeMcu::latch_validation_fault(const FrameValidationError error) noexcept
{
  if (error == FrameValidationError::kProtocolVersion) {
    active_faults_ |= fault_bit(McuFault::kProtocolMismatch);
  } else {
    active_faults_ |= fault_bit(McuFault::kInvalidCommand);
  }
  stop(McuMode::kFault);
}

std::uint32_t FakeMcu::command_age_ms(const TimePoint now) const noexcept
{
  if (!command_received_) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  const auto age = std::max<std::int64_t>(
    0, std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_).count());
  return static_cast<std::uint32_t>(std::min<std::int64_t>(
    age, std::numeric_limits<std::uint32_t>::max()));
}

}  // namespace diffbot_hardware
