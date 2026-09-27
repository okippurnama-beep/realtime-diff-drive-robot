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

#ifndef DIFFBOT_HARDWARE__FAKE_MCU_HPP_
#define DIFFBOT_HARDWARE__FAKE_MCU_HPP_

#include <chrono>
#include <cstdint>

#include "diffbot_hardware/mcu_protocol.hpp"

namespace diffbot_hardware
{

struct FakeMcuConfig
{
  std::uint32_t encoder_counts_per_revolution{2048U};
  std::int32_t maximum_velocity_mrad_s{kMaximumTargetVelocityMradS};
};

class FakeMcu
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit FakeMcu(FakeMcuConfig config = {});

  void boot(TimePoint now) noexcept;
  FrameValidationError accept_command(
    const HostCommandFrame & command, TimePoint now) noexcept;
  void step(TimePoint now) noexcept;
  McuStateFrame make_state(TimePoint now) noexcept;
  void inject_fault(std::uint32_t fault_mask) noexcept;
  void clear_injected_fault(std::uint32_t fault_mask) noexcept;

  std::uint32_t boot_id() const noexcept;
  McuMode mode() const noexcept;
  std::uint32_t active_faults() const noexcept;

private:
  static constexpr std::uint32_t kRecoverableCommunicationFaults =
    static_cast<std::uint32_t>(McuFault::kCommandTimeout) |
    static_cast<std::uint32_t>(McuFault::kProtocolMismatch) |
    static_cast<std::uint32_t>(McuFault::kCommandSequence) |
    static_cast<std::uint32_t>(McuFault::kInvalidCommand);

  void stop(McuMode mode) noexcept;
  void latch_validation_fault(FrameValidationError error) noexcept;
  std::uint32_t command_age_ms(TimePoint now) const noexcept;

  FakeMcuConfig config_{};
  std::uint32_t boot_id_{0U};
  std::uint32_t state_sequence_{0U};
  std::uint32_t accepted_host_session_id_{0U};
  std::uint32_t last_accepted_command_sequence_{0U};
  std::uint32_t active_faults_{0U};
  McuMode mode_{McuMode::kBoot};
  std::int32_t left_target_velocity_mrad_s_{0};
  std::int32_t right_target_velocity_mrad_s_{0};
  double left_encoder_count_{0.0};
  double right_encoder_count_{0.0};
  bool command_received_{false};
  TimePoint boot_time_{};
  TimePoint last_step_time_{};
  TimePoint last_command_time_{};
  TimePoint command_deadline_{};
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__FAKE_MCU_HPP_
