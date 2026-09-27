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

#ifndef DIFFBOT_SAFETY__SAFETY_POLICY_HPP_
#define DIFFBOT_SAFETY__SAFETY_POLICY_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace diffbot_safety
{

using Duration = std::chrono::nanoseconds;
using FaultMask = std::uint32_t;

enum class SafetyState : std::uint8_t
{
  kStartupInhibit = 0,
  kReady = 1,
  kFaultLatched = 2,
  kEstopLatched = 3,
};

enum class Fault : FaultMask
{
  kNone = 0U,
  kManualEstop = 1U << 0,
  kCommandStale = 1U << 1,
  kScanStale = 1U << 2,
  kOdomStale = 1U << 3,
  kNav2Inactive = 1U << 4,
  kInvalidCommand = 1U << 5,
  kCommandLimitViolation = 1U << 6,
  kMcuHeartbeatLost = 1U << 7,
};

constexpr FaultMask to_mask(Fault fault)
{
  return static_cast<FaultMask>(fault);
}

constexpr bool has_fault(FaultMask mask, Fault fault)
{
  return (mask & to_mask(fault)) != 0U;
}

struct MotionCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
};

struct SafetyConfig
{
  Duration command_timeout{std::chrono::milliseconds(300)};
  Duration scan_timeout{std::chrono::milliseconds(500)};
  Duration odom_timeout{std::chrono::milliseconds(500)};
  Duration mcu_heartbeat_timeout{std::chrono::milliseconds(200)};

  double max_forward_velocity{0.25};
  double max_reverse_velocity{0.10};
  double max_angular_velocity{1.0};
  double zero_velocity_epsilon{1.0e-4};

  bool require_scan{true};
  bool require_odom{true};
  bool require_nav2_active{true};
  bool require_mcu_heartbeat{false};
};

struct SafetySnapshot
{
  bool command_seen{false};
  MotionCommand command{};
  Duration command_age{Duration::zero()};

  bool scan_seen{false};
  Duration scan_age{Duration::zero()};

  bool odom_seen{false};
  Duration odom_age{Duration::zero()};

  bool nav2_active{false};
  bool emergency_stop{false};

  bool mcu_heartbeat_seen{false};
  Duration mcu_heartbeat_age{Duration::zero()};
};

struct SafetyDecision
{
  SafetyState state{SafetyState::kStartupInhibit};
  FaultMask active_faults{to_mask(Fault::kNone)};
  FaultMask latched_faults{to_mask(Fault::kNone)};
  bool output_inhibited{true};
  MotionCommand output_command{};
};

class SafetyPolicy
{
public:
  explicit SafetyPolicy(SafetyConfig config = SafetyConfig{});

  SafetyDecision evaluate(const SafetySnapshot & snapshot);
  bool reset(const SafetySnapshot & snapshot);

  SafetyState state() const noexcept;
  FaultMask latched_faults() const noexcept;
  const SafetyConfig & config() const noexcept;

private:
  FaultMask detect_active_faults(const SafetySnapshot & snapshot) const;
  bool command_is_zero(const MotionCommand & command) const;
  bool command_is_finite(const MotionCommand & command) const;
  bool command_exceeds_limits(const MotionCommand & command) const;
  SafetyDecision make_decision(
    const SafetySnapshot & snapshot,
    FaultMask active_faults) const;

  SafetyConfig config_;
  SafetyState state_{SafetyState::kStartupInhibit};
  FaultMask latched_faults_{to_mask(Fault::kNone)};
};

std::string state_name(SafetyState state);
std::string fault_mask_to_string(FaultMask faults);

}  // namespace diffbot_safety

#endif  // DIFFBOT_SAFETY__SAFETY_POLICY_HPP_
