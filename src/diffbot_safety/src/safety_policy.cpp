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

#include "diffbot_safety/safety_policy.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace diffbot_safety
{

namespace
{

void validate_positive_duration(const Duration value, const char * name)
{
  if (value <= Duration::zero()) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

}  // namespace

SafetyPolicy::SafetyPolicy(SafetyConfig config)
: config_(std::move(config))
{
  validate_positive_duration(config_.command_timeout, "command_timeout");
  validate_positive_duration(config_.scan_timeout, "scan_timeout");
  validate_positive_duration(config_.odom_timeout, "odom_timeout");
  validate_positive_duration(
    config_.mcu_heartbeat_timeout,
    "mcu_heartbeat_timeout");

  if (config_.max_forward_velocity <= 0.0 ||
    config_.max_reverse_velocity <= 0.0 ||
    config_.max_angular_velocity <= 0.0)
  {
    throw std::invalid_argument("velocity limits must be positive");
  }
  if (config_.zero_velocity_epsilon < 0.0) {
    throw std::invalid_argument("zero_velocity_epsilon cannot be negative");
  }
}

SafetyDecision SafetyPolicy::evaluate(const SafetySnapshot & snapshot)
{
  const FaultMask active_faults = detect_active_faults(snapshot);
  const bool estop_active = has_fault(active_faults, Fault::kManualEstop);
  const FaultMask invalid_command_faults =
    to_mask(Fault::kInvalidCommand) |
    to_mask(Fault::kCommandLimitViolation);
  const bool startup_can_arm =
    active_faults == to_mask(Fault::kNone) &&
    (!snapshot.command_seen || command_is_zero(snapshot.command));

  if (estop_active) {
    latched_faults_ |= active_faults;
    state_ = SafetyState::kEstopLatched;
  } else {
    switch (state_) {
      case SafetyState::kStartupInhibit:
        if ((active_faults & invalid_command_faults) != 0U) {
          latched_faults_ |= active_faults;
          state_ = SafetyState::kFaultLatched;
        } else if (startup_can_arm) {
          state_ = SafetyState::kReady;
        }
        break;
      case SafetyState::kReady:
        if (active_faults != to_mask(Fault::kNone)) {
          latched_faults_ |= active_faults;
          state_ = SafetyState::kFaultLatched;
        }
        break;
      case SafetyState::kFaultLatched:
      case SafetyState::kEstopLatched:
        latched_faults_ |= active_faults;
        break;
    }
  }

  return make_decision(snapshot, active_faults);
}

bool SafetyPolicy::reset(const SafetySnapshot & snapshot)
{
  if (state_ != SafetyState::kFaultLatched &&
    state_ != SafetyState::kEstopLatched)
  {
    return false;
  }

  const FaultMask active_faults = detect_active_faults(snapshot);
  if (active_faults != to_mask(Fault::kNone)) {
    return false;
  }
  if (snapshot.command_seen && !command_is_zero(snapshot.command)) {
    return false;
  }

  latched_faults_ = to_mask(Fault::kNone);
  state_ = SafetyState::kReady;
  return true;
}

SafetyState SafetyPolicy::state() const noexcept
{
  return state_;
}

FaultMask SafetyPolicy::latched_faults() const noexcept
{
  return latched_faults_;
}

const SafetyConfig & SafetyPolicy::config() const noexcept
{
  return config_;
}

FaultMask SafetyPolicy::detect_active_faults(
  const SafetySnapshot & snapshot) const
{
  FaultMask faults = to_mask(Fault::kNone);

  if (snapshot.emergency_stop) {
    faults |= to_mask(Fault::kManualEstop);
  }

  if (snapshot.command_seen) {
    if (!command_is_finite(snapshot.command)) {
      faults |= to_mask(Fault::kInvalidCommand);
    } else if (command_exceeds_limits(snapshot.command)) {
      faults |= to_mask(Fault::kCommandLimitViolation);
    }

    if (!command_is_zero(snapshot.command) &&
      snapshot.command_age > config_.command_timeout)
    {
      faults |= to_mask(Fault::kCommandStale);
    }
  }

  if (config_.require_scan &&
    (!snapshot.scan_seen || snapshot.scan_age > config_.scan_timeout))
  {
    faults |= to_mask(Fault::kScanStale);
  }

  if (config_.require_odom &&
    (!snapshot.odom_seen || snapshot.odom_age > config_.odom_timeout))
  {
    faults |= to_mask(Fault::kOdomStale);
  }

  if (config_.require_nav2_active && !snapshot.nav2_active) {
    faults |= to_mask(Fault::kNav2Inactive);
  }

  if (config_.require_mcu_heartbeat &&
    (!snapshot.mcu_heartbeat_seen ||
    snapshot.mcu_heartbeat_age > config_.mcu_heartbeat_timeout))
  {
    faults |= to_mask(Fault::kMcuHeartbeatLost);
  }

  return faults;
}

bool SafetyPolicy::command_is_zero(const MotionCommand & command) const
{
  return std::abs(command.linear_x) <= config_.zero_velocity_epsilon &&
         std::abs(command.angular_z) <= config_.zero_velocity_epsilon;
}

bool SafetyPolicy::command_is_finite(const MotionCommand & command) const
{
  return std::isfinite(command.linear_x) &&
         std::isfinite(command.angular_z);
}

bool SafetyPolicy::command_exceeds_limits(
  const MotionCommand & command) const
{
  return command.linear_x > config_.max_forward_velocity ||
         command.linear_x < -config_.max_reverse_velocity ||
         std::abs(command.angular_z) > config_.max_angular_velocity;
}

SafetyDecision SafetyPolicy::make_decision(
  const SafetySnapshot & snapshot,
  const FaultMask active_faults) const
{
  SafetyDecision decision;
  decision.state = state_;
  decision.active_faults = active_faults;
  decision.latched_faults = latched_faults_;
  decision.output_inhibited =
    state_ != SafetyState::kReady ||
    active_faults != to_mask(Fault::kNone) ||
    !snapshot.command_seen;

  if (!decision.output_inhibited) {
    decision.output_command = snapshot.command;
  }

  return decision;
}

std::string state_name(const SafetyState state)
{
  switch (state) {
    case SafetyState::kStartupInhibit:
      return "STARTUP_INHIBIT";
    case SafetyState::kReady:
      return "READY";
    case SafetyState::kFaultLatched:
      return "FAULT_LATCHED";
    case SafetyState::kEstopLatched:
      return "ESTOP_LATCHED";
  }
  return "UNKNOWN";
}

std::string fault_mask_to_string(const FaultMask faults)
{
  if (faults == to_mask(Fault::kNone)) {
    return "NONE";
  }

  const std::vector<std::pair<Fault, const char *>> names{
    {Fault::kManualEstop, "MANUAL_ESTOP"},
    {Fault::kCommandStale, "COMMAND_STALE"},
    {Fault::kScanStale, "SCAN_STALE"},
    {Fault::kOdomStale, "ODOM_STALE"},
    {Fault::kNav2Inactive, "NAV2_INACTIVE"},
    {Fault::kInvalidCommand, "INVALID_COMMAND"},
    {Fault::kCommandLimitViolation, "COMMAND_LIMIT_VIOLATION"},
    {Fault::kMcuHeartbeatLost, "MCU_HEARTBEAT_LOST"},
  };

  std::string result;
  for (const auto & [fault, name] : names) {
    if (!has_fault(faults, fault)) {
      continue;
    }
    if (!result.empty()) {
      result += "|";
    }
    result += name;
  }
  return result;
}

}  // namespace diffbot_safety
