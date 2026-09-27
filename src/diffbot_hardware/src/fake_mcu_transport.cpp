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

#include "diffbot_hardware/fake_mcu_transport.hpp"

#include <utility>

namespace diffbot_hardware
{

FakeMcuTransport::FakeMcuTransport(NowFunction now, FakeMcuConfig mcu_config)
: now_(std::move(now)), mcu_(mcu_config)
{
}

TransportResult FakeMcuTransport::configure(const TransportConfig & config) noexcept
{
  if (config.host_session_id == 0U || config.command_validity.count() <= 0 ||
    config.command_validity.count() > kMaximumCommandValidityMs ||
    config.state_timeout.count() <= 0)
  {
    return TransportResult::kProtocolError;
  }
  config_ = config;
  configured_ = true;
  return TransportResult::kOk;
}

TransportResult FakeMcuTransport::activate() noexcept
{
  if (!configured_) {
    return TransportResult::kProtocolError;
  }
  const auto now = now_();
  clear_pending();
  mcu_.boot(now);
  next_state_time_ = now;
  active_ = true;
  return TransportResult::kOk;
}

TransportResult FakeMcuTransport::send_command(const HostCommandFrame & command) noexcept
{
  if (!active_) {
    return TransportResult::kDisconnected;
  }
  if (validate_command(command) != FrameValidationError::kNone) {
    return TransportResult::kProtocolError;
  }
  if (drop_commands_) {
    return TransportResult::kOk;
  }
  const PendingCommand pending{command, now_() + command_delay_};
  return push_pending(pending) ? TransportResult::kOk : TransportResult::kIoError;
}

TransportResult FakeMcuTransport::receive_latest(ReceivedMcuState & state) noexcept
{
  if (!active_) {
    return TransportResult::kDisconnected;
  }
  const auto now = now_();
  process_pending(now);
  mcu_.step(now);
  if (now < next_state_time_) {
    return TransportResult::kNoData;
  }
  next_state_time_ = now + kStatePeriod;
  const auto frame = mcu_.make_state(now);
  if (drop_states_) {
    return TransportResult::kNoData;
  }
  state.frame = frame;
  state.received_at = now;
  return TransportResult::kOk;
}

void FakeMcuTransport::deactivate() noexcept
{
  active_ = false;
  clear_pending();
}

void FakeMcuTransport::set_drop_commands(const bool enabled) noexcept
{
  drop_commands_ = enabled;
}

void FakeMcuTransport::set_drop_states(const bool enabled) noexcept
{
  drop_states_ = enabled;
}

void FakeMcuTransport::set_command_delay(const std::chrono::milliseconds delay) noexcept
{
  command_delay_ = delay.count() < 0 ? std::chrono::milliseconds{0} : delay;
}

void FakeMcuTransport::reboot() noexcept
{
  if (!active_) {
    return;
  }
  const auto now = now_();
  clear_pending();
  mcu_.boot(now);
  next_state_time_ = now;
}

bool FakeMcuTransport::push_pending(const PendingCommand & command) noexcept
{
  if (pending_size_ >= pending_.size()) {
    return false;
  }
  const auto tail = (pending_head_ + pending_size_) % pending_.size();
  pending_[tail] = command;
  ++pending_size_;
  return true;
}

void FakeMcuTransport::process_pending(const TimePoint now) noexcept
{
  while (pending_size_ > 0U && pending_[pending_head_].delivery_time <= now) {
    mcu_.accept_command(pending_[pending_head_].frame, now);
    pending_head_ = (pending_head_ + 1U) % pending_.size();
    --pending_size_;
  }
}

void FakeMcuTransport::clear_pending() noexcept
{
  pending_head_ = 0U;
  pending_size_ = 0U;
}

}  // namespace diffbot_hardware
