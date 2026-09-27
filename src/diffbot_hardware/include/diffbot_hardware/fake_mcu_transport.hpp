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

#ifndef DIFFBOT_HARDWARE__FAKE_MCU_TRANSPORT_HPP_
#define DIFFBOT_HARDWARE__FAKE_MCU_TRANSPORT_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>

#include "diffbot_hardware/fake_mcu.hpp"
#include "diffbot_hardware/mcu_transport.hpp"

namespace diffbot_hardware
{

class FakeMcuTransport final : public McuTransport
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using NowFunction = std::function<TimePoint()>;

  explicit FakeMcuTransport(
    NowFunction now = [] () noexcept {return Clock::now();},
    FakeMcuConfig mcu_config = {});

  TransportResult configure(const TransportConfig & config) noexcept override;
  TransportResult activate() noexcept override;
  TransportResult send_command(const HostCommandFrame & command) noexcept override;
  TransportResult receive_latest(ReceivedMcuState & state) noexcept override;
  void deactivate() noexcept override;

  void set_drop_commands(bool enabled) noexcept;
  void set_drop_states(bool enabled) noexcept;
  void set_command_delay(std::chrono::milliseconds delay) noexcept;
  void reboot() noexcept;

private:
  struct PendingCommand
  {
    HostCommandFrame frame{};
    TimePoint delivery_time{};
  };

  static constexpr std::size_t kPendingCapacity = 32U;
  static constexpr std::chrono::milliseconds kStatePeriod{10};

  bool push_pending(const PendingCommand & command) noexcept;
  void process_pending(TimePoint now) noexcept;
  void clear_pending() noexcept;

  NowFunction now_;
  FakeMcu mcu_;
  TransportConfig config_{};
  std::array<PendingCommand, kPendingCapacity> pending_{};
  std::size_t pending_head_{0U};
  std::size_t pending_size_{0U};
  bool configured_{false};
  bool active_{false};
  std::atomic<bool> drop_commands_{false};
  std::atomic<bool> drop_states_{false};
  std::atomic<std::int64_t> command_delay_ms_{0};
  std::atomic<bool> reboot_requested_{false};
  TimePoint next_state_time_{};
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__FAKE_MCU_TRANSPORT_HPP_
