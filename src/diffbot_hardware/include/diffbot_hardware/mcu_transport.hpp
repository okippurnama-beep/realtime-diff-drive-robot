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

#ifndef DIFFBOT_HARDWARE__MCU_TRANSPORT_HPP_
#define DIFFBOT_HARDWARE__MCU_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>

#include "diffbot_hardware/mcu_protocol.hpp"

namespace diffbot_hardware
{

struct TransportConfig
{
  std::uint32_t host_session_id{0U};
  std::chrono::milliseconds command_validity{kDefaultCommandValidityMs};
  std::chrono::milliseconds state_timeout{kDefaultStateTimeoutMs};
};

struct ReceivedMcuState
{
  McuStateFrame frame{};
  std::chrono::steady_clock::time_point received_at{};
};

enum class TransportResult : std::uint8_t
{
  kOk = 0U,
  kNoData,
  kDisconnected,
  kProtocolError,
  kIoError,
};

class McuTransport
{
public:
  McuTransport() = default;
  virtual ~McuTransport() = default;

  McuTransport(const McuTransport &) = delete;
  McuTransport & operator=(const McuTransport &) = delete;
  McuTransport(McuTransport &&) = delete;
  McuTransport & operator=(McuTransport &&) = delete;

  virtual TransportResult configure(const TransportConfig & config) noexcept = 0;
  virtual TransportResult activate() noexcept = 0;
  virtual TransportResult send_command(const HostCommandFrame & command) noexcept = 0;
  virtual TransportResult receive_latest(ReceivedMcuState & state) noexcept = 0;
  virtual void deactivate() noexcept = 0;
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__MCU_TRANSPORT_HPP_
