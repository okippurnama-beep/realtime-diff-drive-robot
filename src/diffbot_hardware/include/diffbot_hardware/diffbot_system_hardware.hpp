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

#ifndef DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HARDWARE_HPP_
#define DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HARDWARE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffbot_hardware/mcu_transport.hpp"

namespace diffbot_hardware
{

class DiffbotSystemHardware final : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using NowFunction = std::function<TimePoint()>;

  DiffbotSystemHardware();
  explicit DiffbotSystemHardware(
    std::unique_ptr<McuTransport> transport,
    NowFunction now = [] () noexcept {return Clock::now();});
  ~DiffbotSystemHardware() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool validate_hardware_info() const;
  bool parse_hardware_parameters();
  bool prepare_session();
  bool wait_for_mode(McuMode expected_mode);
  bool accept_state(const ReceivedMcuState & received, bool require_armed);
  bool copy_feedback(const McuStateFrame & frame);
  void reset_runtime_state();
  void zero_commands();
  void stop_transport() noexcept;
  HostCommandFrame make_command(CommandMode mode, double left, double right) noexcept;

  std::unique_ptr<McuTransport> transport_;
  NowFunction now_;
  TransportConfig transport_config_{};
  std::string left_wheel_name_{"left_wheel_joint"};
  std::string right_wheel_name_{"right_wheel_joint"};
  std::string left_position_interface_;
  std::string left_velocity_state_interface_;
  std::string left_velocity_command_interface_;
  std::string right_position_interface_;
  std::string right_velocity_state_interface_;
  std::string right_velocity_command_interface_;
  hardware_interface::StateInterface::SharedPtr left_position_handle_;
  hardware_interface::StateInterface::SharedPtr left_velocity_state_handle_;
  hardware_interface::CommandInterface::SharedPtr left_velocity_command_handle_;
  hardware_interface::StateInterface::SharedPtr right_position_handle_;
  hardware_interface::StateInterface::SharedPtr right_velocity_state_handle_;
  hardware_interface::CommandInterface::SharedPtr right_velocity_command_handle_;
  std::uint32_t encoder_counts_per_revolution_{0U};
  std::chrono::milliseconds activation_timeout_{500};
  std::uint32_t host_session_id_{0U};
  std::uint32_t command_sequence_{0U};
  std::uint32_t mcu_boot_id_{0U};
  std::uint32_t last_state_sequence_{0U};
  TimePoint last_valid_state_time_{};
  bool session_prepared_{false};
  bool transport_active_{false};
  bool have_state_sequence_{false};
  bool have_valid_state_{false};
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HARDWARE_HPP_
