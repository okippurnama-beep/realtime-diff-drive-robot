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

#include "diffbot_hardware/diffbot_system_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "diffbot_hardware/fake_mcu_transport.hpp"

namespace diffbot_hardware
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMradPerRad = 1000.0;
constexpr std::chrono::milliseconds kHandshakePollPeriod{1};

std::uint32_t next_session_id() noexcept
{
  static std::atomic<std::uint32_t> next{1U};
  auto value = next.fetch_add(1U, std::memory_order_relaxed);
  if (value == 0U) {
    value = next.fetch_add(1U, std::memory_order_relaxed);
  }
  return value;
}

bool has_exact_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & first, const std::string & second = "")
{
  const std::size_t expected_size = second.empty() ? 1U : 2U;
  if (interfaces.size() != expected_size) {
    return false;
  }
  const auto contains = [&interfaces](const std::string & name) {
      return std::any_of(
        interfaces.begin(), interfaces.end(),
        [&name](const auto & interface) {
          return interface.name == name && interface.data_type == "double";
        });
    };
  return contains(first) && (second.empty() || contains(second));
}

bool parse_positive_u32(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, std::uint32_t & value, const bool required)
{
  const auto entry = parameters.find(name);
  if (entry == parameters.end()) {
    return !required;
  }
  try {
    std::size_t parsed = 0U;
    const auto number = std::stoull(entry->second, &parsed, 10);
    if (parsed != entry->second.size() || number == 0U ||
      number > std::numeric_limits<std::uint32_t>::max())
    {
      return false;
    }
    value = static_cast<std::uint32_t>(number);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

DiffbotSystemHardware::DiffbotSystemHardware()
: DiffbotSystemHardware(std::make_unique<FakeMcuTransport>())
{
}

DiffbotSystemHardware::DiffbotSystemHardware(
  std::unique_ptr<McuTransport> transport, NowFunction now)
: transport_(std::move(transport)), now_(std::move(now))
{
}

DiffbotSystemHardware::~DiffbotSystemHardware()
{
  stop_transport();
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!transport_ || !now_) {
    RCLCPP_ERROR(get_logger(), "MCU transport or steady clock is not available");
    return CallbackReturn::ERROR;
  }
  if (!parse_hardware_parameters() || !validate_hardware_info()) {
    RCLCPP_ERROR(get_logger(), "Invalid DiffbotSystemHardware URDF configuration");
    return CallbackReturn::ERROR;
  }

  left_position_interface_ = left_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION;
  left_velocity_state_interface_ = left_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
  left_velocity_command_interface_ = left_velocity_state_interface_;
  right_position_interface_ = right_wheel_name_ + "/" + hardware_interface::HW_IF_POSITION;
  right_velocity_state_interface_ = right_wheel_name_ + "/" + hardware_interface::HW_IF_VELOCITY;
  right_velocity_command_interface_ = right_velocity_state_interface_;
  return CallbackReturn::SUCCESS;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  stop_transport();
  reset_runtime_state();
  session_prepared_ = false;
  try {
    left_position_handle_ = get_state_interface_handle(left_position_interface_);
    left_velocity_state_handle_ = get_state_interface_handle(left_velocity_state_interface_);
    left_velocity_command_handle_ = get_command_interface_handle(left_velocity_command_interface_);
    right_position_handle_ = get_state_interface_handle(right_position_interface_);
    right_velocity_state_handle_ = get_state_interface_handle(right_velocity_state_interface_);
    right_velocity_command_handle_ = get_command_interface_handle(
      right_velocity_command_interface_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Failed to cache wheel interfaces: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  zero_commands();
  return prepare_session() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_transport();
  reset_runtime_state();
  zero_commands();
  session_prepared_ = false;
  return CallbackReturn::SUCCESS;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!session_prepared_ && !prepare_session()) {
    return CallbackReturn::ERROR;
  }
  reset_runtime_state();
  zero_commands();

  if (transport_->activate() != TransportResult::kOk) {
    return CallbackReturn::ERROR;
  }
  transport_active_ = true;

  const auto disarm = make_command(CommandMode::kDisarm, 0.0, 0.0);
  if (transport_->send_command(disarm) != TransportResult::kOk ||
    !wait_for_mode(McuMode::kDisarmed))
  {
    stop_transport();
    session_prepared_ = false;
    return CallbackReturn::ERROR;
  }

  const auto arm = make_command(CommandMode::kArmed, 0.0, 0.0);
  if (transport_->send_command(arm) != TransportResult::kOk ||
    !wait_for_mode(McuMode::kArmed))
  {
    stop_transport();
    session_prepared_ = false;
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_transport();
  zero_commands();
  session_prepared_ = false;
  return CallbackReturn::SUCCESS;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_transport();
  zero_commands();
  session_prepared_ = false;
  return CallbackReturn::SUCCESS;
}

DiffbotSystemHardware::CallbackReturn DiffbotSystemHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  stop_transport();
  zero_commands();
  session_prepared_ = false;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffbotSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!transport_active_) {
    return hardware_interface::return_type::ERROR;
  }

  ReceivedMcuState received;
  const auto result = transport_->receive_latest(received);
  if (result == TransportResult::kOk) {
    return accept_state(received, true) ?
           hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
  }
  if (result != TransportResult::kNoData || !have_valid_state_) {
    return hardware_interface::return_type::ERROR;
  }

  const auto now = now_();
  if (now < last_valid_state_time_ ||
    now - last_valid_state_time_ > transport_config_.state_timeout)
  {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffbotSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!transport_active_) {
    return hardware_interface::return_type::ERROR;
  }

  double left = 0.0;
  double right = 0.0;
  if (!get_command(left_velocity_command_handle_, left, false) ||
    !get_command(right_velocity_command_handle_, right, false))
  {
    return hardware_interface::return_type::ERROR;
  }
  if (!std::isfinite(left) || !std::isfinite(right) ||
    std::abs(left * kMradPerRad) > kMaximumTargetVelocityMradS ||
    std::abs(right * kMradPerRad) > kMaximumTargetVelocityMradS)
  {
    return hardware_interface::return_type::ERROR;
  }

  const auto command = make_command(CommandMode::kArmed, left, right);
  return transport_->send_command(command) == TransportResult::kOk ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool DiffbotSystemHardware::validate_hardware_info() const
{
  if (info_.joints.size() != 2U || left_wheel_name_ == right_wheel_name_) {
    return false;
  }
  bool found_left = false;
  bool found_right = false;
  for (const auto & joint : info_.joints) {
    if (joint.name != left_wheel_name_ && joint.name != right_wheel_name_) {
      return false;
    }
    found_left = found_left || joint.name == left_wheel_name_;
    found_right = found_right || joint.name == right_wheel_name_;
    if (!has_exact_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_exact_interface(
        joint.state_interfaces, hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY))
    {
      return false;
    }
  }
  return found_left && found_right;
}

bool DiffbotSystemHardware::parse_hardware_parameters()
{
  const auto & parameters = info_.hardware_parameters;
  const auto left_name = parameters.find("left_wheel_name");
  const auto right_name = parameters.find("right_wheel_name");
  if (left_name != parameters.end()) {
    left_wheel_name_ = left_name->second;
  }
  if (right_name != parameters.end()) {
    right_wheel_name_ = right_name->second;
  }

  std::uint32_t command_validity_ms = kDefaultCommandValidityMs;
  std::uint32_t state_timeout_ms = kDefaultStateTimeoutMs;
  std::uint32_t activation_timeout_ms = 500U;
  if (!parse_positive_u32(
      parameters, "encoder_counts_per_revolution", encoder_counts_per_revolution_, true) ||
    !parse_positive_u32(parameters, "command_validity_ms", command_validity_ms, false) ||
    !parse_positive_u32(parameters, "state_timeout_ms", state_timeout_ms, false) ||
    !parse_positive_u32(parameters, "activation_timeout_ms", activation_timeout_ms, false) ||
    command_validity_ms > kMaximumCommandValidityMs || activation_timeout_ms > 5000U)
  {
    return false;
  }
  transport_config_.command_validity = std::chrono::milliseconds{command_validity_ms};
  transport_config_.state_timeout = std::chrono::milliseconds{state_timeout_ms};
  activation_timeout_ = std::chrono::milliseconds{activation_timeout_ms};
  return true;
}

bool DiffbotSystemHardware::prepare_session()
{
  host_session_id_ = next_session_id();
  command_sequence_ = 0U;
  transport_config_.host_session_id = host_session_id_;
  session_prepared_ = transport_->configure(transport_config_) == TransportResult::kOk;
  return session_prepared_;
}

bool DiffbotSystemHardware::wait_for_mode(const McuMode expected_mode)
{
  const auto deadline = now_() + activation_timeout_;
  while (now_() <= deadline) {
    ReceivedMcuState received;
    const auto result = transport_->receive_latest(received);
    if (result == TransportResult::kOk) {
      if (accept_state(received, false) && received.frame.active_faults == 0U &&
        received.frame.accepted_host_session_id == host_session_id_ &&
        received.frame.last_accepted_command_sequence == command_sequence_ &&
        received.frame.mode == expected_mode)
      {
        return true;
      }
      if (validate_state(received.frame) != FrameValidationError::kNone ||
        received.frame.active_faults != 0U)
      {
        return false;
      }
    } else if (result != TransportResult::kNoData) {
      return false;
    }
    std::this_thread::sleep_for(kHandshakePollPeriod);
  }
  return false;
}

bool DiffbotSystemHardware::accept_state(
  const ReceivedMcuState & received, const bool require_armed)
{
  const auto & frame = received.frame;
  if (validate_state(frame) != FrameValidationError::kNone) {
    return false;
  }
  if (mcu_boot_id_ == 0U) {
    mcu_boot_id_ = frame.mcu_boot_id;
  } else if (frame.mcu_boot_id != mcu_boot_id_) {
    return false;
  }
  if (have_state_sequence_ && !sequence_is_newer(frame.state_sequence, last_state_sequence_)) {
    return false;
  }
  if (have_valid_state_ && received.received_at < last_valid_state_time_) {
    return false;
  }
  if (received.received_at > now_()) {
    return false;
  }
  if (require_armed &&
    (frame.accepted_host_session_id != host_session_id_ || frame.mode != McuMode::kArmed ||
    frame.active_faults != 0U))
  {
    return false;
  }

  if (!copy_feedback(frame)) {
    return false;
  }
  last_state_sequence_ = frame.state_sequence;
  last_valid_state_time_ = received.received_at;
  have_state_sequence_ = true;
  have_valid_state_ = true;
  return true;
}

bool DiffbotSystemHardware::copy_feedback(const McuStateFrame & frame)
{
  const auto radians_per_count = kTwoPi / static_cast<double>(encoder_counts_per_revolution_);
  return set_state(
    left_position_handle_,
    static_cast<double>(frame.left_encoder_count) * radians_per_count, false) &&
         set_state(
    right_position_handle_,
    static_cast<double>(frame.right_encoder_count) * radians_per_count, false) &&
         set_state(
    left_velocity_state_handle_,
    static_cast<double>(frame.left_velocity_mrad_s) / kMradPerRad, false) &&
         set_state(
    right_velocity_state_handle_,
    static_cast<double>(frame.right_velocity_mrad_s) / kMradPerRad, false);
}

void DiffbotSystemHardware::reset_runtime_state()
{
  mcu_boot_id_ = 0U;
  last_state_sequence_ = 0U;
  last_valid_state_time_ = TimePoint{};
  have_state_sequence_ = false;
  have_valid_state_ = false;
}

void DiffbotSystemHardware::zero_commands()
{
  if (left_velocity_command_handle_) {
    static_cast<void>(set_command(left_velocity_command_handle_, 0.0, true));
  }
  if (right_velocity_command_handle_) {
    static_cast<void>(set_command(right_velocity_command_handle_, 0.0, true));
  }
}

void DiffbotSystemHardware::stop_transport() noexcept
{
  if (!transport_ || !transport_active_) {
    return;
  }
  const auto disarm = make_command(CommandMode::kDisarm, 0.0, 0.0);
  static_cast<void>(transport_->send_command(disarm));
  transport_->deactivate();
  transport_active_ = false;
}

HostCommandFrame DiffbotSystemHardware::make_command(
  const CommandMode mode, const double left, const double right) noexcept
{
  HostCommandFrame command;
  command.host_session_id = host_session_id_;
  command.command_sequence = ++command_sequence_;
  command.valid_for_ms = static_cast<std::uint16_t>(transport_config_.command_validity.count());
  command.mode = mode;
  command.left_target_velocity_mrad_s = static_cast<std::int32_t>(
    std::llround(left * kMradPerRad));
  command.right_target_velocity_mrad_s = static_cast<std::int32_t>(
    std::llround(right * kMradPerRad));
  return command;
}

}  // namespace diffbot_hardware

PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::DiffbotSystemHardware, hardware_interface::SystemInterface)
