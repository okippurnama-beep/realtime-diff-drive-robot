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
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

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
: now_([] () noexcept {return Clock::now();})
{
}

DiffbotSystemHardware::DiffbotSystemHardware(
  std::unique_ptr<McuTransport> transport, NowFunction now)
: transport_(std::move(transport)), now_(std::move(now))
{
  fault_injector_ = dynamic_cast<FaultInjectableMcuTransport *>(transport_.get());
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
  if (!now_) {
    RCLCPP_ERROR(get_logger(), "Steady clock is not available");
    return CallbackReturn::ERROR;
  }
  if (!parse_hardware_parameters() || !validate_hardware_info()) {
    RCLCPP_ERROR(get_logger(), "Invalid DiffbotSystemHardware URDF configuration");
    return CallbackReturn::ERROR;
  }
  if (!transport_ && !load_transport()) {
    return CallbackReturn::ERROR;
  }
  fault_injector_ = dynamic_cast<FaultInjectableMcuTransport *>(transport_.get());

  const auto node = get_node();
  if (node) {
    const auto topic_entry = info_.hardware_parameters.find("mcu_state_topic");
    const std::string topic = topic_entry == info_.hardware_parameters.end() ?
      "/mcu/state" : topic_entry->second;
    if (topic.empty()) {
      RCLCPP_ERROR(get_logger(), "mcu_state_topic cannot be empty");
      return CallbackReturn::ERROR;
    }
    mcu_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<
          diffbot_interfaces::msg::McuState>>(
      node, topic, rclcpp::QoS(10).reliable());
    if (fault_injector_) {
      fault_control_service_ = node->create_service<
        diffbot_interfaces::srv::McuFaultControl>(
        "/mcu/fault_control",
        std::bind(
          &DiffbotSystemHardware::fault_control_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
    }
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

bool DiffbotSystemHardware::load_transport()
{
  const auto entry = info_.hardware_parameters.find("transport_plugin");
  if (entry == info_.hardware_parameters.end() || entry->second.empty()) {
    RCLCPP_ERROR(get_logger(), "transport_plugin is required and cannot be empty");
    return false;
  }
  const std::string & plugin_name = entry->second;

  try {
    transport_loader_ = std::make_unique<pluginlib::ClassLoader<McuTransport>>(
      "diffbot_hardware", "diffbot_hardware::McuTransport");
    transport_ = transport_loader_->createSharedInstance(plugin_name);
  } catch (const pluginlib::PluginlibException & exception) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load MCU transport '%s': %s",
      plugin_name.c_str(), exception.what());
    transport_.reset();
    transport_loader_.reset();
    return false;
  }
  return static_cast<bool>(transport_);
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
  if (received.received_at > now_()) {
    return false;
  }
  const bool boot_changed = mcu_boot_id_ != 0U && frame.mcu_boot_id != mcu_boot_id_;
  if (boot_changed) {
    if (require_armed) {
      publish_mcu_state(frame);
    }
    return false;
  }
  if (mcu_boot_id_ == 0U) {
    mcu_boot_id_ = frame.mcu_boot_id;
  }
  if (have_state_sequence_ && !sequence_is_newer(frame.state_sequence, last_state_sequence_)) {
    return false;
  }
  if (have_valid_state_ && received.received_at < last_valid_state_time_) {
    return false;
  }
  if (require_armed &&
    (frame.accepted_host_session_id != host_session_id_ || frame.mode != McuMode::kArmed ||
    frame.active_faults != 0U))
  {
    publish_mcu_state(frame);
    return false;
  }

  if (!copy_feedback(frame)) {
    return false;
  }
  last_state_sequence_ = frame.state_sequence;
  last_valid_state_time_ = received.received_at;
  have_state_sequence_ = true;
  have_valid_state_ = true;
  if (frame.accepted_host_session_id == host_session_id_ &&
    frame.mode == McuMode::kArmed && frame.active_faults == 0U)
  {
    publish_mcu_state(frame);
  }
  return true;
}

void DiffbotSystemHardware::publish_mcu_state(const McuStateFrame & frame) noexcept
{
  if (!mcu_state_publisher_) {
    return;
  }
  diffbot_interfaces::msg::McuState message;
  message.protocol_version = frame.protocol_version;
  message.mcu_boot_id = frame.mcu_boot_id;
  message.state_sequence = frame.state_sequence;
  message.accepted_host_session_id = frame.accepted_host_session_id;
  message.last_accepted_command_sequence = frame.last_accepted_command_sequence;
  message.mcu_uptime_ms = frame.mcu_uptime_ms;
  message.mode = static_cast<std::uint8_t>(frame.mode);
  message.active_faults = frame.active_faults;
  message.left_encoder_count = frame.left_encoder_count;
  message.right_encoder_count = frame.right_encoder_count;
  message.left_velocity_mrad_s = frame.left_velocity_mrad_s;
  message.right_velocity_mrad_s = frame.right_velocity_mrad_s;
  message.last_command_age_ms = frame.last_command_age_ms;
  message.control_loop_overrun_count = frame.control_loop_overrun_count;
  static_cast<void>(mcu_state_publisher_->try_publish(message));
}

void DiffbotSystemHardware::fault_control_callback(
  const diffbot_interfaces::srv::McuFaultControl::Request::SharedPtr request,
  diffbot_interfaces::srv::McuFaultControl::Response::SharedPtr response)
{
  using FaultControl = diffbot_interfaces::srv::McuFaultControl;
  if (!fault_injector_) {
    response->message = "fault injection is unavailable for this transport";
    return;
  }

  switch (request->scenario) {
    case FaultControl::Request::SCENARIO_CLEAR:
      fault_injector_->set_drop_commands(false);
      fault_injector_->set_drop_states(false);
      fault_injector_->set_command_delay(std::chrono::milliseconds{0});
      response->message = "all transport faults cleared";
      break;
    case FaultControl::Request::SCENARIO_DROP_COMMANDS:
      fault_injector_->set_drop_commands(request->enabled);
      response->message = request->enabled ?
        "command loss enabled" : "command loss disabled";
      break;
    case FaultControl::Request::SCENARIO_DROP_STATES:
      fault_injector_->set_drop_states(request->enabled);
      response->message = request->enabled ?
        "state loss enabled" : "state loss disabled";
      break;
    case FaultControl::Request::SCENARIO_COMMAND_DELAY:
      if (request->enabled && (request->delay_ms == 0U || request->delay_ms > 5000U)) {
        response->message = "delay_ms must be in [1, 5000] when enabled";
        return;
      }
      fault_injector_->set_command_delay(
        std::chrono::milliseconds{request->enabled ? request->delay_ms : 0U});
      response->message = request->enabled ?
        "command delay enabled" : "command delay disabled";
      break;
    case FaultControl::Request::SCENARIO_REBOOT:
      if (!request->enabled) {
        response->message = "reboot requires enabled=true";
        return;
      }
      fault_injector_->reboot();
      response->message = "MCU reboot requested";
      break;
    default:
      response->message = "unknown fault scenario";
      return;
  }
  response->success = true;
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
