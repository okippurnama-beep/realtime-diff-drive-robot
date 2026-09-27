# M8 micro-ROS Adapter Boundary

## Scope

This document freezes the host-side seam for replacing the deterministic fake
MCU with a future STM32 micro-ROS endpoint. It is an implementation contract,
not evidence that STM32 firmware, micro-ROS, serial DMA, or physical motors are
already integrated.

The only runtime substitution point is a pluginlib class implementing
`diffbot_hardware::McuTransport`. The current Xacro selects:

```xml
<param name="transport_plugin">diffbot_hardware/FakeMcuTransport</param>
```

The future adapter will use a separate name such as
`diffbot_hardware/MicroRosTransport`. Changing that parameter must not require
changes to `DiffbotSystemHardware`, controller YAML, the M7 safety supervisor,
Nav2, or the command-topic chain.

## Stable Ownership Boundary

```text
Nav2 / M7 safety gate / diff_drive_controller       unchanged
                         |
DiffbotSystemHardware                               unchanged
                         |
HostCommandFrame <-> McuTransport <-> McuStateFrame frozen seam
                         |
          FakeMcuTransport | future MicroRosTransport
                         |
                    STM32 firmware
```

`DiffbotSystemHardware` owns exactly one transport instance and its pluginlib
loader. The transport is created during hardware initialization, configured
during lifecycle setup, activated before the zero-speed handshake, and
destroyed before its class loader. A missing or unknown plugin is an
initialization error; there is no silent fallback to fake hardware.

`FaultInjectableMcuTransport` is an optional test capability used by the fake
transport. A production micro-ROS adapter must not implement it unless a
separate, explicitly test-only fault backend exists.

## Realtime Method Contract

After activation, the following calls remain bounded, non-blocking, and
`noexcept`:

- `send_command(const HostCommandFrame &)`: copy the newest command into a
  preallocated single-producer/single-consumer mailbox and return once.
- `receive_latest(ReceivedMcuState &)`: copy at most the newest complete state
  from a preallocated mailbox and return `kNoData` when no new sample exists.
- `deactivate()`: request transport shutdown without waiting in a control-loop
  callback.

These methods may not allocate, sleep, wait for DDS discovery, perform serial
I/O, call a ROS executor, retry, reconnect, or log per cycle. DDS callbacks,
micro-ROS Agent discovery, serialization, and reconnect handling belong to a
non-realtime worker/executor owned by the future adapter. A generation counter
or equivalent double-buffer protocol must prevent torn frames.

Lifecycle methods may perform documented bounded setup. The existing
`DiffbotSystemHardware` activation handshake remains the authority for new
sessions: zero `DISARM`, matching acknowledgement, zero `ARMED`, matching
acknowledgement, then ordinary commands.

## ROS/micro-ROS Wire Mapping

The future adapter maps the fixed C++ frames one-to-one to the existing
generated messages:

| Direction | Proposed private wire topic | Type | QoS |
| --- | --- | --- | --- |
| Host to MCU | `/mcu/command_raw` | `diffbot_interfaces/msg/McuCommand` | reliable, keep-last 1, volatile |
| MCU to host | `/mcu/state_raw` | `diffbot_interfaces/msg/McuState` | reliable, keep-last 1, volatile |

The public `/mcu/state` topic remains sanitized host evidence published by the
hardware component for M7; the adapter must not publish it directly. This
prevents an MCU topic from bypassing host validation.

Every field, unit, integer width, enum, fault bit, session rule, and modulo
sequence rule remains exactly as defined in
`M8_VIRTUAL_MCU_ARCHITECTURE.md`. Neither endpoint adds ROS timestamps or
compares STM32 time with host time. The adapter attaches a host
`steady_clock` receive time only after a complete state frame reaches the host.

## STM32 Execution Split

The intended physical implementation keeps motor safety independent of ROS:

- A 1 kHz FreeRTOS control task reads encoder snapshots, runs velocity PID,
  applies output limits, and updates the motor driver.
- Timer/input-capture or encoder peripherals use interrupts/DMA; callbacks do
  bounded work and wake tasks rather than running PID or ROS code.
- A local monotonic timer enforces the command deadline and forces PWM to zero
  without relying on the host, DDS, or the micro-ROS Agent.
- A lower-priority micro-ROS executor validates incoming `McuCommand` frames
  and publishes a fixed-size `McuState` snapshot. It exchanges data with the
  control task through static queues/mailboxes.
- Static allocation is preferred for FreeRTOS objects, micro-ROS entities, and
  message storage after startup. No dynamic allocation occurs in the control
  loop.
- Physical E-stop, driver fault, undervoltage, overcurrent, encoder failure,
  loop-overrun, and watchdog-reset evidence maps to the frozen v1 fault mask.

The micro-ROS Agent and serial transport are external dependencies. Losing the
Agent or UART stream must age out the MCU-local command deadline first; host
state freshness then drives ros2_control and M7 into their existing error path.

## Disconnect and Recovery Rules

1. `kNoData` means the adapter is alive but has no newer complete state.
2. Agent/session loss returns `kDisconnected`; serialization or frame mapping
   failures return `kProtocolError`; host I/O failures return `kIoError`.
3. The realtime method does not reconnect. A non-realtime worker may attempt
   rediscovery, but the active hardware session remains invalid.
4. A new MCU boot ID, reconnect, or agent restart requires ros2_control
   lifecycle recovery and a new host session. Old commands are never replayed.
5. The MCU remains stopped until the full zero-speed activation handshake has
   completed for the new session.
6. M7 remains latched after communication health returns and still requires
   its explicit guarded reset.

## Adapter Conformance Checklist

A future `MicroRosTransport` is not accepted until it demonstrates all of the
following without changing callers above `McuTransport`:

- pluginlib discovery and failure on an unknown plugin name;
- protocol/message constant mapping and fixed-size messages;
- bounded zero-speed activation and a new nonzero host session;
- command/state sequence handling including uint32 wraparound;
- encoder-count and mrad/s conversion through the existing hardware plugin;
- independent MCU command watchdog with measured PWM-zero evidence;
- host state timeout, Agent disconnect, delayed/lost frame, and reboot tests;
- no allocation, wait, or I/O in ros2_control `read()`/`write()`;
- repeated latency CSV plus offline acceptance report using the physical test
  profile and explicitly declared thresholds;
- logic-analyzer or oscilloscope evidence for physical output shutdown;
- clean five-package regression with the fake transport still available.

The M8.5 virtual measurements remain a software baseline. They cannot be
reused as STM32, UART, micro-ROS, motor, or stopping-distance results.
