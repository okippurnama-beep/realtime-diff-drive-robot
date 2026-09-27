# Fixed-size MCU-to-host feedback frame. MCU uptime and command age are local
# monotonic values and must not be compared directly with host clock values.

uint16 PROTOCOL_VERSION_CURRENT=1

uint8 MODE_BOOT=0
uint8 MODE_DISARMED=1
uint8 MODE_ARMED=2
uint8 MODE_FAULT=3
uint8 MODE_ESTOP=4

uint32 FAULT_NONE=0
uint32 FAULT_COMMAND_TIMEOUT=1
uint32 FAULT_PROTOCOL_MISMATCH=2
uint32 FAULT_COMMAND_SEQUENCE=4
uint32 FAULT_INVALID_COMMAND=8
uint32 FAULT_LEFT_ENCODER=16
uint32 FAULT_RIGHT_ENCODER=32
uint32 FAULT_LEFT_DRIVER=64
uint32 FAULT_RIGHT_DRIVER=128
uint32 FAULT_UNDERVOLTAGE=256
uint32 FAULT_OVERCURRENT=512
uint32 FAULT_CONTROL_LOOP_OVERRUN=1024
uint32 FAULT_MCU_WATCHDOG_RESET=2048
uint32 FAULT_ESTOP_INPUT=4096

uint16 protocol_version
uint32 mcu_boot_id
uint32 state_sequence
uint32 accepted_host_session_id
uint32 last_accepted_command_sequence
uint64 mcu_uptime_ms
uint8 mode
uint32 active_faults
int64 left_encoder_count
int64 right_encoder_count
int32 left_velocity_mrad_s
int32 right_velocity_mrad_s
uint32 last_command_age_ms
uint32 control_loop_overrun_count
