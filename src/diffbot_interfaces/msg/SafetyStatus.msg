# Safety supervisor state. Values are stable API constants for logs and tests.
uint8 STATE_STARTUP_INHIBIT=0
uint8 STATE_READY=1
uint8 STATE_FAULT_LATCHED=2
uint8 STATE_ESTOP_LATCHED=3

# Faults are bit flags. Multiple faults may be active or latched at once.
uint32 FAULT_NONE=0
uint32 FAULT_MANUAL_ESTOP=1
uint32 FAULT_COMMAND_STALE=2
uint32 FAULT_SCAN_STALE=4
uint32 FAULT_ODOM_STALE=8
uint32 FAULT_NAV2_INACTIVE=16
uint32 FAULT_INVALID_COMMAND=32
uint32 FAULT_COMMAND_LIMIT_VIOLATION=64
uint32 FAULT_MCU_HEARTBEAT_LOST=128

builtin_interfaces/Time stamp
uint8 state
uint32 active_faults
uint32 latched_faults
bool output_inhibited

# A negative age means that the corresponding input has never been observed.
float32 command_age_sec
float32 scan_age_sec
float32 odom_age_sec
float32 mcu_heartbeat_age_sec

string summary
