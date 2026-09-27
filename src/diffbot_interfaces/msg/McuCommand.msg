# Fixed-size host-to-MCU command frame. All timeouts are evaluated by the MCU's
# local monotonic clock after receipt; there is intentionally no ROS timestamp.

uint16 PROTOCOL_VERSION_CURRENT=1

uint8 MODE_DISARM=0
uint8 MODE_ARMED=1
uint8 MODE_ESTOP=2

uint16 protocol_version
uint32 host_session_id
uint32 command_sequence
uint16 valid_for_ms
uint8 mode
int32 left_target_velocity_mrad_s
int32 right_target_velocity_mrad_s
