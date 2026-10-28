std_msgs/Header header

#
TurnSignal input
TurnSignal cmd
TurnSignal output
TurnSignal feedback

# Status
bool ready # Ready to actuate command
bool override_active # Active override
bool override_other  # Override from other system
bool timeout # Command timeout (of valid messages)
bool bad_crc # CRC error on most recent command
bool bad_rc # Repeated rolling counter on most recent command

# Degraded
bool degraded
bool degraded_cmd_type
bool degraded_comms_dbw_steer
bool degraded_comms_dbw_brake
bool degraded_comms_dbw_thrtl
bool degraded_comms_vehicle
bool degraded_control_performance

# Faults
bool fault
bool fault_comms_vehicle
