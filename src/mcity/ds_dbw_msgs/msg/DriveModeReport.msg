std_msgs/Header header

#
DriveMode drive_mode
DriveMode drive_mode_cmd
DriveMode suspension_mode
DriveMode suspension_mode_cmd
AwdMode awd_mode
AwdMode awd_mode_cmd
DiffLock rear_diff
DiffLock rear_diff_cmd
DiffLock front_diff
DiffLock front_diff_cmd
OnePedal one_pedal
OnePedal one_pedal_cmd

# Status
bool ready # Ready to actuate command
bool override_active # Active override
bool override_other  # Override from other system
bool bad_crc # CRC error on most recent command

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

# Supporting flags
bool support_drive_mode_cmd
bool support_suspension_mode_cmd
bool support_awd_mode_cmd
bool support_rear_diff_cmd
bool support_front_diff_cmd
bool support_one_pedal_cmd
