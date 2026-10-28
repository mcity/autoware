std_msgs/Header header

# ULC report
uint8 cmd_type # Options defined in UlcCmd message
float32 vel_ref  # Reference velocity in m/s
float32 vel_meas # Measured velocity in m/s
float32 accel_ref  # Reference acceleration in m/s^2
float32 accel_meas # Measured acceleration in m/s^2

#
bool coast_decel # Echo of value in UlcCmd message

# Status
bool ready    # Ready to be enabled
bool enabled  # Enabled
bool override_active  # Active override
bool override_latched # Latched override
bool preempted # Preempted by other system
bool timeout  # Command timeout (of valid messages)
bool bad_crc # CRC error on most recent command
bool bad_rc # Repeated rolling counter on most recent command
