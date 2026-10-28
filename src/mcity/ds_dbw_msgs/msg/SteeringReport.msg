std_msgs/Header header

# Steering Wheel
float32 steering_wheel_angle  # deg
float32 steering_column_torque # Nm
float32 cmd # Changes with cmd_type
uint8 cmd_type # Options defined in SteeringCmd message

#
bool limiting_value
bool limiting_rate
bool external_control

# Status
bool ready    # Ready to be enabled
bool enabled  # Enabled
bool override_active  # Active override
bool override_other   # Override from other system
bool override_latched # Latched override
bool timeout  # Command timeout (of valid messages)
bool fault # Fault of any kind
bool bad_crc # CRC error on most recent command
bool bad_rc # Repeated rolling counter on most recent command

##### Start of slower secondary reports #####

# Degradation of any kind
bool degraded

# Control disabled in hardware due to external electrical signal
bool hardware_disable

# Actuator temperature
float32 actuator_temp # degC

# Calculated limit vs vehicle speed
float32 limit_rate  # deg/s
float32 limit_value # deg

# Command source
CmdSrc cmd_src

# Diagnostics omitted to ruduce message size and clutter
# See SteeringDiagnostics.msg
