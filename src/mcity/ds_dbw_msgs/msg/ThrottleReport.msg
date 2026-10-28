std_msgs/Header header

#
uint8 cmd_type # Options defined in BrakeCmd message
float32 percent_input   # Percent (0-100)
float32 percent_cmd     # Percent (0-100)
float32 percent_output  # Percent (0-100)

#
bool yield_request # Request brakes to yield to throttle
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

##### Start of slower secondary report #####

# Degradation of any kind
bool degraded

# Control disabled in hardware due to external electrical signal
bool hardware_disable

# Calculated limit vs vehicle speed
float32 limit_value # percent

# Command source
CmdSrc cmd_src

# Diagnostics omitted to ruduce message size and clutter
# See ThrottleDiagnostics.msg
