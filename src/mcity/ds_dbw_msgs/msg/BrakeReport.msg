std_msgs/Header header

#
uint8 cmd_type # Options defined in BrakeCmd message
float32 pressure_input  # Pressure in bar
float32 pressure_cmd    # Pressure in bar
float32 pressure_output # Pressure in bar
float32 torque_input    # Torque in Nm
float32 torque_cmd      # Torque in Nm
float32 torque_output   # Torque in Nm
float32 accel_cmd       # Acceleration in m/s^2
float32 accel_output    # Acceleration in m/s^2
float32 percent_input   # Percent (0-100)
float32 percent_cmd     # Percent (0-100)
float32 percent_output  # Percent (0-100)

#
bool btsi_cmd
bool yield_request # Request throttle to yield to brakes
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

# Actuator temperature
float32 actuator_temp # degC

# Calculated limit vs vehicle speed
float32 limit_value # percent, bar, or m/s^2

# Brake available duration and actions
BrakeInfDur brake_available_partial_inf
float32 brake_available_duration
bool brake_available_full
bool req_shift_park
bool req_park_brake

# Other braking sources
bool external_button
bool comms_loss_armed

# Command source
CmdSrc cmd_src

# Diagnostics omitted to ruduce message size and clutter
# See BrakeDiagnostics.msg
