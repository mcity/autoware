std_msgs/Header header

# Brake command
float32 cmd      # Command, changes with cmd_type
float32 rate_inc # Rate limit for command increase, 0 for default, INFINITY for no limit, changes with cmd_type
float32 rate_dec # Rate limit for command decrease, 0 for default, INFINITY for no limit, changes with cmd_type
uint8 cmd_type

# Enable
bool enable

# Clear latched overrides
bool clear

# Ignore future overrides
bool ignore

# Prechage AEB (only for platforms using ACC/AEB for braking)
uint8 precharge_aeb
uint8 PRECHARGE_NONE=0
uint8 PRECHARGE_LEVEL_1=1
uint8 PRECHARGE_LEVEL_2=2

# Command types
uint8 CMD_NONE=0        # Command  Rate-limit  Range     Note
uint8 CMD_PRESSURE=1    # bar      bar/s       0 to max  Pressure in bar
uint8 CMD_TORQUE=2      # Nm       Nm/s        0 to max  Torque in Nm
uint8 CMD_ACCEL=8       # m/s^2    m/s^3       -10 to 2  Acceleration in m/s^2 using ACC with fallback to AEB for large magnitude
uint8 CMD_ACCEL_ACC=9   # m/s^2    m/s^3       -10 to 2  Acceleration in m/s^2 using ACC only
uint8 CMD_ACCEL_AEB=10  # m/s^2    m/s^3       -10 to 2  Acceleration in m/s^2 using AEB with fallback to ACC for extended duration
uint8 CMD_PEDAL_RAW=13  # %        %/s         0 to 100  Raw pedal sensor units
uint8 CMD_PERCENT=14    # %        %/s         0 to 100  Percent of maximum pressure/torque/decel

# Passive brake acceleration value for CMD_ACCEL* command types
uint8 CMD_ACCEL_PASSIVE=2 # m/s^2
