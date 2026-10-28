std_msgs/Header header

# ULC command
float32 cmd # Command, changes with cmd_type
float32 limit_accel # m/s^2, rate limit for velocity command increase, 0 for default, INFINITY for no limit
float32 limit_decel # m/s^2, rate limit for velocity command decrease, 0 for default, INFINITY for no limit
float32 limit_jerk_throttle # m/s^3, rate limit for acceleration command increase, 0 for default, INFINITY for no limit
float32 limit_jerk_brake    # m/s^3, rate limit for acceleration command decrease, 0 for default, INFINITY for no limit
uint8 cmd_type

# Enable
bool enable

# Clear latched overrides
bool clear

# Options
bool enable_shift
bool enable_shift_park
bool coast_decel

# Command types
uint8 CMD_NONE=0
uint8 CMD_VELOCITY=1 # m/s
uint8 CMD_ACCEL=2    # m/s^2
