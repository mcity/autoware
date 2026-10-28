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

# Command types
uint8 CMD_NONE=0        # Command  Rate-limit  Range     Note
uint8 CMD_PEDAL_RAW=13  # %        %/s         0 to 100  Raw pedal sensor units
uint8 CMD_PERCENT=14    # %        %/s         0 to 100  Percent
