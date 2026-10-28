std_msgs/Header header

# Inhibit control for steer/brake/throttle/gear
bool inhibit

# System parameter values
bool validate_cmd_crc_rc        # Parameter ValidateCmdCrcRc value
SystemSyncMode system_sync_mode # Parameter SystemSyncMode value

# System state
SystemState state

# Status enumerations
uint8 reason_disengage
uint8 reason_not_ready
string reason_disengage_str
string reason_not_ready_str

# Buttons
bool btn_enable
bool btn_disable

# Status
bool lockout
bool override # Any steer/brake/throttle/gear override
bool ready    # All steer/brake/throttle ready, and gear not faulted
bool enabled  # All steer/brake/throttle enabled, and gear not faulted (or any steer/brake/throttle enabled for Mode>=AllOrNone)
bool fault    # Fault in any steer/brake/throttle/gear
bool bad_crc  # CRC error in most recent system command message
bool bad_rc   # Repeated rolling counter in most recent system command message
