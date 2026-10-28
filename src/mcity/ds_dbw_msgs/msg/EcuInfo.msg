std_msgs/Header header

# Info
string name          # OEM_PLATFORM_MODULE
string app_version   # x.y.z
string boot_version  # x.y
string hw_rev
string app_signature # XXXXXXXX
string mac_addr      # XX:XX:XX:XX:XX:XX

# Config
string config_hash  # XXXXXXXX
uint8 config_count_modified
uint8 config_count_configured
bool config_nvm_blank
bool config_nvm_write_pending

# License
string build_date   # YYYY/MM/DD
string license_date # YYYY/MM/DD
bool control_licensed

# Logging
string log_filename # XXXXXX.dbw
bool log_filesystem_present
bool log_fault

# Date/time
builtin_interfaces/Time datetime

# Temperature
float32 ecu_temp   # degC
float32 board_temp # degC
