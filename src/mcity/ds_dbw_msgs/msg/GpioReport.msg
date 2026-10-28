std_msgs/Header header

# Command
uint8 CMD_PASSIVE=0
uint8 CMD_ACTIVE=1
uint8 CMD_LOW=1
uint8 gpio1_cmd
uint8 gpio2_cmd
uint8 gpio3_cmd
uint8 gpio4_cmd
uint8 gpio5_cmd
uint8 gpio6_cmd
uint8 ssr1_cmd
uint8 ssr2_cmd

# Status
uint8 GPIO_LOW=0
uint8 GPIO_HIGH=1
uint8 gpio1_status
uint8 gpio2_status
uint8 gpio3_status
uint8 gpio4_status
uint8 gpio5_status
uint8 gpio6_status

# Supported flags
bool gpio1_cmd_supported
bool gpio2_cmd_supported
bool gpio3_cmd_supported
bool gpio4_cmd_supported
bool gpio5_cmd_supported
bool gpio6_cmd_supported
bool ssr1_cmd_supported
bool ssr2_cmd_supported

# Status
bool timeout  # Command timeout (of valid messages)
bool bad_crc # CRC error on most recent command
bool bad_rc # Repeated rolling counter on most recent command
