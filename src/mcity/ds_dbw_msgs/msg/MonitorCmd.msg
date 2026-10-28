std_msgs/Header header

# User fault command
uint8 cmd_type

# Command types
uint8 NONE=0
uint8 ACTIVATE_TEST_FAULT=1
uint8 CLEAR_TEST_FAULT=2