std_msgs/Header header

# High voltage traction battery
float32 state_of_charge # %
float32 voltage # V
float32 current # A
float32 temperature # deg C

# Charge status
uint8 status
uint8 STATUS_UNKNOWN=0
uint8 STATUS_NOT_CHARGING=1
uint8 STATUS_CHARGING=2
uint8 STATUS_COMPLETE=3
uint8 STATUS_FAULT=7
