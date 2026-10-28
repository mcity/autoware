std_msgs/Header header

# Status
uint8 STATUS_UNKNOWN=0
uint8 STATUS_OFF=1
uint8 STATUS_ACTIVE=2
uint8 STATUS_ERROR=3
uint8 status
bool camera_used
bool navigation_used

# Speed limit
uint8 UNIT_UNKNOWN=0
uint8 UNIT_KPH=1
uint8 UNIT_MPH=2
uint8 speed_units
float32 speed_limit
