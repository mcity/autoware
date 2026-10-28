std_msgs/Header header

uint8 hands_on_confidence
uint8 HANDS_ON_LOW_CONFIDENCE = 0
uint8 HANDS_ON_HIGH_CONFIDENCE = 1

bool ldw_available
bool lka_available
bool deny
bool hands_off

uint8 control_status
uint8 CONTROL_UNAVAILABLE = 0
uint8 CONTROL_AVAILABLE = 1
uint8 CONTROL_IN_PROGRESS = 2
uint8 CONTROL_RAMP_OUT = 3
uint8 CONTROL_DENIED = 4

uint8 control_limit_status
uint8 CONTROL_LIMIT_NOT_REACHED = 0
uint8 CONTROL_LIMIT_CLOSE = 1
uint8 CONTROL_LIMIT_REACHED = 2
uint8 CONTROL_LIMIT_WITH_DRIVER_ACTIVE = 3

uint8 control_capability_status
uint8 CONTROL_CAPABILITY_NONE = 0
uint8 CONTROL_CAPABILITY_LIMITED = 1
uint8 CONTROL_CAPABILITY_EXTENDED = 2
uint8 CONTROL_CAPABILITY_FAULTY = 3