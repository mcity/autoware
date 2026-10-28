std_msgs/Header header

uint8 ramp_request
uint8 RAMP_SLOW = 0
uint8 RAMP_MEDIUM = 1
uint8 RAMP_FAST = 2
uint8 RAMP_IMMEDIATE = 3

uint8 precision_request
uint8 PRECISION_COMFORTABLE = 0
uint8 PRECISION_PRECISE = 1

uint8 control_request
uint8 CONTROL_NONE = 0
uint8 CONTROL_PATH_FOLLOW_LIMITED = 1
uint8 CONTROL_PATH_FOLLOW_EXTENDED = 2
uint8 CONTROL_SAFE_RAMP_OUT = 3

bool hands_off_confirm_request

float32 curvature      # 1/m
float32 path_angle     # degrees
float32 path_offset    # meters
float32 curvature_rate # 1/m^2