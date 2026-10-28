std_msgs/Header header

float32 steering_wheel_angle        # deg, raw + offset
float32 steering_wheel_angle_raw    # deg, angle - offset
float32 steering_wheel_angle_offset # deg, angle - raw
uint8 offset_type
uint8 OFFSET_UNKNOWN=0
uint8 OFFSET_RELATAIVE=1
uint8 OFFSET_ABSOLUTE=2
uint8 OFFSET_INTERNAL=3

float32 steering_wheel_angle_error  # deg
