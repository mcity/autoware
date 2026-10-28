std_msgs/Header header

# Attention report
float32 attention # %
uint8 gaze
bool eyes_present

# Gaze types
uint8 GAZE_OTHER=0
uint8 GAZE_PASSENGER_WINDSHIELD=1
uint8 GAZE_DRIVER_WINDSHIELD=2
uint8 GAZE_INSTRUMENT_PANEL=3
uint8 GAZE_MEDIA_TABLET=4
uint8 GAZE_REAR_MIRROR=6
uint8 GAZE_DRIVER_MIRROR=8
uint8 GAZE_PASSENGER_MIRROR=9
uint8 GAZE_BELOW_DASHBOARD=10