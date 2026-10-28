std_msgs/Header header

# Steering Wheel
float32 cmd       # Command, changes with cmd_type
float32 cmd_rate  # Rate limit, deg/s, range 0 to 1016, 0 = default, INFINITY = unlimited
float32 cmd_accel # Accel limit, deg/s^2, range 0 to 25400, 0 = default, INFINITY = unlimited
uint8 cmd_type
uint8 control_mode

# Torque passthrough (only supported on platforms that intercept a steering actuator torque sensor and transmit the SteerTorqueOut message)
float32 torque_passthrough # 0-100%, 0% for best control performance, 100% for best human interaction

# Enable
bool enable

# Clear latched overrides
bool clear

# Ignore future overrides
bool ignore

# Command types
uint8 CMD_NONE=0        # Command  Rate   Accel    Range        Note
uint8 CMD_TORQUE=1      # Nm       N/A    N/A      <depends>    Torque in Nm
uint8 CMD_ANGLE=2       # deg      deg/s  deg/s^2  <depends>    Angle in deg
uint8 CMD_CURVATURE=3   # 1/m      deg/s  deg/s^2  -0.2 to 0.2  Curvature in 1/m
uint8 CMD_YAW_RATE=4    # rad/s    deg/s  deg/s^2  -8.5 to 8.5  Yaw rate in rad/s
uint8 CMD_PERCENT=14    # %        deg/s  deg/s^2  -100 to 100  Percent of max steering wheel angle
uint8 CMD_SET_OFFSET=13 # deg      N/A    N/A      <depends>    Set steering wheel angle calibration offset to value in cmd
uint8 CMD_CALIBRATE=15  # deg      N/A    N/A      <depends>    Calibrate current steering wheel angle to value in cmd

# Control modes
uint8 CONTROL_UNSPECIFIED=0
uint8 CONTROL_PARK_ASSIST=1
uint8 CONTROL_LANE_KEEP=2