std_msgs/Header header

# Remote control parameter values
RemoteMode remote_control_mode # Parameter RemoteControlMode value

# Remote control state
RemoteState remote_control_state

# Received signal strength from transmitter to receiver
float32 rssi_dbm # dBm

# Remote battery
float32 battery_pc    # (%)
bool battery_charging # Plugged in or charging

# Status
bool estop           # Remote e-stop
bool estop_press     # Remote e-stop button pressed
bool estop_timeout   # Remote e-stop timeout (from transmitter to receiver)
bool estop_fault     # Remote e-stop fault
bool timeout_can_joy # Joystick CAN messages timeout from receiver
bool timeout_can_sys # System CAN messages timeout from receiver
