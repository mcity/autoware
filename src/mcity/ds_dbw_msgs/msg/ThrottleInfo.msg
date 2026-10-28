std_msgs/Header header

# Accelerator Pedal
float32 accel_pedal_pc # Accelerator pedal percent, range 0 to 100
Quality accel_pedal_qf # Non-zero is limp-home
OnePedal one_pedal     # One-pedal-drive mode

# Engine
float32 engine_rpm               # Engine speed (rpm)
float32 engine_throttle_valve_pc # Engine throttle valve percent, range 0 to 100

# Drive Mode
DriveMode drive_mode
