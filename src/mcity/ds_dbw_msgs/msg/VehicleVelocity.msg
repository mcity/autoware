std_msgs/Header header

# Vehicle velocities
float32 vehicle_velocity_brake      # m/s, measured by brakes (wheel sensors)
float32 vehicle_velocity_propulsion # m/s, measured by propulsion system

# Source of direction
uint8 dir_src
uint8 DIR_NONE=0   # Direction unknown
uint8 DIR_PRNDL=1  # Transmission reverse gear sets negative direction
uint8 DIR_SENSOR=2 # Explicit wheel speed/direction sensors
