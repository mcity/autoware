std_msgs/Header header

# Turn Signal enumeration
TurnSignal turn_signal

# Parking brake
PrkBrkStat parking_brake

# Passenger seat
bool passenger_detect
bool passenger_airbag

# Seat Belts
bool buckle_driver
bool buckle_passenger
bool buckle_row2_driver
bool buckle_row2_middle
bool buckle_row2_passenger
bool buckle_row3_driver
bool buckle_row3_middle
bool buckle_row3_passenger

# Doors
bool door_driver
bool door_passenger
bool door_rear_left
bool door_rear_right
bool door_hood
bool door_trunk

# Locks
bool lock_driver
bool lock_passenger
bool lock_rear_left
bool lock_rear_right

# Buttons
bool btn_ld_ok        # Left D-Pad OK
bool btn_ld_up        # Left D-Pad Up
bool btn_ld_down      # Left D-Pad Down
bool btn_ld_left      # Left D-Pad Left
bool btn_ld_right     # Left D-Pad Right
bool btn_rd_ok        # Right D-Pad OK
bool btn_rd_up        # Right D-Pad Up
bool btn_rd_down      # Right D-Pad Down
bool btn_rd_left      # Right D-Pad Left
bool btn_rd_right     # Right D-Pad Right
bool btn_cc_mode      # Cruise Control Mode
bool btn_cc_on        # Cruise Control On
bool btn_cc_off       # Cruise Control Off
bool btn_cc_res       # Cruise Control Resume
bool btn_cc_cncl      # Cruise Control Cancel
bool btn_cc_on_off    # Cruise Control On/Off Toggle
bool btn_cc_res_cncl  # Cruise Control Resume/Cancel
bool btn_cc_res_inc   # Cruise Control Resume+
bool btn_cc_res_dec   # Cruise Control Resume-
bool btn_cc_set_inc   # Cruise Control Set+
bool btn_cc_set_dec   # Cruise Control Set-
bool btn_acc_gap_inc  # Adaptive Cruise Control Gap+
bool btn_acc_gap_dec  # Adaptive Cruise Control Gap-
bool btn_limit_on_off # Speed Limiter On/Off Toggle
bool btn_la_on_off    # Lane Assist On/Off Toggle
bool btn_apa          # Active Park Assist
bool btn_media        # Media (source)
bool btn_vol_inc      # Volume increment
bool btn_vol_dec      # Volume decrement
bool btn_mute         # Volume mute
bool btn_speak        # Speak
bool btn_prev         # Media previous
bool btn_next         # Media next
bool btn_call_start   # Call start/answer
bool btn_call_end     # Call end/disconnect

# Front Windshield Wipers enumeration
Wiper wiper

# Headlight enumerations
bool headlight_low
bool headlight_high
HeadlightCtrlLow headlight_low_control
HeadlightCtrlHigh headlight_high_control
Bool blackout

# Ambient Light Sensor enumeration
AmbientLight ambient_light

# Temperatures (deg C)
float32 outside_air_temp         # Outside air
float32 engine_coolant_temp      # Engine coolant
float32 engine_oil_temp          # Engine oil
float32 trans_oil_temp           # Transmission oil
float32 trans_clutch_temp        # Transmission clutch
float32 traction_motor_coil_temp # BEV/HEV traction motor winding
float32 traction_motor_elec_temp # BEV/HEV traction motor controller

# Over-temperature warnings
bool engine_coolant_hot_warn      # Engine coolant
bool engine_oil_hot_warn          # Engine oil
bool trans_oil_hot_warn           # Transmission oil
bool trans_clutch_hot_warn        # Transmission clutch
bool traction_motor_coil_hot_warn # BEV/HEV traction motor winding
bool traction_motor_elec_hot_warn # BEV/HEV traction motor controller
