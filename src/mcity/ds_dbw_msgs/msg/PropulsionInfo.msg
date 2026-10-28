std_msgs/Header header

# Ford
float32 propulsion_torque_request # Nm
float32 propulsion_torque_actual  # Nm
GearNum gear_num_target
GearNum gear_num_actual

# Polaris
float32 engine_torque_request # Nm
float32 engine_torque_actual  # Nm
float32 transmission_ratio
Bool torque_limiting
Bool limp_home

# Reserved3
float32 engine_torque_request_pc # %
float32 engine_torque_actual_pc  # %
# Also gear_num_target and gear_num_actual from Ford
