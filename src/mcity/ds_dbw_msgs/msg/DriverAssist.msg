std_msgs/Header header

# Brake deceleration
float32 decel # m/s^2
DecelSrc decel_src

# Forward Collision Warning
bool fcw_active
bool fcw_enabled

# Automated Emergency Braking
bool aeb_active
bool aeb_precharge
bool aeb_enabled

# Adaptive Cruise Control
bool acc_braking
bool acc_enabled

# Blind Spot Information System
bool blis_l_alert
bool blis_l_enabled
bool blis_r_alert
bool blis_r_enabled

# Cross Traffic Alert
bool cta_l_brake
bool cta_l_alert
bool cta_l_enabled
bool cta_r_brake
bool cta_r_alert
bool cta_r_enabled

# Reverse Brake Assist
bool rba_active
bool rba_enabled

# Auto Brake Hold
bool hold_active
bool hold_enabled