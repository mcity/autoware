#include <speed_control.hpp>


void SpeedControl::get_padel_opening(float acc, float speed, float & throttle, float & brake)
{
    // step 0: check
    throttle = 0.0f;
    brake    = 0.23f;

    // step 1: find the speed level
    int vLevel = 0;

    if (speed < SPEEDTABLE[0])
    {
        brake = 0.0f;

        if (acc > 1.5){
            throttle = 0.42f;
        }
        else if (acc > 1.0){
            throttle = 0.32f;
        }
        else if (acc > 0.7){
            throttle = 0.26f;
        }
        else if (acc > 0.25){
            throttle = 0.22f;
        }
        else{
            throttle = 0.0f;
            brake    = 0.05f;
        }

        return;
    }
    else if ( speed > SPEEDTABLE[VNUM-1] - 0.1 )
    {
        throttle = 0.0f;
        brake    = 0.0f;
        return;
    }
    else
    {
        if (speed < SPEEDTABLE[1])
            vLevel = 0;
        else
            vLevel = (int)(speed/0.5);
    }

    // step 2: find the acc level
    float accT[TNUM];

    for (int i = 0; i < TNUM; ++i)
        accT[i] = (  ACCTABLE[i][vLevel+0] *(SPEEDTABLE[vLevel+1]-speed) 
                   + ACCTABLE[i][vLevel+1] *(speed - SPEEDTABLE[vLevel]))
                  / (SPEEDTABLE[vLevel+1]-SPEEDTABLE[vLevel]);

    
    if (acc < accT[0])
    {
        throttle = 0.05f;
        brake    = 0.05f;
    }
        
    else if (acc >= accT[TNUM-1])
    {
        throttle = THROTTLETABLE[TNUM-1];
        brake    = 0.0f;
    }
        
    else
    {
        for (int i = 0; i < TNUM-1; ++i)
            if (acc>= accT[i] && acc < accT[i+1])
            {
                throttle = (THROTTLETABLE[i] * (accT[i+1]-acc) + THROTTLETABLE[i+1] * (acc-accT[i]))/(accT[i+1]-accT[i]);
                brake    = 0.0f;
                break;
            }
    }

    // step 3: computing brake
    // step 2b: just below the lowest table throttle, blend down to 0 instead of jumping 0.15 -> 0.
    // On the MachE zero throttle means regen (~0.3 m/s^2), so the jump caused a surge/regen limit cycle.
    if (acc < accT[0] && lift_decel > 0 && (accT[0] - acc) < lift_decel)
    {
        throttle = THROTTLETABLE[0] * (1.0f - (accT[0] - acc) / lift_decel);
        brake    = 0.0f;
        return;
    }

    if (acc < accT[0]) // need more brake, i.e., coast is not enough
    {
        throttle = 0.0f;
        if (accT[0] - acc < coast_band)
            brake = 0.0f;   // lift off, regen slows the car
        else if (brake_gain > 0)   // MachE: brake 0..1 = 0..100 % (linear in decel)
            brake = brake_gain * (accT[0] - acc - coast_band);
        else                       // MKZ pedal table (min opening 0.184)
            brake = cal_brake_pedal_opening(accT[0] - acc - coast_band);
    }

    if (brake > 0)
        throttle = 0;

    return;
}

float SpeedControl::cal_brake_pedal_opening(float deceleration)
{
    float torque_demand = VEHICLE_WEIGHT * fabs(deceleration) * WHEEL_DIAMETER / 2;
    float brake_pade = 0;

    if ( torque_demand >= BRAKETABLE[5][1]) //row 10
        return BRAKETABLE[5][0];

    for (int i = 0; i < 5; ++i)
    {
        if (torque_demand >= BRAKETABLE[i][1] && torque_demand < BRAKETABLE[i+1][1])
        {
            brake_pade  =  BRAKETABLE[i][0] + (torque_demand - BRAKETABLE[i][1])
                        * (BRAKETABLE[i+1][0] - BRAKETABLE[i][0])
                        / (BRAKETABLE[i+1][1] - BRAKETABLE[i][1]);
            break;
        }
    }
    return brake_pade;
}

float SpeedControl::cal_road_load(float vehicle_speed)
{
    float road_load = ( 76.98 + 40.28 * vehicle_speed)/VEHICLE_WEIGHT;
    
    if (vehicle_speed < 1)
        road_load = 0;

    return road_load;
}

void SpeedControl::ini(
    Plan_Rlt_S * const planning2control_msg,
    VehState_S * const vehicle_state,
    Control_Value_S * const control_output,
    float pid_kp_, 
    float pid_ki_, 
    int frequence_)
{   
    _p2c = planning2control_msg;
    _vs = vehicle_state;
    _ctrl = control_output;
    pid_kp = pid_kp_;
    pid_ki  = pid_ki_;
    frequence = frequence_;

    SC.vd  = 0.0f;
    SC.throttle_cmd = 0.0f;
    SC.brake_cmd    = 0.05f;
    SC.acc_cmd      = -3.0f;
    SC.errIntg       = 0.0f;
    SC.acc_max_barr  = -3.0f;
}

void SpeedControl::set_smoothing(float max_speed_, float coast_band_, float throttle_rate_, float brake_rate_,
                                 float brake_gain_)
{
    brake_gain    = MAX(0.0f, brake_gain_);
    max_speed     = max_speed_ > 0 ? MIN(max_speed_, MAX_SPEED) : MAX_SPEED;
    coast_band    = MAX(0.0f, coast_band_);
    throttle_rate = MAX(0.0f, throttle_rate_);
    brake_rate    = MAX(0.0f, brake_rate_);
}

void SpeedControl::set_throttle_fall_rate(float rate)
{
    throttle_fall_rate = MAX(0.0f, rate);
}

void SpeedControl::set_stop_braking(float stop_decel_, float stop_brake_gain_)
{
    stop_decel      = MAX(0.0f, stop_decel_);
    stop_brake_gain = MAX(0.0f, stop_brake_gain_);
}

void SpeedControl::set_max_accel(float max_accel_)
{
    max_accel = MAX(0.0f, max_accel_);
}

void SpeedControl::set_lift_off(float lift_decel_, bool reset_when_dbw_off_)
{
    lift_decel         = MAX(0.0f, lift_decel_);
    reset_when_dbw_off = reset_when_dbw_off_;
}

int SpeedControl::run()
{
    if (_p2c == NULL || _vs == NULL || _ctrl == NULL ){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Speed controller not initizlized, return");
        return -1;
    }
    count ++;

    // *************** step 0: INPUT, Assignment *****************
    SC.vd       = MIN(_p2c->vd, max_speed);
    SC.speed_x  = MAX(0.0, _vs->speed_x);

    // planned stop ahead: never aim faster than what stop_decel can still stop in the remaining distance
    bool stop_ahead = stop_decel > 0 && _p2c->stop_dist < 1e8;
    if (stop_ahead)
        SC.vd = MIN(SC.vd, std::sqrt(2.0f * stop_decel * MAX(0.0f, float(_p2c->stop_dist))));

    // measured acceleration (low-passed speed derivative, ~0.3 s lag)
    float dt = 1.0f / frequence;
    if (!accel_init) { v_filt = SC.speed_x; a_filt = 0.0f; vd_ramp = SC.speed_x; accel_init = true; }
    float v_prev = v_filt;
    v_filt += (dt / (0.15f + dt)) * (SC.speed_x - v_filt);
    a_filt += (dt / (0.15f + dt)) * ((v_filt - v_prev) / dt - a_filt);

    // ramp the speed target up at max_accel (down immediately); restart from the actual speed while not driving
    if (max_accel > 0)
    {
        if (!_vs->by_wire_enabled)
            vd_ramp = SC.speed_x;
        vd_ramp = MIN(SC.vd, MIN(vd_ramp + max_accel * dt, SC.speed_x + 1.0f));
        SC.vd = vd_ramp;
    }

    // ************************************************************
    //step 1: estimate road slope and mileage
    float slope = _p2c->slope;
    SC.acc_slope = 9.8*sin(slope);

    //step 2: speed error
    SC.vErr = SC.speed_x-SC.vd;

    // While the driver drives (by-wire off) the commands go nowhere; don't let the integral wind up,
    // and restart the pedal rate limiters from 0, so engaging starts smoothly instead of surging.
    if (reset_when_dbw_off && !_vs->by_wire_enabled)
    {
        SC.errIntg   = 0.0f;
        throttle_out = 0.0f;
        brake_out    = 0.0f;
    }

    //step 3: speed control rule
    SC.acc_cmd = speed_pid_ctrl();
    SC.acc_cmd = MAX(MAX_DCC, MIN(MAX_ACC, SC.acc_cmd));

    //step 4: get the padel opening
    get_padel_opening(SC.acc_cmd, SC.speed_x, SC.throttle_cmd, SC.brake_cmd);

    //step 5: check gear
    //make sure the transmission is shifted to "drive"
    if (_vs->gear_position != GEAR_DRIVE && SC.vd > 0.1) //target speed > 0.1
    {
        SC.errIntg      = 0.0f;
		SC.brake_cmd    = 0.0f;
		SC.throttle_cmd = 0.0f;
        SC.gear_cmd     = GEAR_DRIVE;
    }

    // step 6: correct SC.errIntg
    if (_vs->gear_position != GEAR_DRIVE)
        SC.errIntg = 0.0f;

    if (SC.vd == 0 && SC.errIntg < 0)
        SC.errIntg = 0.0f; //no accelerate

    if (SC.speed_x == 0 && SC.errIntg > 0)
        SC.errIntg = 0.0f; //no dcc

    // step 7:if speeding
    if (SC.speed_x > MAX_SPEED + 0.2)
        SC.throttle_cmd = 0.0f;

    // step 8: low speed
    if (SC.speed_x < 2.5 && SC.vd > SC.speed_x+3.0 && _p2c->estop ==0)
    {
        SC.throttle_cmd = 0.3f + SLOPE2PADEL;
    }

    if (SC.speed_x < 0.3 && SC.vd < 2.0 && SC.brake_cmd > 0.1f )
    {
        if (SC.vd < SC.speed_x)
            SC.brake_cmd = MAX(0.05f, 0.05f - SLOPE2PADEL);

        if (SC.vd == 0)
            SC.brake_cmd = 0.05f + ABS(SLOPE2PADEL);
    }
 
    // step 9: stop-point braking. The PI + coast band brakes far too softly for a stop (2026-11-20: the car
    // crossed the goal at 2.8 m/s with brake 0.06 and stopped 7.6 m past it). Brake for the decel the
    // remaining distance needs; it self-corrects: braking too little makes v^2 / 2d grow.
    if (stop_ahead)
    {
        float d = float(_p2c->stop_dist);
        float a_req = (d > 0.05f) ? SC.speed_x * SC.speed_x / (2.0f * d) : stop_decel_max;
        if (SC.speed_x > 0.3f && a_req >= 0.6f * stop_decel)
            stop_braking = true;
        else if (SC.speed_x <= 0.3f || a_req < 0.3f * stop_decel)
            stop_braking = false;

        if (stop_braking)
        {
            float a = MIN(a_req, stop_decel_max);
            SC.errIntg      = 0.0f;
            SC.throttle_cmd = 0.0f;
            SC.brake_cmd    = MAX(SC.brake_cmd, stop_brake_gain * MAX(0.0f, a - stop_regen));
        }
        // at (or past) the point and nearly stopped: hold like set_stop()
        if (d < 0.3f && SC.speed_x < 0.5f)
        {
            SC.errIntg      = 0.0f;
            SC.throttle_cmd = 0.0f;
            SC.brake_cmd    = MAX(SC.brake_cmd, 0.22f);
        }
    }
    else
        stop_braking = false;

    // step 10: acceleration governor. Over max_accel take throttle back (1.5/s per m/s^2 over, not slowed by
    // throttle_fall_rate); above half of it, don't add throttle. Pulling away from a stop: at most 0.12 (MachE:
    // ~1 m/s^2 at low speed; the standstill branch of get_padel_opening asks 0.22-0.42). Tuned in a longitudinal
    // model with the measured MachE throttle response: launch 1.8 mph/s, curve exit 1.5 mph/s at max_accel 0.7.
    bool governing = false;
    if (max_accel > 0 && SC.throttle_cmd > 0)
    {
        if (SC.speed_x < 0.2f)
            SC.throttle_cmd = MIN(SC.throttle_cmd, 0.12f);
        if (a_filt > max_accel)
        {
            SC.throttle_cmd = MAX(0.0f, MIN(SC.throttle_cmd, throttle_out - 1.5f * (a_filt - max_accel) * dt));
            governing = true;
        }
        else if (a_filt > 0.5f * max_accel)
            SC.throttle_cmd = MIN(SC.throttle_cmd, throttle_out);
    }

    // a tiny brake request while still on throttle: ease the throttle off (throttle_fall_rate) instead of cutting
    // it at once. 2026-11-23: brake 0.02 dropped throttle 0.20 -> 0 in one step (1.4 m/s^2 jolt at 39 s).
    if (throttle_fall_rate > 0 && throttle_out > 0.02f && SC.brake_cmd > 0 && SC.brake_cmd < 0.06f
        && !stop_braking && _p2c->estop == estop_NONE)
    {
        SC.brake_cmd    = 0.0f;
        SC.throttle_cmd = 0.0f;
    }

    // ********************** OUTPUT *******************
    SC.throttle_cmd  = MAX(0.0f, MIN( MAX_THROTTLE, SC.throttle_cmd ));
    SC.brake_cmd     = MAX(0.0f, MIN( MAX_BRAKE,    SC.brake_cmd ));

    if (SC.brake_cmd > 0.0f) SC.throttle_cmd = 0.0f;
    if (SC.throttle_cmd> 0.01f) SC.brake_cmd = 0.0f;

    // limit how fast throttle / brake rise (releasing is immediate); estop below is not limited
    if (throttle_rate > 0 && SC.throttle_cmd > throttle_out)
        SC.throttle_cmd = MIN(SC.throttle_cmd, throttle_out + throttle_rate / frequence);
    // ease off the throttle too (MachE lift-off = regen jolt), but only while no brake is wanted:
    // a brake request still drops the throttle at once (brake > 0 forces throttle 0 above)
    if (throttle_fall_rate > 0 && SC.throttle_cmd < throttle_out && SC.brake_cmd <= 0.0f && !governing)
        SC.throttle_cmd = MAX(SC.throttle_cmd, throttle_out - throttle_fall_rate / frequence);
    if (brake_rate > 0 && SC.brake_cmd > brake_out)
        SC.brake_cmd = MIN(SC.brake_cmd, brake_out + brake_rate / frequence);
    throttle_out = SC.throttle_cmd;
    brake_out    = SC.brake_cmd;

    _ctrl->brake         = SC.brake_cmd;
    _ctrl->throttle      = SC.throttle_cmd;
    _ctrl->gear          = SC.gear_cmd;
    _ctrl->turn_signal   = _p2c->signal;


    // ********************** final steps *******************
    if( _p2c->estop != estop_NONE )
    {
        if (_p2c->estop == estop_LOW)
            setestop_level_low();
        else if (_p2c->estop == estop_MEDIUM)
            setestop_level_medium();
        else
            setestop_level_high();
    }

    //smooth brake
    float beta = 0.95;
    if (_ctrl->brake >= 0.18)
    {
        if (_ctrl->brake > brake_prev)
            _ctrl->brake = _ctrl->brake * (1-beta) + brake_prev * beta;
        brake_prev = MAX(0.18, _ctrl->brake);
    };

    return 0;
}

float SpeedControl::speed_pid_ctrl()
{
    float acc = -2.0f;

    //step 1: error integral
    SC.errIntg += (SC.vErr/frequence) * pid_ki;
    SC.errIntg = max(-0.42f, min(1.25f, SC.errIntg) ); // acc ->[ -1.25, 0.42]

    //step 2: feedback of err
    SC.speedFB  = SC.vErr * pid_kp;

    //step 3: control
    acc = - SC.errIntg - SC.speedFB + _p2c->acc_d + SC.acc_slope;
    return acc;
}

void SpeedControl::set_stop()
{
    if ( _vs->speed_x >= 2.0 )
        _ctrl->brake = MAX(0.220f, 0.220f - SLOPE2PADEL);
    else
        _ctrl->brake = MAX(0.22f, 0.22f - SLOPE2PADEL);

    if (_vs->speed_x < 0.2)
        _ctrl->brake = 0.22f + ABS(SLOPE2PADEL);

    if (_vs->speed_x == 0 )
    {        
        if ( stopTimeCount++ > frequence*1.5) // 1.5 seconds
        {
            _ctrl->brake = 0.235f;
            _ctrl->gear = GEAR_PARK;
        }
    }
    else
        stopTimeCount = 0;

    _ctrl->throttle = 0.0f;

    return;
}

void SpeedControl::setestop_level_high()
{
    float throttle = 0, brake =0.05; 
    get_padel_opening(-3.0, _vs->speed_x, throttle, brake);
   
    if (_vs->speed_x < 0.5)
        brake = 0.235f;

    _ctrl->brake = MAX(brake,0.05f);
    _ctrl->throttle = 0.0f;
}

void SpeedControl::setestop_level_medium()
{
    _ctrl->brake = MAX(0.05f, 0.05f - SLOPE2PADEL);
    _ctrl->throttle = 0.0f;
}

void SpeedControl::setestop_level_low()
{
    if ( _vs->speed_x >= 2.0 )
        _ctrl->brake = MAX(0.05f, 0.05f - SLOPE2PADEL);
    else
        _ctrl->brake = MAX(0.05f, 0.05f - SLOPE2PADEL);

    _ctrl->throttle = 0.0f;
    if (_vs->speed_x < 0.2)
        _ctrl->brake = 0.05f + ABS(SLOPE2PADEL);
}
