#ifndef PATH_PROCESSING_HPP
#define PATH_PROCESSING_HPP

#include <map>
#include <fstream> 
#include <iostream>
#include <algorithm>
#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <sx.hpp>
#include <xmath.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;

class pathProcessing
{

public:
    pathProcessing(){};
    ~pathProcessing(){};

    void init(
        Plan_Rlt_S * const planning2control_msg,
        VehState_S * const vehicle_state,
        double max_allowed_ey_, 
        double max_allowed_ephi_,
        double max_allowed_curvature_,
        double heading_offset_,
        int heading_lookahead_points_,
        double lateral_offset_,
        std::string slope_file_
    );

    void run();
    void resampling();
    void process_path(double desired_time_resolution, double preview_time);
    void set_curve_speed(double lat_acc_, double decel_, double min_speed_);
    void set_curve_lookahead(double seconds_);
    // Curvature preview for the feedforward (p2c.cr_vector); gain 0 = off (cr_vector stays empty).
    void set_feedforward(double gain) { ff_gain = gain > 0 ? gain : 0.0; }


private:
    void compute_curvature();
    void fill_curvature_preview(int closest_index);
    void compute_curve_speed();
    void find_stop_point();        // on the raw planner path, before downsampling
    double get_stop_distance();
    int get_closest_index();

    double get_desired_velocity(int closest_index);
    double get_orientation_error(int closest_index);
    double get_lateral_error(int closest_index);
    void load_slope(std::string slope_file);
    double get_slope();

    Plan_Rlt_S * _p2c  = NULL;
    VehState_S * _vs   = NULL;

    double max_allowed_ey;
    double max_allowed_ephi;
    double max_allowed_curvature;
    double heading_offset;
    int heading_lookahead_points;
    int values;
    double lateral_offset;
    std::map<std::pair<int, int>, double> slope_map; // (x, y) [m] -> road grade [rad]

    // curve speed limit: v <= sqrt(curve_lat_acc / curvature), reached with curve_decel; 0 = off
    double curve_lat_acc   = 0.0; // m/s^2
    double curve_decel     = 1.0; // m/s^2
    double curve_min_speed = 0.0; // m/s
    // use the lowest curve limit within this many seconds ahead (at the current speed), so the slow speed loop
    // starts braking early enough; 0 = limit at the closest point only
    double curve_lookahead = 0.0; // s
    std::vector<double> curve_vmax; // per path point, filled by compute_curve_speed()

    // feedforward: signed path curvature (left +, 1/m) per processed path point, and its arc length
    double ff_gain = 0.0;

    // next planned stop point (first point ahead with vd <= 0.05) and the path direction there
    bool   stop_valid = false;
    double stop_x = 0.0, stop_y = 0.0, stop_tx = 1.0, stop_ty = 0.0;
    std::vector<double> path_s, path_kappa;
};

#endif // PATHFOLLOWING_HPP