#include <path_process.hpp>

void pathProcessing::init(
    Plan_Rlt_S * const planning2control_msg,
    VehState_S * const vehicle_state, 
    double max_allowed_ey_,
    double max_allowed_ephi_,
    double max_allowed_curvature_,
    double heading_offset_,
    int heading_lookahead_points_,
    double lateral_offset_,
    std::string slope_file_){

    _p2c = planning2control_msg;
    _vs = vehicle_state;

    max_allowed_ey = max_allowed_ey_;
    max_allowed_ephi = max_allowed_ephi_;
    max_allowed_curvature = max_allowed_curvature_;
    heading_offset = heading_offset_;
    heading_lookahead_points = heading_lookahead_points_;
    lateral_offset = lateral_offset_;

    load_slope(slope_file_);
}

void pathProcessing::load_slope(std::string slope_file){
    slope_map.clear();
    if (slope_file.empty())
        return;

    std::ifstream fin(slope_file);
    if (!fin.is_open()){
        RCLCPP_WARN(rclcpp::get_logger("path_process"), "Open slope file failed: %s", slope_file.c_str());
        return;
    }

    // each line: "(x, y): grade"
    std::string line;
    int x, y;
    double grade;
    while (std::getline(fin, line)){
        if (sscanf(line.c_str(), " (%d , %d ) : %lf", &x, &y, &grade) == 3)
            slope_map[std::make_pair(x, y)] = grade;
    }
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "Loaded %zu slope points from %s", slope_map.size(), slope_file.c_str());
}

double pathProcessing::get_slope(){
    if (slope_map.empty())
        return 0.0;

    auto it = slope_map.find(std::make_pair(int(std::round(_vs->pos_x)), int(std::round(_vs->pos_y))));
    if (it == slope_map.end())
        return 0.0;

    return it->second;
}

void pathProcessing::run(){
    int size = int(_p2c->x_vector.size());
    int closest_index = get_closest_index();

    _p2c->vd = get_desired_velocity(closest_index);
    _p2c->ephi = get_orientation_error(closest_index);
    _p2c->ey = get_lateral_error(closest_index);
    _p2c->slope = get_slope();

    RCLCPP_INFO(rclcpp::get_logger("path_process"), "remaining length %d", size);
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "index %d", closest_index);
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "vd %f", _p2c->vd);
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "vc %f", _vs->speed_x);
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "ephi %f", _p2c->ephi);
    RCLCPP_INFO(rclcpp::get_logger("path_process"), "ey %f", _p2c->ey);
}

int pathProcessing::get_closest_index(){
    int closest_index = -1;

    double distance;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t idx = 1; idx < _p2c->x_vector.size()-1; idx++) {
        // Calculate Euclidean distance
        distance = std::sqrt(
            std::pow(_p2c->x_vector[idx] - _vs->pos_x, 2)
            + std::pow(_p2c->y_vector[idx] - _vs->pos_y, 2)
        );

        // Check if this distance is smaller than current minimum 
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = idx;
        }
    }

    // Look ~1 m behind the closest point to account for the localization difference
    // between the vehicle reference point and autoware (was 5 points at 0.2 m spacing)
    double back_dist = 0.0;
    while (closest_index > 1 && back_dist < 1.0){
        back_dist += std::hypot(_p2c->x_vector[closest_index] - _p2c->x_vector[closest_index - 1],
                                _p2c->y_vector[closest_index] - _p2c->y_vector[closest_index - 1]);
        closest_index--;
    }

    return closest_index;
}

void pathProcessing::process_path(double desired_time_resolution, double preview_time) {
    // fall back to fixed 0.2 m spacing when the time-based parameters are not set
    if (desired_time_resolution <= 0.0 || preview_time <= 0.0){
        resampling();
        return;
    }

    // keep one point every desired_time_resolution seconds of travel (at the desired
    // speed, floored at 1.5 m/s), up to preview_time seconds ahead
    std::vector<float> ds_x, ds_y, ds_vd, ds_ori;
    size_t max_points = size_t(preview_time / desired_time_resolution);
    double accumulated = 0.0;

    for (size_t i = 0; i + 1 < _p2c->x_vector.size(); i++) {
        double dist = std::hypot(_p2c->x_vector[i + 1] - _p2c->x_vector[i],
                                 _p2c->y_vector[i + 1] - _p2c->y_vector[i]);
        accumulated += dist / MAX(1.5, double(_p2c->vd_vector[i]));

        if (accumulated >= desired_time_resolution) {
            ds_x.push_back(_p2c->x_vector[i]);
            ds_y.push_back(_p2c->y_vector[i]);
            ds_vd.push_back(_p2c->vd_vector[i]);
            ds_ori.push_back(_p2c->ori_vector[i]);
            accumulated = 0.0;
        }

        if (ds_x.size() >= max_points)
            break;
    }

    _p2c->x_vector = ds_x;
    _p2c->y_vector = ds_y;
    _p2c->vd_vector = ds_vd;
    _p2c->ori_vector = ds_ori;
}

void pathProcessing::resampling() {
    std::vector<float> resampled_x, resampled_y, resampled_vd, resampled_ori;

    double desired_spacing = 0.2;  // fixed resampling distance
    double dist_since_last = 0.0;  // distance traveled since last resampled point

    // Always include the very first waypoint
    resampled_x.push_back(_p2c->x_vector[0]);
    resampled_y.push_back(_p2c->y_vector[0]);
    resampled_vd.push_back(_p2c->vd_vector[0]);
    resampled_ori.push_back(_p2c->ori_vector[0]);

    for (size_t i = 0; i < _p2c->x_vector.size() - 1; i++) {
        double x0 = _p2c->x_vector[i];
        double y0 = _p2c->y_vector[i];
        double vd0 = _p2c->vd_vector[i];
        double ori0 = _p2c->ori_vector[i];

        double x1 = _p2c->x_vector[i + 1];
        double y1 = _p2c->y_vector[i + 1];
        double vd1 = _p2c->vd_vector[i + 1];
        double ori1 = _p2c->ori_vector[i + 1];

        double dx = x1 - x0;
        double dy = y1 - y0;
        double seg_len = std::sqrt(dx * dx + dy * dy);

        double seg_travel = 0.0;  // how far we’ve walked in this segment

        while (dist_since_last + seg_len - seg_travel >= desired_spacing) {
            // we can fit another resampled point inside this segment
            double remain = desired_spacing - dist_since_last;
            double ratio = (seg_travel + remain) / seg_len;

            double new_x = x0 + ratio * dx;
            double new_y = y0 + ratio * dy;
            double new_vd = vd0 + ratio * (vd1 - vd0);
            double new_ori = ori0 + ratio * (ori1 - ori0);

            resampled_x.push_back(new_x);
            resampled_y.push_back(new_y);
            resampled_vd.push_back(new_vd);
            resampled_ori.push_back(new_ori);

            seg_travel += remain;      // move forward in this segment
            dist_since_last = 0.0;     // reset spacing counter
            x0 = new_x;                // shift starting point
            y0 = new_y;
            vd0 = new_vd;
            ori0 = new_ori;
            dx = x1 - x0;
            dy = y1 - y0;
            seg_len = std::sqrt(dx * dx + dy * dy);
        }

        // Whatever distance is left at the end of this segment
        dist_since_last += seg_len - seg_travel;
    }

    // Always include the last waypoint
    resampled_x.push_back(_p2c->x_vector.back());
    resampled_y.push_back(_p2c->y_vector.back());
    resampled_vd.push_back(_p2c->vd_vector.back());
    resampled_ori.push_back(_p2c->ori_vector.back());

    // Assign back
    _p2c->x_vector = resampled_x;
    _p2c->y_vector = resampled_y;
    _p2c->vd_vector = resampled_vd;
    _p2c->ori_vector = resampled_ori;
}

double pathProcessing::get_desired_velocity(int closest_index){
    // adaptively update desired velocity based on the curernt velocity
    double current_velocity = _vs->speed_x;
    double desired_velocity = _p2c->vd_vector[closest_index];

    // the vehicle is slowing down and will stop very soon
    if (current_velocity <= 0.5 && _p2c->vd_vector.back() <= 0.5){
        return 0.0;
    }

    // find a velocity from future velocities that is close to the current velocity
    if (current_velocity >= desired_velocity){
        double min_difference = std::numeric_limits<double>::max();

        for (size_t i = closest_index; i < _p2c->vd_vector.size(); i++){
            double difference = abs(current_velocity + 0.1 - _p2c->vd_vector[i]);
            if (difference < min_difference){
                min_difference = difference;
                desired_velocity = _p2c->vd_vector[i];
            }
        }
    }

    return desired_velocity;
}

double pathProcessing::get_orientation_error(int closest_index){
    // Calculate vehicle heading
    double roll, pitch, yaw;
    XM::quaternion_to_euler(_vs->qx, _vs->qy, _vs->qz, _vs->qw, roll, pitch, yaw);

    // read the heading of the closest point
    double traj_heading = _p2c->ori_vector[closest_index];

    if (size_t(closest_index + heading_lookahead_points) < _p2c->ori_vector.size()){
        traj_heading = _p2c->ori_vector[closest_index + heading_lookahead_points];
    }

    // Calculate the angle between the vehicle heading and the trajectory heading
    double orientation_error = traj_heading - (yaw + heading_offset);

    //bound heading error between -pi and pi
    if (orientation_error > M_PI) {
        orientation_error -= 2 * M_PI;
    } else if (orientation_error < -M_PI) {
        orientation_error += 2 * M_PI;
    }

    if (orientation_error > max_allowed_ephi){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Orientation error too large: %f radians", orientation_error);
    }

    return orientation_error;
}

double pathProcessing::get_lateral_error(int closest_index){
    double x_pre = _p2c->x_vector[closest_index - 1];
    double y_pre = _p2c->y_vector[closest_index - 1];
    double x_next = _p2c->x_vector[closest_index + 1];
    double y_next = _p2c->y_vector[closest_index + 1];

    // Compute the distance from the pose to the line
    double lateral_error = abs((y_next - y_pre) * _vs->pos_x - (x_next - x_pre) * _vs->pos_y + x_next * y_pre - y_next * x_pre) / 
        sqrt(pow(y_next - y_pre, 2) + pow(x_next - x_pre, 2));

    // Calculate cross product 
    double cross_product = (x_next - x_pre) * (_vs->pos_y - y_pre) - (y_next - y_pre) * (_vs->pos_x - x_pre);
    // Determine if the pose is left or left of the line
    if (cross_product > 0) {
        // Negate lateral_error if pose is to the left of the line
        lateral_error = -lateral_error;
    }
    lateral_error += lateral_offset;
    
    if (fabs(lateral_error) > max_allowed_ey){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Lateral error too large: %f", lateral_error);
    }

    return lateral_error;
}