#include "PathPlanner.h"
#include "spline.h"
#include "helpers.h"

PathPlanner::PathPlanner(std::vector<double> &map_waypoints_x, std::vector<double> &map_waypoints_y, std::vector<double> &map_waypoints_s) {
    this->ref_vel = 0.0;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}

void PathPlanner::upadtePathPlannerState(vector<double> &previous_path_x, std::vector<double> &previous_path_y, CarState &ego_car) {
    this->path_size = previous_path_x.size();
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->ego_car = ego_car;
    this->ego_lane = DetectLaneFromCarPos(ego_car.d);
}

vector<double> PathPlanner::behaviorPlanner(std::vector<std::vector<double>> &sensor_fusion) {

    bool too_close = false;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        float d = sensor_fusion[i][6];
        // check if car is in ego lane
        if (d < (2 + 4*this->ego_lane + 2) && d > (2 + 4*this->ego_lane - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)this->path_size*0.2*check_speed);
            // if the car in front is closer then 30m -> do something
            if ((check_car_s > this->ego_car.s) && ((check_car_s - this->ego_car.s) < 30)) {
            //ref_vel = 29.5;
            // too_close = true;
                if (this->ego_lane > 0) {
                    this->ego_lane = 0;
                }
            }
        }
    }

    if (too_close) {
        this->ref_vel -= 0.224;
    } else if (this->ref_vel < 49.5) {
        this->ref_vel += 0.224;
    }

    std::vector<double> goal = {0.0, 0.0};
    return goal;
}

std::vector<std::vector<double>> PathPlanner::trajectoryGeneration(std::vector<double> &goal) {

        std::vector<double> ptsx;
        std::vector<double> ptsy;
        double ref_x = this->ego_car.x;
        double ref_y = this->ego_car.y;
        double ref_yaw = deg2rad(this->ego_car.yaw);

        if (this->path_size < 2) {
            double prev_car_x = this->ego_car.x - cos(this->ego_car.yaw);
            double prev_car_y = this->ego_car.y - sin(this->ego_car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(this->ego_car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(this->ego_car.y);

        } else {
            ref_x = previous_path_x[this->path_size-1];
            ref_y = previous_path_y[this->path_size-1];

            double ref_x_prev = previous_path_x[this->path_size-2];
            double ref_y_prev = previous_path_y[this->path_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
        }

        std::vector<double> next_wp0 = getXY(this->ego_car.s+30, 2+4*this->ego_lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
        std::vector<double> next_wp1 = getXY(this->ego_car.s+60, 2+4*this->ego_lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
        std::vector<double> next_wp2 = getXY(this->ego_car.s+90, 2+4*this->ego_lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);

        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);

        for (int i=0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
        }

        tk::spline s;

        s.set_points(ptsx, ptsy);

        std::vector<double> next_trajectory_x;
        std::vector<double> next_trajectory_y;

        for (int i=0; i < previous_path_x.size(); i++) {
            next_trajectory_x.push_back(previous_path_x[i]);
            next_trajectory_y.push_back(previous_path_y[i]);
        }

        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);

        double x_add_on = 0.0;

        for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_trajectory_x.push_back(x_point);
            next_trajectory_y.push_back(y_point);
        }

        for (int i = 0; i < next_trajectory_x.size(); i++) {
            std::cout << "next x[" << i << "] = " << next_trajectory_x[i] << std::endl;
            std::cout << "next y[" << i << "] = " << next_trajectory_y[i] << std::endl;
        }

    std::vector<std::vector<double>> next_trajectory = {next_trajectory_x, next_trajectory_y};
    return next_trajectory;
}