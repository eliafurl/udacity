#include <iostream>
#include "PathPlanner.h"
#include "spline.h"
#include "helpers.h"

namespace Constants
{
    const double MAX_VELOCITY = 49.0;       // mph
    const double MAX_ACCELERATION = 0.224;  //
    const double SAFETY_DISTANCE = 30.0;    // m
    const double CYCLE_TIME = 0.02;         // s
    const double MS2MPH = 2.23694;          // -
}

using namespace Constants;

PathPlanner::PathPlanner(std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y, std::vector<double>& map_waypoints_s)
{
    this->ref_vel = 0.0;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}

void PathPlanner::UpadtePathPlannerState(vector<double>& previous_path_x, std::vector<double>& previous_path_y, CarState& ego_car)
{
    this->path_size = previous_path_x.size();
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->ego_car = ego_car;
    this->ego_lane = DetectLaneFromCarPos(ego_car.d);
}

vector<double> PathPlanner::BehaviorPlanner(std::vector<std::vector<double>>& sensor_fusion)
{
    bool should_change_lane = false;
    double front_car_speed;
    double front_car_s = INFINITY;
    vector<vector<double>> cars_left_lane;
    vector<vector<double>> cars_right_lane;

    for (auto detected_car : sensor_fusion)
    {
        // Retrieve state of detected vehicle
        double vx = detected_car[3];
        double vy = detected_car[4];
        double s = detected_car[5];
        double d = detected_car[6];

        // Compute s value for the detected and ego car after step
        double speed = sqrt(vx*vx + vy*vy);
        s += (double) speed * CYCLE_TIME;

        // Check the car lane
        int detected_car_lane = DetectLaneFromCarPos(d);
        bool in_same_lane = this->ego_lane == detected_car_lane;
        bool from_left = this->ego_lane - 1 == detected_car_lane;
        bool from_right = this->ego_lane + 1 == detected_car_lane;

        bool too_close = (s > this->ego_car.s) && (s - this->ego_car.s < SAFETY_DISTANCE);

        if (in_same_lane && too_close)
        {
            should_change_lane = true;
            if (s < front_car_s)
            {
                front_car_speed = speed;
                front_car_s = s;
            }
        }
        else if (from_left)
        {
            cars_left_lane.push_back(detected_car);
        }
        else if (from_right)
        {
            cars_right_lane.push_back(detected_car);
        }
    }
    // Try changing lane, if not possible slow down
    if (should_change_lane)
    {
        return TryChangingLane(cars_left_lane, cars_right_lane, front_car_speed);
    }
    // Stay on the same lane and keep going forward
    else
    {
        UpdateSpeed();
        return {this->ego_car.s + SAFETY_DISTANCE, 2+4*this->ego_lane, this->ref_vel};
    }
}

vector<double> PathPlanner::TryChangingLane(std::vector<std::vector<double>>& cars_left_lane,
                                            std::vector<std::vector<double>>& cars_right_lane, double front_car_speed)
{
    // Check lanes
    bool is_left_lane_safe = CheckLane(cars_left_lane);
    bool is_right_lane_safe = CheckLane(cars_right_lane);
    double best = BestLane(cars_left_lane, cars_right_lane);

    // Can't go left, try right
    if ((this->ego_lane == 0 || (this->ego_lane == 1 && !is_left_lane_safe)) && is_right_lane_safe)
    {
        UpdateSpeed();
        return {this->ego_car.s+1.5*SAFETY_DISTANCE, 2+4*(this->ego_lane+1), this->ref_vel};
    }
    // Try best lane
    else if (this->ego_lane == 1 && is_left_lane_safe && is_right_lane_safe)
    {
        UpdateSpeed();
        return {this->ego_car.s+1.5*SAFETY_DISTANCE, 2+4*best, this->ref_vel};
    }
    // Can't go right, try left
    else if ((this->ego_lane == 2 || (this->ego_lane == 1 && !is_right_lane_safe)) && is_left_lane_safe)
    {
        UpdateSpeed();
        return {this->ego_car.s+1.5*SAFETY_DISTANCE, 2+4*(this->ego_lane-1), this->ref_vel};
    }
    // Otherwise, slow down
    else
    {
        UpdateSpeed(front_car_speed);
        return {this->ego_car.s+SAFETY_DISTANCE, 2+4*this->ego_lane, this->ref_vel};
    }
}

bool PathPlanner::CheckLane(std::vector<std::vector<double>>& cars_in_lane)
{
    // Check all cars from the desired lane
    for (auto detected_car : cars_in_lane)
    {
        // Retrieve detected car's state
        double vx = detected_car[3];
        double vy = detected_car[4];
        double s = detected_car[5];

        double speed = sqrt(vx*vx + vy*vy);
        s += (double) speed * CYCLE_TIME;

        // Check for cars in front of ego car
        if ((s >= this->ego_car.s) && (s - this->ego_car.s <= 2.0*SAFETY_DISTANCE))
        {
            return false;
        }

        // Check for cars behind of ego car
        double min_dist = 0.25 * SAFETY_DISTANCE;
        double max_dist = 1.0 * SAFETY_DISTANCE;
        double dist = (1 - this->ref_vel/MAX_VELOCITY) * max_dist + min_dist;
        double dist_to_car = this->ego_car.s - s;
        if ((s <= this->ego_car.s) && (dist_to_car <= dist))
        {
            return false;
        }
    }

  // It's safe!
  return true;
}

int PathPlanner::BestLane(std::vector<std::vector<double>>& cars_left_lane, std::vector<std::vector<double>>& cars_right_lane)
{
    double closest_left_car_dist = INFINITY;
    double closest_right_car_dist = INFINITY;

    // Find left lane closest car
    for (auto detected_car_left : cars_left_lane)
    {
        // Retrieve detected car's state
        double vx = detected_car_left[3];
        double vy = detected_car_left[4];
        double s = detected_car_left[5];

        double speed = sqrt(vx*vx + vy*vy);
        s += (double) speed * CYCLE_TIME;

        if ((s > this->ego_car.s) && (s - this->ego_car.s < closest_left_car_dist))
        {
            closest_left_car_dist = s - this->ego_car.s;
        }
    }

    // Find right lane closest car
    for (auto detected_car_right : cars_right_lane)
    {
        // Retrieve detected car's state
        double vx = detected_car_right[3];
        double vy = detected_car_right[4];
        double s = detected_car_right[5];

        double speed = sqrt(vx*vx + vy*vy);
        s += (double) speed * CYCLE_TIME;

        if ((s > this->ego_car.s) && (s - this->ego_car.s < closest_right_car_dist))
        {
            closest_right_car_dist = s - this->ego_car.s;
        }
    }

    // Always go left, if cars are far away in front of ego car
    // Or when the left front car is farest than the right front car
    if ((closest_left_car_dist >= 3*SAFETY_DISTANCE && closest_right_car_dist >= 3*SAFETY_DISTANCE) ||
        closest_left_car_dist >= closest_right_car_dist)
    {
        return 0;
    }
    // Otherwise, go right
    else
    {
        return 2;
    }
}

void PathPlanner::UpdateSpeed(double speed_to_match)
{
    // By default speed up
    double updated_speed = this->ref_vel + MAX_ACCELERATION;

    // Otherwise, try to catch lane speed
    if (speed_to_match != -1)
    {
        speed_to_match *= MS2MPH * .98; // mph // .98 to prevent overshooting front car speed

        // Slow down to match lane speed
        if (MAX_ACCELERATION <= this->ref_vel)
        {
            updated_speed = std::max(speed_to_match, this->ref_vel - 1.5*MAX_ACCELERATION);
        }
        // Accelerate to match lane speed
        else
        {
            updated_speed = std::min(speed_to_match, this->ref_vel + MAX_ACCELERATION);
        }
    }

    // Check to not exceed speed limit
    this->ref_vel = std::min(MAX_VELOCITY, updated_speed);

}

std::vector<std::vector<double>> PathPlanner::TrajectoryGeneration(std::vector<double>& goal)
{

        std::vector<double> ptsx;
        std::vector<double> ptsy;
        double ref_x = this->ego_car.x;
        double ref_y = this->ego_car.y;
        double ref_yaw = deg2rad(this->ego_car.yaw);

        if (this->path_size < 2)
        {
            double prev_car_x = this->ego_car.x - cos(this->ego_car.yaw);
            double prev_car_y = this->ego_car.y - sin(this->ego_car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(this->ego_car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(this->ego_car.y);

        }
        else
        {
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

        std::vector<double> next_wp0 = getXY(goal[0], goal[1], this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);

        ptsx.push_back(next_wp0[0]);
        ptsy.push_back(next_wp0[1]);

        for (int i=0; i < ptsx.size(); i++)
        {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
        }

        tk::spline s;
        s.set_points(ptsx, ptsy);

        std::vector<double> next_trajectory_x;
        std::vector<double> next_trajectory_y;

        for (int i=0; i < this->path_size; i++)
        {
            next_trajectory_x.push_back(previous_path_x[i]);
            next_trajectory_y.push_back(previous_path_y[i]);
        }

        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);
        double N = (target_dist/(CYCLE_TIME*goal[2]/MS2MPH));

        double x_add_on = 0.0;

        for (int i = 1; i <= 50 - this->path_size; i++)
        {
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

        for (int i = 0; i < next_trajectory_x.size(); i++)
        {
            std::cout << "next x[" << i << "] = " << next_trajectory_x[i] << std::endl;
            std::cout << "next y[" << i << "] = " << next_trajectory_y[i] << std::endl;
        }

    std::vector<std::vector<double>> next_trajectory = {next_trajectory_x, next_trajectory_y};
    return next_trajectory;
}