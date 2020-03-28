#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>

struct CarState {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

class PathPlanner {
private:
    double ref_vel;
    double path_size;

    CarState ego_car;
    double ego_lane;
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;

public:
    PathPlanner(std::vector<double> &map_waypoints_x, std::vector<double> &map_waypoints_y, std::vector<double> &map_waypoints_s);
    ~PathPlanner() {};

    void UpadtePathPlannerState(std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, CarState &ego_car);

    std::vector<double> TryChangingLane(std::vector<std::vector<double>> &cars_left_lane,
                                        std::vector<std::vector<double>> &cars_right_lane, double front_car_speed);

    bool CheckLane(std::vector<std::vector<double>> &cars_in_lane);

    int BestLane(std::vector<std::vector<double>> &cars_left_lane, std::vector<std::vector<double>> &cars_right_lane);

    void UpdateSpeed(double speed_to_match = -1.0);

    std::vector<double> BehaviorPlanner(std::vector<std::vector<double>> &sensor_fusion);

    std::vector<std::vector<double>> TrajectoryGeneration(std::vector<double> &goal);
};

#endif // __PATH_PLANNER_H__