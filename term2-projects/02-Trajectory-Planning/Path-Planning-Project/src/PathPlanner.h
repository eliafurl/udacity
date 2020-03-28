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
    void upadtePathPlannerState(std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, CarState &ego_car);
    std::vector<double> behaviorPlanner(std::vector<std::vector<double>> &sensor_fusion);
    std::vector<std::vector<double>> trajectoryGeneration(std::vector<double> &goal);
};

#endif // __PATH_PLANNER_H__